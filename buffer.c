/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014-2015 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * */

#include "iio-private.h"

#include <errno.h>
#include <string.h>
#ifdef _WIN32
#include <io.h>
#include <winsock2.h>
#else
#ifdef WITH_SPLICE
#include <fcntl.h>
#include <poll.h>
#endif
#include <unistd.h>
#include <sys/socket.h>
#endif

#ifndef _WIN32
#define _read(x, y, z) read(x, y, z)
#define _write(x, y, z) write(x, y, z)

/* winsock2.h defines ERROR, we don't want that */
#undef ERROR
#endif

#include "debug.h"

/* Forward declaration */
static ssize_t foreach_sample(struct iio_buffer *buffer,
		ssize_t (*callback)(const struct iio_channel *,
			void *, size_t, void *), void *d);

struct callback_wrapper_data {
	ssize_t (*callback)(const struct iio_channel *, void *, size_t, void *);
	void *data;
	uint32_t *mask;
};

static bool device_is_high_speed(const struct iio_device *dev)
{
	/* Little trick: We call the backend's get_buffer() function, which is
	 * for now only implemented in the Local backend, with a NULL pointer.
	 * It will return -ENOSYS if the device is not high speed, and either
	 * -EBADF or -EINVAL otherwise. */
	const struct iio_backend_ops *ops = dev->ctx->ops;
	return !!ops->get_buffer &&
		(ops->get_buffer(dev, NULL, 0, NULL, 0) != -ENOSYS);
}

static struct iio_buffer * create_buffer(const struct iio_device *dev,
		size_t samples_count, bool cyclic, bool for_splice)
{
	int ret = -EINVAL;
	struct iio_buffer *buf;
	unsigned int sample_size = iio_device_get_sample_size(dev);
	if (!sample_size)
		goto err_set_errno;

	buf = malloc(sizeof(*buf));
	if (!buf) {
		ret = -ENOMEM;
		goto err_set_errno;
	}

	buf->dev_sample_size = sample_size;
	buf->length = sample_size * samples_count;
	buf->dev = dev;
	buf->mask = calloc(dev->words, sizeof(*buf->mask));
	if (!buf->mask) {
		ret = -ENOMEM;
		goto err_free_buf;
	}

	/* Set the default channel mask to the one used by the device.
	 * While input buffers will erase this as soon as the refill function
	 * is used, it is useful for output buffers, as it permits
	 * iio_buffer_foreach_sample to be used. */
	memcpy(buf->mask, dev->mask, dev->words * sizeof(*buf->mask));

	ret = iio_device_open(dev, samples_count, cyclic, for_splice);
	if (ret < 0)
		goto err_free_mask;

	if (for_splice) {
		ret = iio_device_get_splice_fd(dev);
		if (ret < 0)
			goto err_free_mask;

		buf->splice_fd = ret;
#ifdef WITH_SPLICE
		ret = pipe(buf->pipefd);
		if (ret < 0) {
			ret = -errno;
			goto err_free_mask;
		}
#endif
	}

	/* We default to true here. If one I/O call fails with -ENOTSOCK, we
	 * will set this to false and revert to regular I/O calls. */
	buf->splice_fd_is_socket = true;

	buf->use_splice = for_splice;

	buf->dev_is_high_speed = !for_splice && device_is_high_speed(dev);
	if (buf->dev_is_high_speed) {
		/* Dequeue the first buffer, so that buf->buffer is correctly
		 * initialized */
		buf->buffer = NULL;
		if (iio_device_is_tx(dev)) {
			ret = dev->ctx->ops->get_buffer(dev, &buf->buffer,
					buf->length, buf->mask, dev->words);
			if (ret < 0)
				goto err_close_device;
		}
	} else {
		buf->buffer = malloc(buf->length);
		if (!buf->buffer) {
			ret = -ENOMEM;
			goto err_close_device;
		}
	}

	buf->sample_size = iio_device_get_sample_size_mask(dev,
			buf->mask, dev->words);
	buf->data_length = buf->length;
	return buf;

err_close_device:
	iio_device_close(dev);
err_free_mask:
	free(buf->mask);
err_free_buf:
	free(buf);
err_set_errno:
	errno = -ret;
	return NULL;
}

struct iio_buffer * iio_device_create_buffer(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
{
	return create_buffer(dev, samples_count, cyclic, false);
}

struct iio_buffer * iio_device_create_splice_buffer(
		const struct iio_device *dev,
		size_t samples_count)
{
	return create_buffer(dev, samples_count, false, true);
}

void iio_buffer_destroy(struct iio_buffer *buffer)
{
	iio_device_close(buffer->dev);
	if (!buffer->use_splice && !buffer->dev_is_high_speed)
		free(buffer->buffer);
#ifdef WITH_SPLICE
	if (buffer->use_splice) {
		close(buffer->pipefd[0]);
		close(buffer->pipefd[1]);
	}
#endif
	free(buffer->mask);
	free(buffer);
}

int iio_buffer_get_poll_fd(struct iio_buffer *buffer)
{
	return iio_device_get_poll_fd(buffer->dev);
}

int iio_buffer_set_blocking_mode(struct iio_buffer *buffer, bool blocking)
{
	return iio_device_set_blocking_mode(buffer->dev, blocking);
}

ssize_t iio_buffer_refill(struct iio_buffer *buffer)
{
	ssize_t read;
	const struct iio_device *dev = buffer->dev;

	if (buffer->use_splice)
		return -EPERM;

	if (buffer->dev_is_high_speed) {
		read = dev->ctx->ops->get_buffer(dev, &buffer->buffer,
				buffer->length, buffer->mask, dev->words);
	} else {
		read = iio_device_read_raw(dev, buffer->buffer, buffer->length,
				buffer->mask, dev->words);
	}

	if (read >= 0) {
		buffer->data_length = read;
		buffer->sample_size = iio_device_get_sample_size_mask(dev,
				buffer->mask, dev->words);
	}
	return read;
}

ssize_t iio_buffer_push(struct iio_buffer *buffer)
{
	const struct iio_device *dev = buffer->dev;
	ssize_t ret;

	if (buffer->use_splice)
		return -EPERM;

	if (buffer->dev_is_high_speed) {
		void *buf;
		ret = dev->ctx->ops->get_buffer(dev, &buf,
				buffer->data_length, buffer->mask, dev->words);
		if (ret >= 0)
			buffer->buffer = buf;
	} else {
		void *ptr = buffer->buffer;
		size_t tmp_len;

		/* iio_device_write_raw doesn't guarantee that all bytes are
		 * written */
		for (tmp_len = buffer->data_length; tmp_len; ) {
			ret = iio_device_write_raw(dev, ptr, tmp_len);
			if (ret < 0)
				goto out_reset_data_length;

			tmp_len -= ret;
			ptr = (void *) ((uintptr_t) ptr + ret);
		}

		ret = (ssize_t) buffer->data_length;
	}

out_reset_data_length:
	buffer->data_length = buffer->length;
	return ret;
}

ssize_t iio_buffer_push_partial(struct iio_buffer *buffer, size_t samples_count)
{
	size_t new_len = samples_count * buffer->dev_sample_size;

	if (new_len == 0 || new_len > buffer->length)
		return -EINVAL;

	buffer->data_length = new_len;
	return iio_buffer_push(buffer);
}

static bool iio_buffer_mask_match(const struct iio_buffer *buf)
{
	unsigned int i;

	for (i = 0; i < buf->dev->words; i++)
		if (buf->mask[i] != buf->dev->mask[i])
			return false;

	return true;
}

static ssize_t buffer_transfer(int fd, void *buffer, size_t len,
		bool is_socket, bool is_tx)
{
	uintptr_t ptr = (uintptr_t) buffer;

	while (len) {
		ssize_t ret;

		if (is_tx) {
			if (is_socket)
				ret = send(fd, (const void *) ptr, len, 0);
			else
				ret = _write(fd, (const void *) ptr, len);
		} else {
			if (is_socket)
				ret = recv(fd, (void *) ptr, len, 0);
			else
				ret = _read(fd, (void *) ptr, len);
		}

		if (ret < 0) {
#ifdef _WIN32
			int error = WSAGetLastError();
#else
			int error = errno;
#endif
			if (error == EINTR)
				continue;

			return (ssize_t) -error;
		}

		ptr += ret;
		len -= ret;
	}

	return (ssize_t) (ptr - (uintptr_t) buffer);
}

static ssize_t iio_buffer_transfer_hw(struct iio_buffer *buf,
		size_t len, bool is_tx)
{
	ssize_t ret;

	ret = buffer_transfer(buf->splice_fd, buf->buffer, len,
			buf->splice_fd_is_socket, is_tx);
	if (ret == -ENOTSOCK) {
		buf->splice_fd_is_socket = false;
		ret = buffer_transfer(buf->splice_fd, buf->buffer, len,
				buf->splice_fd_is_socket, is_tx);
	}

	return ret;
}

static ssize_t iio_buffer_transfer_fd(struct iio_buffer *buf,
		int fd, size_t len, bool is_tx)
{
	ssize_t ret;

	ret = buffer_transfer(fd, buf->buffer, len, true, is_tx);
	if (ret == -ENOTSOCK)
		ret = buffer_transfer(fd, buf->buffer, len, false, is_tx);

	return ret;
}

struct buffer_extract_params {
	/* 16 bytes for a sample of one channel, should be enough */
	char zeros[16];
	size_t last_sample_size;
	bool is_socket;
	int fd;
};

static ssize_t extract_sample_cb(const struct iio_channel *chn,
		void *buf, size_t len, void *d)
{
	struct buffer_extract_params *params = d;
	ssize_t ret;

	/* Transfer some dummy data for alignment purposes, if needed. */
	if (params->last_sample_size % len) {
		size_t nb_zeros = len - (params->last_sample_size % len);

		ret = buffer_transfer(params->fd, params->zeros, nb_zeros,
				params->is_socket, true);
		if (ret == -ENOTSOCK) {
			params->is_socket = false;
			ret = buffer_transfer(params->fd, params->zeros,
					nb_zeros, false, true);
		}
	}

	ret = buffer_transfer(params->fd, buf, len,
			params->is_socket, true);
	if (ret == -ENOTSOCK) {
		params->is_socket = false;
		ret = buffer_transfer(params->fd, buf, len, false, true);
	}

	params->last_sample_size = len;
	return ret;
}

static ssize_t iio_buffer_extract(struct iio_buffer *buf, int fd, size_t len)
{
	uintptr_t ptr = (uintptr_t) buf->buffer;
	bool is_tx = iio_device_is_tx(buf->dev);
	bool is_socket = true;
	ssize_t ret;
	struct buffer_extract_params extract_params;

	if (is_tx) {
		/* This should never happen. */
		ERROR("Mask mismatch detected for a TX device!\n");
		return -EIO;
	}

	memset(extract_params.zeros, 0, sizeof(extract_params.zeros));
	extract_params.is_socket = true;
	extract_params.fd = fd;
	extract_params.last_sample_size = 0;

	ret = iio_buffer_transfer_hw(buf, len, false);
	if (ret < 0)
		return ret;

	buf->data_length = (size_t) ret;
	return foreach_sample(buf, extract_sample_cb, &extract_params);
}

static ssize_t iio_buffer_copy(struct iio_buffer *buf, int fd, size_t len)
{
	uintptr_t ptr = (uintptr_t) buf->buffer;
	bool is_tx = iio_device_is_tx(buf->dev);
	bool is_socket = true;
	ssize_t ret;

	if (is_tx)
		ret = iio_buffer_transfer_fd(buf, fd, len, false);
	else
		ret = iio_buffer_transfer_hw(buf, len, false);
	if (ret < 0)
		return ret;

	if (is_tx)
		ret = iio_buffer_transfer_hw(buf, len, true);
	else
		ret = iio_buffer_transfer_fd(buf, fd, len, true);
	return ret;
}

#ifdef WITH_SPLICE
static ssize_t iio_buffer_do_splice(struct iio_buffer *buf, int fd, size_t len)
{
	bool is_tx = iio_device_is_tx(buf->dev);
	int fd_in, fd_out;
	ssize_t ret, read_len = len;

	fd_in = is_tx ? fd : buf->splice_fd;
	fd_out = is_tx ? buf->splice_fd : fd;

	do {
		size_t splice_len, to_splice;
		struct pollfd pfd = {
			.fd = fd_in,
			.events = POLLIN,
		};

		/* Wait for the input file descriptor to be ready */
		ret = (ssize_t) poll(&pfd, 1, 0);
		if (ret < 0)
			return (ssize_t) -errno;

		/*
		 * SPLICE_F_NONBLOCK is just here to avoid a deadlock when
		 * splicing to the pipe, that happens because the other end of
		 * the pipe is not yet connected.
		 *
		 * As we first poll the input file descriptor, it should never
		 * return -EAGAIN.
		 */
		ret = splice(fd_in, NULL, buf->pipefd[1], NULL, len,
				SPLICE_F_MOVE | SPLICE_F_NONBLOCK);
		if (!ret)
			return -EIO;
		if (ret < 0)
			return -errno;

		to_splice = (size_t) ret;

		for (splice_len = to_splice; splice_len;
				splice_len -= (size_t) ret) {
			ret = splice(buf->pipefd[0], NULL,
					fd_out, NULL, splice_len,
					SPLICE_F_MOVE | SPLICE_F_MORE);
			if (!ret)
				return -EIO;
			if (ret < 0)
				return -errno;
		}

		len -= to_splice;
	} while (len);

	/* Splice 0 bytes without the SPLICE_F_MORE flag, to tell the kernel
	 * that we won't splice anymore */
	splice(buf->pipefd[0], NULL, fd_out, NULL, 0, SPLICE_F_MOVE);

	return read_len;
}
#endif

static ssize_t iio_buffer_zerocopy(struct iio_buffer *buffer,
		int fd, size_t len, bool mask_match)
{
	if (!mask_match)
		return iio_buffer_extract(buffer, fd, len);
#ifdef WITH_SPLICE
	else if (!isatty(fd))
		return iio_buffer_do_splice(buffer, fd, len);
#endif
	else
		return iio_buffer_copy(buffer, fd, len);
}

ssize_t iio_buffer_splice(struct iio_buffer *buffer, int fd, size_t len)
{
	const struct iio_device *dev = buffer->dev;
	const struct iio_backend_ops *ops = dev->ctx->ops;
	size_t nb_bytes = 0;
	ssize_t ret;
	bool mask_match;

	if (len > buffer->length)
		return -EINVAL;

	if (!buffer->use_splice)
		return -EPERM;

	if (ops->before_splice) {
		ret = ops->before_splice(dev, len, buffer->mask, dev->words);
		if (ret < 0)
			return ret;

		len = (size_t) ret;
	}

	/* Only allow splicing if the channel mask we get is the one we want */
	mask_match = iio_buffer_mask_match(buffer);

	while (len) {
		ssize_t splice_len;

		if (ops->get_splice_len) {
			splice_len = ops->get_splice_len(dev, len);
			if (splice_len < 0) {
				ret = splice_len;
				break;
			}
		} else {
			splice_len = len;
		}

		ret = iio_buffer_zerocopy(buffer, fd,
				(size_t) splice_len, mask_match);
		if (ret < 0)
			break;

		nb_bytes += (size_t) ret;
		len -= (size_t) ret;
	}

	if (ops->after_splice)
		ops->after_splice(dev);

err_return:
	return ret < 0 ? ret : nb_bytes;
}

ssize_t iio_buffer_foreach_sample(struct iio_buffer *buffer,
		ssize_t (*callback)(const struct iio_channel *,
			void *, size_t, void *), void *d)
{
	if (buffer->use_splice)
		return -EPERM;
	else
		return foreach_sample(buffer, callback, d);
}

static ssize_t foreach_sample(struct iio_buffer *buffer,
		ssize_t (*callback)(const struct iio_channel *,
			void *, size_t, void *), void *d)
{
	uintptr_t ptr = (uintptr_t) buffer->buffer,
		  end = ptr + buffer->data_length;
	const struct iio_device *dev = buffer->dev;
	ssize_t processed = 0;

	if (buffer->sample_size <= 0)
		return -EINVAL;

	if (buffer->data_length < buffer->dev_sample_size)
		return 0;

	while (end - ptr >= (size_t) buffer->sample_size) {
		unsigned int i;

		for (i = 0; i < dev->nb_channels; i++) {
			const struct iio_channel *chn = dev->channels[i];
			unsigned int length = chn->format.length / 8;

			if (chn->index < 0)
				break;

			/* Test if the buffer has samples for this channel */
			if (!TEST_BIT(buffer->mask, chn->index))
				continue;

			if (ptr % length)
				ptr += length - (ptr % length);

			/* Test if the client wants samples from this channel */
			if (TEST_BIT(dev->mask, chn->index)) {
				ssize_t ret = callback(chn,
						(void *) ptr, length, d);
				if (ret < 0)
					return ret;
				else
					processed += ret;
			}

			ptr += length;
		}
	}
	return processed;
}

void * iio_buffer_start(const struct iio_buffer *buffer)
{
	return buffer->buffer;
}

void * iio_buffer_first(const struct iio_buffer *buffer,
		const struct iio_channel *chn)
{
	size_t len;
	unsigned int i;
	uintptr_t ptr = (uintptr_t) buffer->buffer;

	if (!iio_channel_is_enabled(chn))
		return iio_buffer_end(buffer);

	for (i = 0; i < buffer->dev->nb_channels; i++) {
		struct iio_channel *cur = buffer->dev->channels[i];
		len = cur->format.length / 8;

		/* NOTE: dev->channels are ordered by index */
		if (cur->index < 0 || cur->index == chn->index)
			break;

		/* Test if the buffer has samples for this channel */
		if (!TEST_BIT(buffer->mask, cur->index))
			continue;

		if (ptr % len)
			ptr += len - (ptr % len);
		ptr += len;
	}

	len = chn->format.length / 8;
	if (ptr % len)
		ptr += len - (ptr % len);
	return (void *) ptr;
}

ptrdiff_t iio_buffer_step(const struct iio_buffer *buffer)
{
	return (ptrdiff_t) buffer->sample_size;
}

void * iio_buffer_end(const struct iio_buffer *buffer)
{
	return (void *) ((uintptr_t) buffer->buffer + buffer->data_length);
}

void iio_buffer_set_data(struct iio_buffer *buf, void *data)
{
	buf->userdata = data;
}

void * iio_buffer_get_data(const struct iio_buffer *buf)
{
	return buf->userdata;
}

const struct iio_device * iio_buffer_get_device(const struct iio_buffer *buf)
{
	return buf->dev;
}
