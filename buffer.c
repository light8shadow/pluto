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
#include <unistd.h>
#include <sys/socket.h>
#endif

#ifndef _WIN32
#define _read(x, y, z) read(x, y, z)
#define _write(x, y, z) write(x, y, z)
#endif

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

struct iio_buffer * iio_device_create_buffer(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
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

	ret = iio_device_open(dev, samples_count, cyclic);
	if (ret < 0)
		goto err_free_mask;

	buf->dev_is_high_speed = device_is_high_speed(dev);
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

void iio_buffer_destroy(struct iio_buffer *buffer)
{
	iio_device_close(buffer->dev);
	if (!buffer->dev_is_high_speed)
		free(buffer->buffer);
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
		buffer->spliced = false;
	}
	return read;
}

ssize_t iio_buffer_push(struct iio_buffer *buffer)
{
	const struct iio_device *dev = buffer->dev;

	if (buffer->dev_is_high_speed) {
		void *buf;
		ssize_t ret = dev->ctx->ops->get_buffer(dev,
				&buf, buffer->length, buffer->mask, dev->words);
		if (ret >= 0) {
			buffer->buffer = buf;
			buffer->spliced = false;
		}
		return ret;
	} else {
		size_t length = buffer->length;
		void *ptr = buffer->buffer;

		/* iio_device_write_raw doesn't guarantee that all bytes are
		 * written */
		while (length) {
			ssize_t ret = iio_device_write_raw(dev, ptr, length);
			if (ret < 0)
				return ret;

			length -= ret;
			ptr = (void *) ((uintptr_t) ptr + ret);
		}

		buffer->spliced = false;
		return (ssize_t) buffer->length;
	}
}

static ssize_t iio_buffer_copy(struct iio_buffer *buf, int fd, size_t len)
{
	uintptr_t ptr = (uintptr_t) buf->buffer;
	bool is_tx = iio_device_is_tx(buf->dev);
	bool is_socket = true;

	while (len) {
		ssize_t ret;

		if (is_tx) {
			if (is_socket)
				ret = recv(fd, (void *) ptr, len, 0);
			else
				ret = _read(fd, (void *) ptr, len);
		} else {
			if (is_socket)
				ret = send(fd, (const void *) ptr, len, 0);
			else
				ret = _write(fd, (const void *) ptr, len);
		}

		if (ret < 0) {
#ifdef _WIN32
			int error = WSAGetLastError();
#else
			int error = errno;
#endif
			if (error == ENOTSOCK) {
				is_socket = false;
				continue;
			}
			if (error == EINTR)
				continue;
			return (ssize_t) -error;
		}

		ptr += ret;
		len -= ret;
	}

	return (ssize_t) (ptr - (uintptr_t) buf->buffer);
}

#ifdef WITH_SPLICE
static ssize_t iio_buffer_zerocopy(struct iio_buffer *buf, int fd, size_t len)
{
	ssize_t ret;
	bool is_tx = iio_device_is_tx(buf->dev);
	int fd_dev, fd_in, fd_out;

	fd_dev = iio_device_get_splice_fd(buf->dev);
	if (fd_dev < 0)
		return (ssize_t) fd_dev;

	fd_in = is_tx ? fd : fd_dev;
	fd_out = is_tx ? fd_dev : fd;

	return iio_splice(fd_out, fd_in, len);
}
#endif

ssize_t iio_buffer_splice(struct iio_buffer *buffer, int fd, size_t len)
{
	ssize_t ret = -ENOSYS;

	if (buffer->spliced)
		return -EBUSY;

	if (len > buffer->length)
		return -EINVAL;

#ifdef WITH_SPLICE
	if (!isatty(fd))
		ret = iio_buffer_zerocopy(buffer, fd, len);
#endif
	if (ret == -ENOSYS)
		ret = iio_buffer_copy(buffer, fd, len);

	if (ret > 0)
		buffer->spliced = true;
	return ret;
}

ssize_t iio_buffer_foreach_sample(struct iio_buffer *buffer,
		ssize_t (*callback)(const struct iio_channel *,
			void *, size_t, void *), void *d)
{
	uintptr_t ptr = (uintptr_t) buffer->buffer,
		  end = ptr + buffer->data_length;
	const struct iio_device *dev = buffer->dev;
	ssize_t processed = 0;

	if (buffer->sample_size <= 0)
		return -EINVAL;

	if (buffer->spliced)
		return -EBUSY;

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
