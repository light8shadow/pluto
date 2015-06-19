/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2015 Analog Devices, Inc.
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

#include <iio.hpp>

#include <iio.h>

using namespace iio;

Context::Context(struct iio_context *ctx) :
	ctx(ctx),
	name(std::string(iio_context_get_name(ctx))),
	description(std::string(iio_context_get_description(ctx))),
	xml(std::string(iio_context_get_xml(ctx))),
	devices_count(iio_context_get_devices_count(ctx))
{
	/* TODO: Exception if !ctx */
}

Context::Context() :
	Context(iio_create_default_context())
{
}

Context::Context(std::string &hostname) :
	Context(iio_create_network_context(hostname.c_str()))
{
}

Context::Context(Context &ctx) :
	Context(iio_context_clone(ctx.ctx))
{
}

Context::~Context()
{
	iio_context_destroy(this->ctx);
}

int Context::set_timeout(unsigned int timeout_ms)
{
	return iio_context_set_timeout(ctx, timeout_ms);
}

Device::Device(std::shared_ptr<Context> &ctx, struct iio_device *dev) :
	ctx(ctx),
	dev(dev),
	id(std::string(iio_device_get_id(dev))),
	name(std::string(iio_device_get_name(dev) ?: "")),
	channels_count(iio_device_get_channels_count(dev)),
	attrs_count(iio_device_get_attrs_count(dev)),
	debug_attrs_count(iio_device_get_debug_attrs_count(dev))
{
}

Device::Device(std::shared_ptr<Context> &ctx, unsigned int id) :
	Device(ctx, iio_context_get_device(ctx.get()->ctx, id))
{
}

Device::Device(std::shared_ptr<Context> &ctx, std::string &id) :
	Device(ctx, iio_context_find_device(ctx.get()->ctx, id.c_str()))
{
}

ssize_t Device::get_sample_size()
{
	return iio_device_get_sample_size(this->dev);
}

int Device::reg_read(uint32_t address, uint32_t *value)
{
	return iio_device_reg_read(dev, address, value);
}

int Device::reg_write(uint32_t address, uint32_t value)
{
	return iio_device_reg_write(dev, address, value);
}

ssize_t Buffer::iio_buffer_foreach_sample_cb(
		const struct iio_channel *chn,
		void *src, size_t bytes, void *d)
{
	Buffer *buf = static_cast<Buffer *>(d);
	long index = iio_channel_get_index(chn);

	Channel channel(buf->dev, index);
	return buf->cb(channel, src, bytes);
}

Buffer::Buffer(std::shared_ptr<Device> &dev,
		size_t samples_count, bool cyclic) :
	dev(dev),
	samples_count(samples_count),
	cyclic(cyclic),
	buf(iio_device_create_buffer(dev.get()->dev, samples_count, cyclic))
{
	/* TODO: Exception if !buf */
}

Buffer::~Buffer()
{
	iio_buffer_destroy(this->buf);
}

ssize_t Buffer::refill()
{
	return iio_buffer_refill(this->buf);
}

ssize_t Buffer::push()
{
	return iio_buffer_push(this->buf);
}

ssize_t Buffer::foreach_sample(
		std::function<ssize_t(Channel &, void *, size_t)> func)
{
	this->cb = func;
	return iio_buffer_foreach_sample(this->buf,
			iio_buffer_foreach_sample_cb, this);
}

Channel::Channel(std::shared_ptr<Device> &dev, struct iio_channel *chn) :
	dev(dev),
	chn(chn),
	id(std::string(iio_channel_get_id(chn))),
	name(std::string(iio_channel_get_name(chn) ?: "")),
	attrs_count(iio_channel_get_attrs_count(chn)),
	output(iio_channel_is_output(chn)),
	scan_element(iio_channel_is_scan_element(chn)),
	index(iio_channel_get_index(chn))
{
}

Channel::Channel(std::shared_ptr<Device> &dev, unsigned int id) :
	Channel(dev, iio_device_get_channel(dev.get()->dev, id))
{
	/* TODO: Exception if id is invalid */
}

Channel::Channel(std::shared_ptr<Device> &dev, std::string &name, bool output) :
	Channel(dev, iio_device_find_channel(dev.get()->dev,
				name.c_str(), output))
{
	/* TODO: Exception if name is invalid */
}

bool Channel::enabled()
{
	return iio_channel_is_enabled(this->chn);
}

bool Channel::enable()
{
	iio_channel_enable(this->chn);
	return this->enabled();
}

bool Channel::disable()
{
	iio_channel_disable(this->chn);
	return this->enabled();
}

size_t Channel::read(Buffer &buffer, void *dst, size_t len, bool raw)
{
	if (raw)
		return iio_channel_read_raw(this->chn, buffer.buf, dst, len);
	else
		return iio_channel_read(this->chn, buffer.buf, dst, len);
}

size_t Channel::write(Buffer &buffer, const void *src, size_t len, bool raw)
{
	if (raw)
		return iio_channel_write_raw(this->chn, buffer.buf, src, len);
	else
		return iio_channel_write(this->chn, buffer.buf, src, len);
}

std::string Attr::get_name(struct iio_device *dev, unsigned int id, bool debug)
{
	if (debug)
		return std::string(iio_device_get_debug_attr(dev, id));
	else
		return std::string(iio_device_get_attr(dev, id));
}

std::string Attr::get_name(struct iio_channel *chn, unsigned int id)
{
	return std::string(iio_channel_get_attr(chn, id));
}

std::string Attr::get_filename(struct iio_channel *chn, const std::string &name)
{
	return std::string(iio_channel_attr_get_filename(chn, name.c_str()));
}

Attr::Attr(std::shared_ptr<Channel> &chn, unsigned int id) :
	dev(nullptr),
	chn(chn),
	debug(false),
	name(get_name(chn.get()->chn, id)),
	filename(name)
{
	/* TODO: exception if attr not found */
}

Attr::Attr(std::shared_ptr<Channel> &chn, std::string &name) :
	dev(nullptr),
	chn(chn),
	debug(false),
	name(name),
	filename(get_filename(chn.get()->chn, name))
{
	/* TODO: exception if attr not found */
}

Attr::Attr(std::shared_ptr<Device> &dev, unsigned int id, bool debug) :
	dev(dev),
	chn(nullptr),
	debug(debug),
	name(get_name(dev.get()->dev, id, debug)),
	filename(name)
{
	/* TODO: exception if attr not found */
}

Attr::Attr(std::shared_ptr<Device> &dev, std::string &name, bool debug) :
	dev(dev),
	chn(nullptr),
	debug(debug),
	name(name),
	filename(name)
{
	/* TODO: exception if attr not found */
}

ssize_t Attr::read(char *dst, size_t len)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_read(chn->chn, name, dst, len);
	else if (this->debug)
		return iio_device_debug_attr_read(dev->dev, name, dst, len);
	else
		return iio_device_attr_read(dev->dev, name, dst, len);
}

int Attr::read(double *value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_read_double(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_read_double(dev->dev, name, value);
	else
		return iio_device_attr_read_double(dev->dev, name, value);
}

int Attr::read(bool *value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_read_bool(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_read_bool(dev->dev, name, value);
	else
		return iio_device_attr_read_bool(dev->dev, name, value);
}

int Attr::read(long long *value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_read_longlong(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_read_longlong(
				dev->dev, name, value);
	else
		return iio_device_attr_read_longlong(dev->dev, name, value);
}

ssize_t Attr::write(const std::string &src)
{
	const char *name = this->name.c_str();
	const char *data = src.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_write(chn->chn, name, data);
	else if (this->debug)
		return iio_device_debug_attr_write(dev->dev, name, data);
	else
		return iio_device_attr_write(dev->dev, name, data);
}

int Attr::write(double value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_write_double(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_write_double(
				dev->dev, name, value);
	else
		return iio_device_attr_write_double(dev->dev, name, value);
}

int Attr::write(bool value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_write_bool(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_write_bool(dev->dev, name, value);
	else
		return iio_device_attr_write_bool(dev->dev, name, value);
}

int Attr::write(long long value)
{
	const char *name = this->name.c_str();
	auto chn = this->chn.get();
	auto dev = this->dev.get();

	if (chn)
		return iio_channel_attr_write_longlong(chn->chn, name, value);
	else if (this->debug)
		return iio_device_debug_attr_write_longlong(
				dev->dev, name, value);
	else
		return iio_device_attr_write_longlong(dev->dev, name, value);
}
