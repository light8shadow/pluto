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

#include <functional>
#include <memory>
#include <string>
#include <vector>

struct iio_buffer;
struct iio_channel;
struct iio_context;
struct iio_device;

namespace iio {
	class Channel;

	class Context {
		friend class Device;

	private:
		struct iio_context *ctx;

		Context(struct iio_context *ctx);

	public:
		const std::string name;
		const std::string description;
		const std::string xml;
		const unsigned int devices_count;

		Context();
		Context(std::string &hostname);
		Context(iio::Context &ctx);

		~Context();

		int set_timeout(unsigned int timeout_ms);
	};

	class Device {
		friend class Attr;
		friend class Buffer;
		friend class Channel;

	private:
		std::shared_ptr<Context> ctx;
		struct iio_device *dev;

		Device(std::shared_ptr<Context> &ctx, struct iio_device *dev);

	public:
		const std::string id;
		const std::string name;
		const unsigned int channels_count;
		const unsigned int attrs_count;
		const unsigned int debug_attrs_count;

		Device(std::shared_ptr<Context> &ctx, unsigned int id);
		Device(std::shared_ptr<Context> &ctx, std::string &id);

		ssize_t get_sample_size();

		int reg_read(uint32_t address, uint32_t *value);
		int reg_write(uint32_t address, uint32_t value);
	};

	class Buffer {
		friend class Channel;

	private:
		struct iio_buffer *buf;
		std::shared_ptr<Device> dev;

		/* Temporary placeholder */
		std::function<ssize_t(Channel &, void *, size_t)> cb;

		static ssize_t iio_buffer_foreach_sample_cb(
				const struct iio_channel *chn,
				void *src, size_t bytes, void *d);

	public:
		const size_t samples_count;
		const bool cyclic;

		Buffer(std::shared_ptr<Device> &dev,
				size_t samples_count, bool cyclic = false);

		~Buffer();

		ssize_t refill();
		ssize_t push();

		ssize_t foreach_sample(std::function<ssize_t(Channel &,
					void *, size_t)> func);
	};

	class Channel {
		friend class Attr;

	private:
		std::shared_ptr<Device> dev;
		struct iio_channel *chn;

		Channel(std::shared_ptr<Device> &dev, struct iio_channel *chn);

	public:
		const std::string id;
		const std::string name;
		const unsigned int attrs_count;
		const bool output;
		const bool scan_element;
		const long index;

		Channel(std::shared_ptr<Device> &dev, unsigned int id);
		Channel(std::shared_ptr<Device> &dev,
				std::string &name, bool output = false);

		bool enabled();
		bool enable();
		bool disable();

		size_t read(iio::Buffer &buffer, void *dst,
				size_t len, bool raw = false);

		size_t write(iio::Buffer &buffer, const void *src,
				size_t len, bool raw = false);
	};

	class Attr {
	private:
		std::shared_ptr<Device> dev;
		std::shared_ptr<Channel> chn;

		static std::string get_name(struct iio_device *dev,
				unsigned int id, bool debug);
		static std::string get_name(
				struct iio_channel *chn, unsigned int id);
		static std::string get_filename(struct iio_channel *chn,
				const std::string &name);

	public:
		const std::string name;
		const std::string filename;
		const bool debug;

		Attr(std::shared_ptr<Channel> &chn, unsigned int id);
		Attr(std::shared_ptr<Channel> &chn, std::string &name);
		Attr(std::shared_ptr<Device> &dev,
				unsigned int id, bool debug = false);
		Attr(std::shared_ptr<Device> &dev,
				std::string &name, bool debug = false);

		ssize_t read(char *dst, size_t len);
		int read(double *value);
		int read(bool *value);
		int read(long long *value);

		ssize_t write(const std::string &src);
		int write(double value);
		int write(bool value);
		int write(long long value);
	};
}
