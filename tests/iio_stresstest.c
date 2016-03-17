/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014 Analog Devices, Inc.
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

#define _BSD_SOURCE

#include <getopt.h>
#include <iio.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>

#define MY_NAME "iio_stresstest"

#define SAMPLES_PER_READ 256

static const struct option options[] = {
	  {"help", no_argument, 0, 'h'},
	  {"network", required_argument, 0, 'n'},
	  {"usb", required_argument, 0, 'u'},
	  {"buffer-size", required_argument, 0, 'b'},
	  {"samples", required_argument, 0, 's' },
	  {0, 0, 0, 0},
};

static const char *options_descriptions[] = {
	"Show this help and quit.",
	"Use the network backend with the provided hostname.",
	"Use the USB backend with the device that matches the given VID/PID.",
	"Size of the capture buffer. Default is 256.",
	"Number of samples to capture, 0 = infinite. Default is 0."
};

static void usage(void)
{
	unsigned int i;

	printf("Usage:\n\t" MY_NAME " [-n <hostname>] [-u <vid>:<pid>] "
			"[-t <trigger>] [-b <buffer-size>] [-s <samples>] "
			"<iio_device> [<channel> ...]\n\nOptions:\n");
	for (i = 0; options[i].name; i++)
		printf("\t-%c, --%s\n\t\t\t%s\n",
					options[i].val, options[i].name,
					options_descriptions[i]);
}

static bool app_running = true;
static bool threads_running = true;
static int exit_code = EXIT_SUCCESS;

static void quit_all(int sig)
{
	exit_code = sig;
	app_running = false;
}

static void set_handler(int signal_nb, void (*handler)(int))
{
#ifdef _WIN32
	signal(signal_nb, handler);
#else
	struct sigaction sig;
	sigaction(signal_nb, NULL, &sig);
	sig.sa_handler = handler;
	sigaction(signal_nb, &sig, NULL);
#endif
}

static struct iio_device * get_device(const struct iio_context *ctx,
		const char *id)
{
	unsigned int i, nb_devices = iio_context_get_devices_count(ctx);
	struct iio_device *device;

	for (i = 0; i < nb_devices; i++) {
		const char *name;
		device = iio_context_get_device(ctx, i);
		name = iio_device_get_name(device);
		if (name && !strcmp(name, id))
			break;
		if (!strcmp(id, iio_device_get_id(device)))
			break;
	}

	if (i < nb_devices)
		return device;

	fprintf(stderr, "Device %s not found\n", id);
	fflush(stderr);
	return NULL;
}

struct info {
	int argc;
	char **argv;

	int ip_index, device_index, arg_index;
	unsigned int buffer_size;
};

static void *client_thread(void *data)
{
	struct info *info = data;
	struct iio_context *ctx;
	struct iio_buffer *buffer;
	unsigned int i, nb_channels;
	struct iio_device *dev;

	if (info->ip_index) {
		ctx = iio_create_network_context(info->argv[info->ip_index]);
	} else if (info->device_index) {
		char *ptr = info->argv[info->device_index], *end;
		long vid, pid;

		vid = strtol(ptr, &end, 0);
		if (ptr == end || *end != ':') {
			fprintf(stderr, "Invalid VID/PID\n");
			fflush(stderr);
			return (void *)EXIT_FAILURE;
		}

		ptr = end + 1;
		pid = strtol(ptr, &end, 0);
		if (ptr == end) {
			fprintf(stderr, "Invalid VID/PID\n");
			return (void *)EXIT_FAILURE;
		}

		ctx = iio_create_usb_context(
				(unsigned short) vid, (unsigned short) pid);
	} else {
		ctx = iio_create_default_context();
	}

	if (!ctx) {
		fprintf(stderr, "Unable to create IIO context\n");
		return (void *)EXIT_FAILURE;
	}

	iio_context_set_timeout(ctx, UINT_MAX);

	dev = get_device(ctx, info->argv[info->arg_index + 1]);
	if (!dev) {
		iio_context_destroy(ctx);
		return (void *)EXIT_FAILURE;
	}

	nb_channels = iio_device_get_channels_count(dev);

	if (info->argc == info->arg_index + 2) {
		/* Enable all channels */
		for (i = 0; i < nb_channels; i++)
			iio_channel_enable(iio_device_get_channel(dev, i));
	} else {
		for (i = 0; i < nb_channels; i++) {
			unsigned int j;
			struct iio_channel *ch = iio_device_get_channel(dev, i);
			for (j = info->arg_index + 2; j < (unsigned int) info->argc; j++) {
				const char *n = iio_channel_get_name(ch);
				if (!strcmp(info->argv[j], iio_channel_get_id(ch)) ||
						(n && !strcmp(n, info->argv[j])))
					iio_channel_enable(ch);
			}
		}
	}

	printf("Running\n");
	fflush(stdout);

	while (threads_running) {
		buffer = iio_device_create_buffer(dev, info->buffer_size, false);
		if (!buffer) {
			fprintf(stderr, "Unable to allocate buffer: %s\n", strerror(errno));
			fflush(stderr);
			usleep(1);
			continue;
		}

		while (threads_running) {
			int ret = iio_buffer_refill(buffer);
			if (ret < 0) {
				fprintf(stderr, "Unable to refill buffer: %s\n",
						strerror(-ret));
				fflush(stderr);
				break;
			}

			if (rand() % 10 == 0)
				break;
		}
		iio_buffer_destroy(buffer);
	}

	printf("Stopping\n");
	fflush(stdout);

	iio_context_destroy(ctx);

	printf("Stopped\n");
	fflush(stdout);

	return (void *)0;
}

int main(int argc, char **argv)
{
	unsigned int num_threads = 20;
	sigset_t set, oldset;
	pthread_t *threads;
	struct info info;
	int option_index;
	unsigned int i;
	int c;

#ifndef _WIN32
	set_handler(SIGHUP, &quit_all);
	set_handler(SIGPIPE, &quit_all);
#endif
	set_handler(SIGINT, &quit_all);
	set_handler(SIGSEGV, &quit_all);
	set_handler(SIGTERM, &quit_all);

	info.buffer_size = SAMPLES_PER_READ;
	info.arg_index = 0;
	info.argc = argc;
	info.argv = argv;

	while ((c = getopt_long(argc, argv, "+hn:u:b:s:",
					options, &option_index)) != -1) {
		switch (c) {
		case 'h':
			usage();
			return EXIT_SUCCESS;
		case 'n':
			info.arg_index += 2;
			info.ip_index = info.arg_index;
			break;
		case 'u':
			info.arg_index += 2;
			info.device_index = info.arg_index;
			break;
		case 'b':
			info.arg_index += 2;
			info.buffer_size = atoi(info.argv[info.arg_index]);
			break;
		case '?':
			return EXIT_FAILURE;
		}
	}

	if (info.arg_index + 1 >= argc) {
		fprintf(stderr, "Incorrect number of arguments.\n\n");
		usage();
		return EXIT_FAILURE;
	}

	threads = calloc(num_threads, sizeof(*threads));
	sigfillset(&set);

	while (app_running) {
		threads_running = true;
		pthread_sigmask(SIG_BLOCK, &set, &oldset);
		for (i = 0; i < num_threads; i++) {
			pthread_create(&threads[i], NULL, client_thread, &info);;
		}
		pthread_sigmask(SIG_SETMASK, &oldset, NULL);

		while (app_running) {
			if (rand() % 1000 == 0)
				break;
			usleep(10000);
		}

		threads_running = false;

		for (i = 0; i < num_threads; i++)
			pthread_join(threads[i], NULL);
	}

	return 0;
}
