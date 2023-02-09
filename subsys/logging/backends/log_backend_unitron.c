/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_backend_unitron.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_output_custom.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_output_dict.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/sys/cbprintf.h>
#include <ctype.h>


#define HEXDUMP_BYTES_IN_LINE 16

LOG_MODULE_REGISTER(log_unitron);

struct k_msgq k_unitron_logs;
char __aligned(4) k_unitron_logs_buffer[10 * sizeof(struct unitron_queue_item)];

static uint32_t log_format_current = CONFIG_LOG_BACKEND_UNITRON_OUTPUT_DEFAULT;

static volatile bool in_panic;

static int char_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);
	// printf("%d\n", length);
	for (size_t i = 0; i < length; i++) {
		printf("%c", (unsigned char)data[i]);
	}	
	return length;

			// struct unitron_queue_item item = {0};
		// int ret = k_msgq_put(&k_unitron_logs, &item, K_NO_WAIT);
// if(ret == 0){
// 		return length
// }


}


//OUTPUT

static void buffer_write_unitron(log_output_func_t outf, uint8_t *buf, size_t len,
			 void *ctx)
{
	int processed;

	do {
		processed = outf(buf, len, ctx);
		len -= processed;
		buf += processed;
	} while (len != 0);
}


void log_output_flush_unitron(const struct log_output *output)
{
	buffer_write_unitron(output->func, output->buf,
		     output->control_block->offset,
		     output->control_block->ctx);

	output->control_block->offset = 0;
}

static int out_func_unitron(int c, void *ctx)
{
	const struct log_output *out_ctx = (const struct log_output *)ctx;
	int idx;

	if (IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE)) {
		/* Backend must be thread safe in synchronous operation. */
		/* Need that step for big endian */
		char x = (char)c;

		out_ctx->func((uint8_t *)&x, 1, out_ctx->control_block->ctx);
		return 0;
	}

	if (out_ctx->control_block->offset == out_ctx->size) {
		log_output_flush_unitron(out_ctx);
	}

	idx = atomic_inc(&out_ctx->control_block->offset);
	out_ctx->buf[idx] = (uint8_t)c;

	__ASSERT_NO_MSG(out_ctx->control_block->offset <= out_ctx->size);

	return 0;
}

static int cr_out_func_unitron(int c, void *ctx)
{
	if (c == '\n') {
		out_func_unitron((int)'\r', ctx);
	}
	out_func_unitron(c, ctx);

	return 0;
}

static int print_formatted_unitron(const struct log_output *output,
			   const char *fmt, ...)
{
	va_list args;
	int length = 0;

	va_start(args, fmt);
	length = cbvprintf(out_func_unitron, (void *)output, fmt, args);
	va_end(args);

	return length;
}

static void newline_print_unitron(const struct log_output *ctx, uint32_t flags)
{
	if (IS_ENABLED(CONFIG_LOG_BACKEND_NET) &&
	    flags & LOG_OUTPUT_FLAG_FORMAT_SYSLOG) {
		return;
	}

	if ((flags & LOG_OUTPUT_FLAG_CRLF_NONE) != 0U) {
		return;
	}

	if ((flags & LOG_OUTPUT_FLAG_CRLF_LFONLY) != 0U) {
		print_formatted_unitron(ctx, "\n");
	} else {
		print_formatted_unitron(ctx, "\r\n");
	}
}

static void hexdump_line_print_unitron(const struct log_output *output,
			       const uint8_t *data, uint32_t length,
			       int prefix_offset, uint32_t flags)
{
	newline_print_unitron(output, flags);

	for (int i = 0; i < prefix_offset; i++) {
		print_formatted_unitron(output, " ");
	}

	for (int i = 0; i < HEXDUMP_BYTES_IN_LINE; i++) {
		if (i > 0 && !(i % 8)) {
			print_formatted_unitron(output, " ");
		}

		if (i < length) {
			print_formatted_unitron(output, "%02x ", data[i]);
		} else {
			print_formatted_unitron(output, "   ");
		}
	}

	print_formatted_unitron(output, "|");

	for (int i = 0; i < HEXDUMP_BYTES_IN_LINE; i++) {
		if (i > 0 && !(i % 8)) {
			print_formatted_unitron(output, " ");
		}

		if (i < length) {
			unsigned char c = (unsigned char)data[i];

			print_formatted_unitron(output, "%c",
			      isprint((int)c) ? c : '.');
		} else {
			print_formatted_unitron(output, " ");
		}
	}
}

static void log_msg_hexdump_unitron(const struct log_output *output,
			    uint8_t *data, uint32_t len,
			    int prefix_offset, uint32_t flags)
{
	size_t length;

	do {
		length = MIN(len, HEXDUMP_BYTES_IN_LINE);

		hexdump_line_print_unitron(output, data, length,
				   prefix_offset, flags);
		data += length;
		len -= length;
	} while (len);
}

void log_output_process_unitron(const struct log_output *output,
			log_timestamp_t timestamp,
			const char *domain,
			const char *source,
			uint8_t level,
			const uint8_t *package,
			const uint8_t *data,
			size_t data_len,
			uint32_t flags)
{
	cbprintf_cb cb = ((uintptr_t)source == 1) ? out_func_unitron : cr_out_func_unitron;
	
	if (package) {
		int err = cbpprintf(cb, (void *)output, (void *)package);

		(void)err;
		__ASSERT_NO_MSG(err >= 0);
	}

	if (data_len) {
		log_msg_hexdump_unitron(output, (uint8_t *)data, data_len, 0, flags);
	}

	log_output_flush_unitron(output);
}

void log_output_msg_process_unitron(const struct log_output *output,
			    struct log_msg *msg, uint32_t flags)
{
	log_timestamp_t timestamp = log_msg_get_timestamp(msg);
	uint8_t level = log_msg_get_level(msg);
	uint8_t domain_id = log_msg_get_domain(msg);
	int16_t source_id;

	if (IS_ENABLED(CONFIG_LOG_MULTIDOMAIN) && domain_id != Z_LOG_LOCAL_DOMAIN_ID) {
		/* Remote domain is converting source pointer to ID */
		source_id = (int16_t)(uintptr_t)log_msg_get_source(msg);
	} else {
		void *source = (void *)log_msg_get_source(msg);

		if (source != NULL) {
			source_id = IS_ENABLED(CONFIG_LOG_RUNTIME_FILTERING) ?
					log_dynamic_source_id(source) :
					log_const_source_id(source);
		} else {
			source_id = -1;
		}
	}

	const char *sname = source_id >= 0 ? log_source_name_get(domain_id, source_id) : NULL;
	size_t plen, dlen;
	uint8_t *package = log_msg_get_package(msg, &plen);
	uint8_t *data = log_msg_get_data(msg, &dlen);

	log_output_process_unitron(output, timestamp, NULL, sname, level,
			   plen > 0 ? package : NULL, data, dlen, flags);
}

static uint8_t unitron_output_buf[CONFIG_LOG_BACKEND_UNITRON_BUFFER_SIZE];
LOG_OUTPUT_DEFINE(log_output_unitron, char_out, unitron_output_buf, sizeof(unitron_output_buf));

static void process(const struct log_backend *const backend,
		union log_msg_generic *msg)
{

	uint8_t level = log_msg_get_level(&(msg->log));

	if(level == LOG_LEVEL_ERR){
		printk("%d\n", level);

		//msg->log.hdr.desc.level = LOG_LEVEL_INTERNAL_RAW_STRING;

		uint32_t flags = 0;//log_backend_std_get_flags();

		log_custom_output_msg_set(log_output_msg_process_unitron);

		log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

		log_output_func(&log_output_unitron, &(msg->log), flags);

		printf("\n");

		size_t dlen;
		uint8_t *data = log_msg_get_data(&msg->log, &dlen);
		for (size_t i = 0; i < dlen; i++) {
			unsigned char c = (unsigned char)data[i];
			printf("%c", c);
		}	
		
		printf("<= data\n");

		size_t plen;
		uint8_t *package = log_msg_get_package(&msg->log, &plen);
		for (size_t i = 0; i < plen; i++) {
			printf("%c", (unsigned char)package[i]);
		}
		
		printf("<= package\n");


		// struct unitron_queue_item item = {0};
		// item.log_level = level;

		// void *source = (void *)log_msg_get_source(&(msg->log));
		// uint8_t domain_id = log_msg_get_domain(&(msg->log));
		// int16_t source_id = (IS_ENABLED(CONFIG_LOG_RUNTIME_FILTERING) ? log_dynamic_source_id(source) : log_const_source_id(source));
		// const char* name = log_source_name_get(domain_id, source_id);

		// item.module_name = (unsigned char *) malloc(strlen(name) + 1);  // allocate memory for the string
		// strcpy(item.module_name, name);

		// uint64_t timestamp = log_msg_get_timestamp(&(msg->log));
		// item.timestamp = timestamp;

		// size_t len;
		// uint8_t *data = log_msg_get_package(&(msg->log), &len);
		// printf("%d\n", len);
		// for (size_t i = 0; i < len; i++) {
		// 	printf("%c", (unsigned char)data[i]);
		// }	

		// // item.module_name = (char *) malloc(strlen(name) + 1);  // allocate memory for the string
		// item.logmessage = (unsigned char *) malloc(len + 1);  // allocate memory for the string

		// snprintf(item.logmessage, len, "%s", data);

		// printf("%s\n", item.logmessage);

		// int ret = k_msgq_put(&k_unitron_logs, &item, K_NO_WAIT);
		// if(ret){
		// 	LOG_WRN("impossible to add to queue %d", ret);
		// }
	}
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

static void log_backend_unitron_init(struct log_backend const *const backend)
{
	k_msgq_init(&k_unitron_logs, k_unitron_logs_buffer, sizeof(struct unitron_queue_item), 10);
}

static void panic(struct log_backend const *const backend)
{
	log_backend_std_panic(&log_output_unitron);
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);
	log_backend_std_dropped(&log_output_unitron, cnt);
}

const struct log_backend_api log_backend_unitron_api = {
	.process = process,
	.panic = panic,
	.init = log_backend_unitron_init,
	.dropped = dropped,
	.format_set = format_set,
};

LOG_BACKEND_DEFINE(log_backend_unitron, log_backend_unitron_api, 
		IS_ENABLED(CONFIG_LOG_BACKEND_UNITRON_AUTOSTART));


// void log_custom_output_msg_process(const struct log_output *log_output, struct log_msg *msg, uint32_t flags);

// /** @brief Set the formatting log function that will be applied with LOG_OUTPUT_CUSTOM
//  *
//  * @param format Pointer to the external formatter function
//  */
// void log_custom_output_msg_set(log_format_func_t format);