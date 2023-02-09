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

LOG_MODULE_REGISTER(log_unitron);

struct k_msgq k_unitron_logs;
char __aligned(4) k_unitron_logs_buffer[10 * sizeof(struct unitron_queue_item)];

static uint32_t log_format_current = CONFIG_LOG_BACKEND_UNITRON_OUTPUT_DEFAULT;

static volatile bool in_panic;

struct unitron_queue_item item = {0};

static int char_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);
	for (size_t i = 0; i < length; i++) {
		// printf("%c", (unsigned char)data[i]);
		if(item.messagesize<149){
			item.logmessage[item.messagesize] = (unsigned char)data[i];
			item.messagesize++;
		}
	}
	return length;
}

//OUTPUT

static void buffer_write_unitron(log_output_func_t outf, uint8_t *buf, size_t len, void *ctx)
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

	// push_to_unitron_logs(output->buf + output->control_block->offset);

	output->control_block->offset = 0;
}

static int out_func_unitron(int c, void *ctx)
{
	const struct log_output *out_ctx = (const struct log_output *)ctx;
	int idx;

	if (out_ctx->control_block->offset == out_ctx->size) {
		log_output_flush_unitron(out_ctx);
	}

	idx = atomic_inc(&out_ctx->control_block->offset);
	out_ctx->buf[idx] = (uint8_t)c;

	__ASSERT_NO_MSG(out_ctx->control_block->offset <= out_ctx->size);

	return 0;
}

void log_output_process_unitron(const struct log_output *output, const uint8_t *package)
{	
	if (package) {
		int err = cbpprintf(out_func_unitron, (void *)output, (void *)package);

		(void)err;
		__ASSERT_NO_MSG(err >= 0);
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

	memset(&item, 0, sizeof(item));
	
	item.log_level = level;
	item.timestamp = timestamp;	

	item.module_name = (unsigned char *) malloc(strlen(sname) + 1);  // allocate memory for the string
	strcpy(item.module_name, sname);

	log_output_process_unitron(output, plen > 0 ? package : NULL);

	// item.logmessage = (unsigned char *) malloc(output->size + 1);  // allocate memory for the string
	// strcpy(item.logmessage, (unsigned char *)output->buf);

	uint8_t * buffer = output->buf;
	size_t size = output->control_block->offset;

	// printf("size %d msg: ",size);

	// while (size != 0){
	// 	printf("%c", (unsigned char)*buffer);
	// 	size--;
	// 	buffer++;
	// }
	
	// printf("\n");

	// output->control_block->offset=0;

	int ret = k_msgq_put(&k_unitron_logs, &item, K_NO_WAIT);
	if(ret){
		// LOG_WRN("impossible to add to queue %d", ret);
	}
}

static uint8_t unitron_output_buf[CONFIG_LOG_BACKEND_UNITRON_BUFFER_SIZE];
LOG_OUTPUT_DEFINE(log_output_unitron, char_out, unitron_output_buf, sizeof(unitron_output_buf));

static void process(const struct log_backend *const backend,
		union log_msg_generic *msg)
{

	uint8_t level = log_msg_get_level(&(msg->log));

	if(level == LOG_LEVEL_ERR){

		uint32_t flags = 0;//log_backend_std_get_flags();

		log_custom_output_msg_set(log_output_msg_process_unitron);

		log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

		log_output_func(&log_output_unitron, &(msg->log), flags);


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