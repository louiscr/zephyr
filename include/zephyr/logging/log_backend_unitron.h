/*
 * Copyright (c) 2021 unitron, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UNITRON_INCLUDE_LOGGING_UNITRON_H_
#define UNITRON_INCLUDE_LOGGING_UNITRON_H_

#include <zephyr/logging/log_msg.h>
#include <stdarg.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/kernel.h>


struct unitron_queue_item {
    uint32_t log_level;
    uint64_t timestamp;
    unsigned char* module_name;
    unsigned char* logmessage;
};

extern struct k_msgq k_unitron_logs;

/**
 * @defgroup logging unitron Logging
 * Functions for interfacing with the unitron Zephyr logging backend.
 * @{
 */




/** @} */

#endif /* unitron_INCLUDE_LOGGING_unitron_H_ */
