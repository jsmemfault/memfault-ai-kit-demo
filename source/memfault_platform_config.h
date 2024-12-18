//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See License.txt for details
//!
//! Platform overrides for the default configuration settings in the memfault-firmware-sdk.
//! Default configuration settings can be found in "memfault/config.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define MEMFAULT_USE_GNU_BUILD_ID 1
#define MEMFAULT_EVENT_STORAGE_RAM_SIZE 4096
#define MEMFAULT_EVENT_LOG_RAM_SIZE 4096
#define MEMFAULT_METRICS_HEARTBEAT_INTERVAL_SECS 120
#define MEMFAULT_COREDUMP_COMPUTE_THREAD_STACK_USAGE 1
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_USE_RAM 1
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_RAM_SIZE 131072
#define MEMFAULT_CDR_ENABLE 1

#ifdef __cplusplus
}
#endif
