//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See License.txt for details
//!
//! Glue layer between the Memfault SDK and the underlying platform
//!

#include <FreeRTOSConfig.h>
#include <core_cm4.h>
#include <cy_device_headers.h>
#include <cy_retarget_io.h>
#include <cy_syslib.h>
#include <stdbool.h>

#include "memfault/components.h"
#include "memfault/core/compiler.h"
#include "memfault/ports/freertos.h"
#include "memfault/ports/reboot_reason.h"
#include "memfault_platform_log_config.h"

#define MEMFAULT_PRINT_RESET_INFO(...) MEMFAULT_LOG_INFO(__VA_ARGS__)

typedef struct {
  uint32_t start_addr;
  size_t length;
} sMemRegions;

sMemRegions s_mcu_mem_regions[] = {
  { .start_addr = 0x08030000, .length = 0xB7000 },
};

//! Last function called after a coredump is saved. Should perform
//! any final cleanup and then reset the device
void memfault_platform_reboot(void) {
  NVIC_SystemReset();
  while (1) { }  // unreachable
}

//! If device does not track time, return false, else return true if time is valid
bool memfault_platform_time_get_current(sMemfaultCurrentTime *time) {
  *time = (sMemfaultCurrentTime){
    .type = kMemfaultCurrentTimeType_UnixEpochTimeSec,
    .info = { .unix_timestamp_secs = 0 },
  };
  return false;
}

#include <stdarg.h>
#include <stdio.h>

void memfault_platform_log_raw(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
}

void memfault_platform_log(eMemfaultPlatformLogLevel level, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  char log_buf[128];
  vsnprintf(log_buf, sizeof(log_buf), fmt, args);

  const char *lvl_str;
  switch (level) {
    case kMemfaultPlatformLogLevel_Debug:
      lvl_str = "D";
      break;

    case kMemfaultPlatformLogLevel_Info:
      lvl_str = "I";
      break;

    case kMemfaultPlatformLogLevel_Warning:
      lvl_str = "W";
      break;

    case kMemfaultPlatformLogLevel_Error:
      lvl_str = "E";
      break;

    default:
      lvl_str = "D";
      break;
  }

  vsnprintf(log_buf, sizeof(log_buf), fmt, args);

  printf("[%s] MFLT: %s\n", lvl_str, log_buf);
}

/*
uint64_t memfault_platform_get_time_since_boot_ms(void) {
  const uint64_t tick_count = OS_Tick_GetCount();
  const uint32_t ticks_per_sec = OS_Tick_GetClock();
  return (1000 * tick_count) / ticks_per_sec;
}
*/

MEMFAULT_PUT_IN_SECTION(".noinit.mflt_reboot_info")
static uint8_t s_reboot_tracking[MEMFAULT_REBOOT_TRACKING_REGION_SIZE];
