#include "memfault/components.h"

int test_logging(int argc, char *argv[]) {
  MEMFAULT_LOG_DEBUG("Debug log!");
  MEMFAULT_LOG_INFO("Info log!");
  MEMFAULT_LOG_WARN("Warning log!");
  MEMFAULT_LOG_ERROR("Error log!");
  return 0;
}

// Runs a sanity test to confirm coredump port is working as expected
int test_coredump_storage(int argc, char *argv[]) {

  // Note: Coredump saving runs from an ISR prior to reboot so should
  // be safe to call with interrupts disabled.
  your_platform_disable_interrupts();
  memfault_coredump_storage_debug_test_begin();
  your_platform_enable_interrupts();

  memfault_coredump_storage_debug_test_finish();
  return 0;
}

// Triggers an immediate heartbeat capture (instead of waiting for timer
// to expire)
int test_heartbeat(int argc, char *argv[]) {
  memfault_metrics_heartbeat_debug_trigger();
  return 0;
}

int test_trace(int argc, char *argv[]) {
  MEMFAULT_TRACE_EVENT_WITH_LOG(critical_error, "An artificially-intelligent test error!");
  return 0;
}

//! Trigger a user initiated reboot and confirm reason is persisted
int test_reboot(int argc, char *argv[]) {
  memfault_reboot_tracking_mark_reset_imminent(kMfltRebootReason_UserReset, NULL);
  memfault_platform_reboot();
}

int test_assert(int argc, char *argv[]) {
  MEMFAULT_ASSERT(0);
  return -1; // should never get here
}

int test_fault(void) {
  void (*bad_func)(void) = (void *)0xEEEEDEAD;
  bad_func();
  return -1; // should never get here
}

int test_hang(int argc, char *argv[]) {
  while (1) {}
  return -1; // should never get here
}

// Dump Memfault data collected to console
int test_export(int argc, char *argv[]) {
  memfault_data_export_dump_chunks();
  return 0;
}
