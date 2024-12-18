/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for Imagimob_MTBML_Deploy Example.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdlib.h>
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cy_log.h"

#include "app_task.h"

#include <sys/times.h>

#include "memfault/components.h"
LOG_MODULE_REGISTER(cdr);
#include "memfault/components.h"

#include "memfault/metrics/platform/overrides.h"

/******************************************************************************
 * Global Variables
 ******************************************************************************/

// static RAM storage where logs will be stored.
static uint8_t s_log_buf_storage[512];

bool s_cdr_ready_to_send = false;
// keep track of the offset into the CDR data; the packetizer may call
// repeatedly depending on chunk size constraints
size_t s_cdr_read_offset = 0;

// set of MIME types for this payload
static const char *mimetypes[] = {MEMFAULT_CDR_BINARY};

// the actual payload is just a simple string in this example
static const char cdr_payload[] = "audio (PDM) data recorded";

// the CDR metadata structure
static sMemfaultCdrMetadata s_cdr_metadata = {
  .start_time = {
    .type = kMemfaultCurrentTimeType_Unknown,
  },
  .mimetypes = mimetypes,
  .num_mimetypes = MEMFAULT_ARRAY_SIZE(mimetypes),

  // in this case, the data size is fixed. typically it would be set in the
  // prv_has_cdr_cb() function, and likely variable size
  .data_size_bytes = sizeof(cdr_payload) - 1,
  .duration_ms = 0,

  .collection_reason = "nearly-confident PDM sample for analysis",
};

// called to see if there's any data available; uses the *metadata output to set
// the header fields in the chunked message sent to Memfault
static bool prv_has_cdr_cb(sMemfaultCdrMetadata *metadata) {
  *metadata = s_cdr_metadata;
  return s_cdr_ready_to_send;
}

// called by the packetizer to read up to .data_size_bytes of CDR data
static bool prv_read_data_cb(uint32_t offset, void *data, size_t data_len) {
  if (offset != s_cdr_read_offset) {
    MEMFAULT_LOG_ERROR("Unexpected offset: %d vs %d", offset, s_cdr_read_offset);
    s_cdr_read_offset = 0;
    return false;
  }

  const size_t copy_len = MEMFAULT_MIN(data_len, sizeof(cdr_payload) - offset);
  MEMFAULT_LOG_INFO("Reading %d bytes from offset %d", copy_len, offset);

  memcpy(data, ((uint8_t *)cdr_payload) + offset, copy_len);
  s_cdr_read_offset += copy_len;
  return true;
}

// called when all data has been drained from the read callback
static void prv_mark_cdr_read_cb(void) {
  s_cdr_ready_to_send = false;
  // only reset this offset when the data has been read
  s_cdr_read_offset = 0;
}

// Set up the callback functions. This CDR Source Implementation structure must
// have a lifetime through the duration of the program- typically setting it to
// 'const' is appropriate
const sMemfaultCdrSourceImpl g_custom_data_recording_source = {
  .has_cdr_cb = prv_has_cdr_cb,
  .read_data_cb = prv_read_data_cb,
  .mark_cdr_read_cb = prv_mark_cdr_read_cb,
};

volatile int uxTopUsedPriority;

// A VERY crude implementation of log output so we can at least see some messages
int app_log_output_callback(CY_LOG_FACILITY_T facility, CY_LOG_LEVEL_T level, char *logmsg) {
  (void)facility;     // Can be used to decide to reduce output or send output to remote logging
  (void)level;        // Can be used to decide to reduce output, although the output has already been
                      // limited by the log routines

  return printf( "%s\n", logmsg);   // print directly to console
}

// A VERY crude implementation for obtaining timestamp (always 0) for logs so we can at least see some messages
cy_rslt_t app_log_time(uint32_t* time) {
    if (time != NULL) {
        *time = 0;
    }
    return CY_RSLT_SUCCESS;
}

unsigned long ulGetRunTimeCounterValue(void) {
  struct tms xTimes;
  times(&xTimes);

  return (unsigned long)xTimes.tms_utime;
}

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function initializes retarget IO, sets up
 *  the MQTT client task, and then starts the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main() {
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE) || defined (IOTC_OTA_SUPPORT)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package. */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* To avoid compiler warnings. */
    (void) result;

    /* Enable global interrupts. */
    __enable_irq();

    /* default for all logging to WARNING */
    cy_log_init(CY_LOG_WARNING, app_log_output_callback, app_log_time);

    // Uncomment this line to get more info from HTTP and similar
    // if encountering issues
    //cy_log_set_facility_level(CYLF_MIDDLEWARE, CY_LOG_INFO);

    memfault_platform_boot();
    memfault_log_boot(s_log_buf_storage, sizeof(s_log_buf_storage));

    memfault_cdr_register_source(&g_custom_data_recording_source);

    //! Load root certificates necessary for talking to Memfault servers
    if (CY_RSLT_SUCCESS != cy_tls_load_global_root_ca_certificates(MEMFAULT_ROOT_CERTS_PEM, sizeof(MEMFAULT_ROOT_CERTS_PEM) - 1)) {
    	MEMFAULT_LOG_ERROR("cy_tls_load_global_root_ca_certificates failed!");
    } else {
    	MEMFAULT_LOG_INFO("Successfully loaded global root CA certs");
    }

    /* Create the Client task. */
    xTaskCreate(app_task, "App Task", APP_TASK_STACK_SIZE, NULL, APP_TASK_PRIORITY, NULL);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}

sMfltHttpClientConfig g_mflt_http_client_config = {
  .api_key = "xxxxxxx",
};

void memfault_metrics_heartbeat_collect_data(void) {
  // NOTE: When using FreeRTOS we can just call
  //UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
  //MEMFAULT_METRIC_SET_UNSIGNED(stackHighWaterMark, stack_high_water_mark);
  MEMFAULT_METRIC_SET_STRING(motd, "have a nice day");
}

void memfault_platform_get_device_info(sMemfaultDeviceInfo *info) {
  // !FIXME: Populate with platform device information

  // IMPORTANT: All strings returned in info must be constant
  // or static as they will be used _after_ the function returns

  // See https://mflt.io/version-nomenclature for more context
  *info = (sMemfaultDeviceInfo) {
    // An ID that uniquely identifies the device in your fleet
    // (i.e serial number, mac addr, chip id, etc)
    // Regular expression defining valid device serials: ^[-a-zA-Z0-9_]+$
    .device_serial = "AIDEVBOARD1",
     // A name to represent the firmware running on the MCU.
    // (i.e "ble-fw", "main-fw", or a codename for your project)
    .software_type = "app-fw",
    // The version of the "software_type" currently running.
    // "software_type" + "software_version" must uniquely represent
    // a single binary
    .software_version = "0.3.0-dev",
    // The revision of hardware for the device. This value must remain
    // the same for a unique device.
    // (i.e evt, dvt, pvt, or rev1, rev2, etc)
    // Regular expression defining valid Hardware Versions: ^[-a-zA-Z0-9_\.\+]+$
    .hardware_version = "cy8ckit-062s2-ai",
  };
}

/* [] END OF FILE */
