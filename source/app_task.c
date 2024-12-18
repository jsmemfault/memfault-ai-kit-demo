/*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
/* SPDX-License-Identifier: MIT
 * Copyright (C) 2024 Avnet
 * Authors: Nikola Markovic <nikola.markovic@avnet.com>, Shu Liu <shu.liu@avnet.com> et al.
 */
#include "cyhal.h"
#include "cybsp.h"

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"

/* Configuration file for Wi-Fi and MQTT client */
#include "wifi_config.h"
#include "memfault/components.h"

/* Middleware libraries */
#include "cy_retarget_io.h"
//#include "clock.h"

/* LwIP header files */
#include "lwip/netif.h"
#include "lwip/apps/sntp.h"

// AI model related:
#include "mtb_ml_utils.h"
#include "mtb_ml_common.h"
#include "imu.h"
#include "audio.h"
#include "inference.h"
#include "config.h"
#include "imu_model.h"
#include "pdm_model.h"
#include MTB_ML_INCLUDE_MODEL_FILE(MODEL_NAME)

// App related
#include "iotc_gencert.h"
#include "app_eeprom_data.h"
#include "app_config.h"
#include "app_task.h"

#define APP_VERSION "03.01.00"

#define LABEL_BABY_CRY 	1
#define LABEL_UNLABELED	0

/* Constants to define LONG and SHORT presses on User Button (x10 = ms) */
#define QUICK_PRESS_COUNT       2u      /* 20 ms < press < 200 ms */
#define SHORT_PRESS_COUNT       20u     /* 200 ms < press < 2 sec */
#define LONG_PRESS_COUNT        200u    /* press > 2 sec */

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS   10u     /* in ms */
#define LONG_GLITCH_DELAY_MS    100u    /* in ms */

/* User button press delay*/
#define USER_BTN_PRESS_DELAY    10u     /* in ms */

typedef enum
{
    SWITCH_NO_EVENT     = 0u,
    SWITCH_QUICK_PRESS  = 1u,
    SWITCH_SHORT_PRESS  = 2u,
    SWITCH_LONG_PRESS   = 3u,
    SWITCH_KEY1         = 4u,
    SWITCH_KEY2         = 5u,
} en_switch_event_t;

en_switch_event_t get_switch_event(void);

typedef enum UserInputYnStatus {
	APP_INPUT_NONE = 0,
	APP_INPUT_YES,
	APP_INPUT_NO
} UserInputYnStatus;

static UserInputYnStatus user_input_status = APP_INPUT_NONE;
static TaskHandle_t model_task_handle = NULL;

// AI Model related values --------------
// NOTE: The ML code logic in inference.c does not seem to be decoupled from what would be "application logic".
// One could say that it is actually an application, but there is some knowledge embedded in it that we should be able to refer to instead.
// We have to duplicate some things locally and hack unfortunately.

// Globals:
volatile bool pdm_pcm_flag;
volatile bool imu_flag;

/* Model information */
static mtb_ml_model_t *imagimob_model_obj;

/* Output/result buffers for the inference engine */
static MTB_ML_DATA_T *result_buffer;

/* Model Output Size */
static int model_output_size;
static bool processing = false;

#if INFERENCE_MODE_SELECT == IMU_INFERENCE
static const char* LABELS[IMAI_IMU_DATA_OUT_COUNT] = IMAI_IMU_SYMBOL_MAP;
#else
static const char* LABELS[IMAI_PDM_DATA_OUT_COUNT] = IMAI_PDM_SYMBOL_MAP;
#endif
// --------------

static float confidence_baby_cry = 0;
static float highest_confidence = 0; // when was the value recorded. Used in the logic to "hold" the value.

static int detection_threshold = 85;

extern int test_logging(int argc, char *argv[]);
extern int test_coredump_storage(int argc, char *argv[]);
extern int test_heartbeat(int argc, char *argv[]);
extern int test_trace(int argc, char *argv[]);
extern int test_reboot(int argc, char *argv[]);
extern int test_assert(int argc, char *argv[]);
extern int test_fault(void);
extern int test_hang(int argc, char *argv[]);
extern int test_export(int argc, char *argv[]);


/******************************************************************************
 * Function Name: wifi_connect
 ******************************************************************************
 * Summary:
 *  Function that initiates connection to the Wi-Fi Access Point using the
 *  specified SSID and PASSWORD. The connection is retried a maximum of
 *  'MAX_WIFI_CONN_RETRIES' times with interval of 'WIFI_CONN_RETRY_INTERVAL_MS'
 *  milliseconds.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t : CY_RSLT_SUCCESS upon a successful Wi-Fi connection, else an
 *              error code indicating the failure.
 *
 ******************************************************************************/
static cy_rslt_t wifi_connect(void) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_connect_params_t connect_param;
    cy_wcm_ip_address_t ip_address;
    const char* wifi_ssid = app_eeprom_data_get_wifi_ssid(WIFI_SSID);
    const char* wifi_pass = app_eeprom_data_get_wifi_pass(WIFI_PASSWORD);

    /* Check if Wi-Fi connection is already established. */
    if (cy_wcm_is_connected_to_ap() == 0) {
        /* Configure the connection parameters for the Wi-Fi interface. */
        memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
        memcpy(connect_param.ap_credentials.SSID, wifi_ssid, strlen(wifi_ssid));
        memcpy(connect_param.ap_credentials.password, wifi_pass, strlen(wifi_pass));
        connect_param.ap_credentials.security = WIFI_SECURITY;

        printf("Connecting to Wi-Fi AP '%s'\n", connect_param.ap_credentials.SSID);

        /* Connect to the Wi-Fi AP. */
        for (uint32_t retry_count = 0; retry_count < MAX_WIFI_CONN_RETRIES; retry_count++) {
            result = cy_wcm_connect_ap(&connect_param, &ip_address);

            if (result == CY_RSLT_SUCCESS) {
                printf("\nSuccessfully connected to Wi-Fi network '%s'.\n", connect_param.ap_credentials.SSID);

                /* Set the appropriate bit in the status_flag to denote
                 * successful Wi-Fi connection, print the assigned IP address.
                 */
                if (ip_address.version == CY_WCM_IP_VER_V4) {
                    printf("IPv4 Address Assigned: %s\n", ip4addr_ntoa((const ip4_addr_t*) &ip_address.ip.v4));
                } else if (ip_address.version == CY_WCM_IP_VER_V6) {
                    printf("IPv6 Address Assigned: %s\n", ip6addr_ntoa((const ip6_addr_t*) &ip_address.ip.v6));
                }
                return result;
            }

            printf("Connection to Wi-Fi network failed with error code 0x%0X. Retrying in %d ms. Retries left: %d\n",
                    (int) result, WIFI_CONN_RETRY_INTERVAL_MS, (int) (MAX_WIFI_CONN_RETRIES - retry_count - 1));
            vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MS));
        }

        printf("\nExceeded maximum Wi-Fi connection attempts!\n");
        printf("Wi-Fi connection failed after retrying for %d mins\n",
                (int) ((WIFI_CONN_RETRY_INTERVAL_MS * MAX_WIFI_CONN_RETRIES) / 60000u));
    }
    return result;
}

static void app_inference_feed(float *processed_data) {
#if !COMPONENT_ML_FLOAT32
            /* Quantize data before feeding model */
            MTB_ML_DATA_T data_feed_int[PDM_BATCH_SIZE][PDM_NUM_AXIS];
            mtb_ml_utils_model_quantize(imagimob_model_obj, &processed_data[0][0], &data_feed_int[0][0]);

            /* Feed the Model */
            MTB_ML_DATA_T *input_reference = (MTB_ML_DATA_T *) data_feed_int;
            mtb_ml_model_run(imagimob_model_obj, input_reference);
            mtb_ml_utils_model_dequantize(imagimob_model_obj, results);

#else
            MTB_ML_DATA_T *input_reference = (MTB_ML_DATA_T *) processed_data;
            mtb_ml_model_run(imagimob_model_obj, input_reference);
#endif
}


static void app_model_output(void) {

#if !COMPONENT_ML_FLOAT32
    /* Convert 8-bit fixed-point output to floating-point for visualization */
    float *nn_float_buffer = (float *) pvPortMalloc(model_output_size * sizeof(float));
    mtb_ml_utils_model_dequantize(imagimob_model_obj, nn_float_buffer);
#else
    float *nn_float_buffer = result_buffer;
#endif

    if (!nn_float_buffer) {
        printf("Failed to allocate the model data buffer!\n");
        return;
    }

    // the previous algorithm was flexible to detect any class.
    // Keep "LABELS" around, so there's no compiler warning, in case if we ever return to it
    (void) LABELS;

    confidence_baby_cry = 100.0f * nn_float_buffer[1];
    if (confidence_baby_cry > highest_confidence) {
    	highest_confidence = confidence_baby_cry;
    	MEMFAULT_METRIC_SESSION_SET_UNSIGNED(highestConfidence, listening, highest_confidence);
    }
    if (confidence_baby_cry > (float) detection_threshold) {
        printf("confidence: %3.1f%% (detected)\n", confidence_baby_cry);
    	MEMFAULT_LOG_INFO("cry! (confidence: %3.1f)", confidence_baby_cry);
    	MEMFAULT_METRIC_SESSION_ADD(cries, listening, 1);
    } else if (confidence_baby_cry > (float) (detection_threshold - (detection_threshold * .15))) { // "almost" cries are 15% off of a confident cry
    	s_cdr_ready_to_send = true;
    	s_cdr_read_offset = 0;
    	MEMFAULT_LOG_INFO("cry. (maybe) (confidence: %.2f)", confidence_baby_cry);
    	MEMFAULT_METRIC_SESSION_ADD(almostCries, listening, 1);
    } else {
    	MEMFAULT_METRIC_SESSION_SET_UNSIGNED(systemLoad, listening, 80u);
        printf("confidence: %3.1f%%\n", confidence_baby_cry);
    }

#if !COMPONENT_ML_FLOAT32
    vPortFree(nn_float_buffer);
#endif
}


// Feed input data into the model (read from board sensors and write to the model)
static bool app_model_process(void) {

#if INFERENCE_MODE_SELECT == IMU_INFERENCE
    /* Store the processed data from IMAI_IMU_dequeue */
    float processed_data[IMAI_IMU_DATA_OUT_COUNT];
#else
    /* Store processed data from IMAI_dequeue */
    float processed_data[IMAI_PDM_DATA_OUT_COUNT];
#endif


#if INFERENCE_MODE_SELECT == IMU_INFERENCE
        /* Check if the IMU interrupt has triggered */
        if(true == imu_flag) {
            imu_flag = false;

            /* Check if there is enough pre-processed data to start
             * an inference. When there is enough data, processed_data
             * points to the processed IMU data */
            if (IMAI_IMU_RET_SUCCESS == IMAI_IMU_dequeue(processed_data)) {
                /* Feed the neural net the pre-processed data */
                app_inference_feed(processed_data);
                app_model_output();
                return true;
			}
			return false;
        }
#endif

#if INFERENCE_MODE_SELECT == PDM_INFERENCE
        /* Check if the PDM event has triggered */
        if(true == pdm_pcm_flag) {
            /* Check if there is enough pre-processed data to start
             * an inference. When there is enough data, processed_data
             * points to the processed PDM data */
            if(PDM_PROCESSING_COMPLETE == pdm_preprocessing_feed(processed_data)) {
                /* Feed the neural net the pre-processed data */
                app_inference_feed(processed_data);
                app_model_output();
            }
            /* Reset the pdm event flag */
            pdm_pcm_flag = false;
            return true;
        }
        return false;
#endif
}

static cy_rslt_t app_inference_init(void) {
    cy_rslt_t result;

    mtb_ml_model_bin_t imagimob_model_bin = {MTB_ML_MODEL_BIN_DATA(MODEL_NAME)};

    /* Initialize the Neural Network */
    result = mtb_ml_model_init(&imagimob_model_bin, NULL, &imagimob_model_obj);
    if(CY_RSLT_SUCCESS != result) {
        return result;
    }

    mtb_ml_model_get_output(imagimob_model_obj, &result_buffer, &model_output_size);

    return result;
}

static cy_rslt_t app_model_init(void) {
    cy_rslt_t result;

    /* Initialize the inference engine */
    result = app_inference_init();

    if(CY_RSLT_SUCCESS != result) {
        printf("Failed to inference_init()!\n");
        return result;
    }

#if INFERENCE_MODE_SELECT == IMU_INFERENCE
    /* Initialize Imagimob pre-processing library */
    IMAI_IMU_init();
    /* Start the imu, timer, and pre-process the data */
    result = imu_init();
#endif

#if INFERENCE_MODE_SELECT == PDM_INFERENCE
    /* Initialize Imagimob pre-processing library */
    IMAI_PDM_init();
    /* Configure PDM, PDM clocks, and PDM event */
    result = pdm_init();
#endif
    if(CY_RSLT_SUCCESS != result) {
        printf("Failed to init model input!\n");
    }
    return result;
}

static void app_model_task(void *pvParameters) {
    TaskHandle_t *parent_task = pvParameters;

    // Resume the application task. No longer needs to wait for us.
    xTaskNotifyGive(*parent_task);

    while (true) {
        // We guess that sensor data needs data to be passed to the model at 50hz (20ms period)
        // unsure about this....
        vTaskDelay(pdMS_TO_TICKS(20));

        if (processing) {
        	app_model_process();
        }
    }
    while (1) {
        taskYIELD();
    }
}

void app_task(void *pvParameters) {
    (void) pvParameters;
    int count = 0;

    UBaseType_t my_priority = uxTaskPriorityGet(NULL);
    TaskHandle_t my_task = xTaskGetCurrentTaskHandle();

    /* Port and pin translations for the USER_LED */
    GPIO_PRT_Type *CYBSP_USER_LED_PORT = CYHAL_GET_PORTADDR(CYBSP_USER_LED);
    uint8_t CYBSP_USER_LED_PIN = CYHAL_GET_PIN(CYBSP_USER_LED);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    // printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
    printf("Starting The App Task\n");
    printf("===============================================================\n\n");

    if (app_model_init()) {
    	// called function will print the error
        return;
    }

    if (app_eeprom_data_init()){
    	printf("App EEPROM data init failed!\n");
    }

    /* Initialize the User Button */
    // ...and keys
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_init(P9_0, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_init(P9_1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    /* Enable the GPIO interrupt to wake-up the device */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_enable_event(P9_0, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_enable_event(P9_1, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    Cy_GPIO_Pin_FastInit(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CY_GPIO_DM_STRONG, 1UL, HSIOM_SEL_GPIO);
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);

    printf("Current Settings:\n");
    printf("WiFi SSID: %s\n", app_eeprom_data_get_wifi_ssid(WIFI_SSID));

    cy_wcm_config_t wcm_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    if (CY_RSLT_SUCCESS != cy_wcm_init(&wcm_config)) {
        printf("Error: Wi-Fi Connection Manager initialization failed!\n");
        goto exit_cleanup;
    }

    printf("Wi-Fi Connection Manager initialized.\n");

    /* Initiate connection to the Wi-Fi AP and cleanup if the operation fails. */
    if (CY_RSLT_SUCCESS != wifi_connect()) {
        goto exit_cleanup;
    }

    if (!model_task_handle) {
    	xTaskCreate(app_model_task, "Model Task", 1024 * 16, &my_task, my_priority + 1, &model_task_handle);
    	ulTaskNotifyTake(pdTRUE, 10000);
    }

    // app task loop: wait for button presses
    for (;;) {
        switch (get_switch_event()) {
            case SWITCH_QUICK_PRESS:
            case SWITCH_SHORT_PRESS:
            	MEMFAULT_LOG_INFO("short press.");
				if (processing) {
					processing = false;
					MEMFAULT_METRICS_SESSION_END(listening);
				    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
				} else {
					processing = true;
					MEMFAULT_METRICS_SESSION_START(listening);
					// Start w/ a zero value, so it has a value in case no cries are detected:
					MEMFAULT_METRIC_SESSION_SET_UNSIGNED(cries, listening, 0);
					MEMFAULT_METRIC_SESSION_SET_UNSIGNED(almostCries, listening, 0);
					MEMFAULT_METRIC_SESSION_SET_UNSIGNED(definitelyBaby, listening, 0);
					MEMFAULT_METRIC_SESSION_SET_UNSIGNED(couldBeOzzy, listening, 0);
					// Turn on LED:
				    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
				}
                break;

            case SWITCH_LONG_PRESS:
            	MEMFAULT_LOG_INFO("long press. playing fault roulette...");
            	switch(xTaskGetTickCount() % 3) {
            	case 0:
            		printf("calling test_trace()...");
            		MEMFAULT_LOG_ERROR("calling test_trace()...");
            		test_trace(1, "trace roulette");
            		break;
            	case 1:
            		printf("calling test_reboot()...");
            		MEMFAULT_LOG_ERROR("calling test_reboot()...");
            		test_reboot(1, "fault roulette");
            		break;
            	case 2:
            		printf("calling test_fault()...");
            		MEMFAULT_LOG_ERROR("calling test_fault()...");
            		test_fault();
            		break;
            	}
                break;

            case SWITCH_KEY1:
            	printf("'Baby' key pressed!\n");
				MEMFAULT_METRIC_SESSION_ADD(definitelyBaby, listening, 1);
            	break;
            case SWITCH_KEY2:
            	printf("'Metal' key pressed!\n");
				MEMFAULT_METRIC_SESSION_ADD(couldBeOzzy, listening, 1);
            	break;
            default:
                break;
        }

        vTaskDelay(50);

        // periodically do Memfault things:
        if (count++ % 2000 == 0) {
        	MEMFAULT_METRIC_ADD(mainTaskWakeups, 1);
        	memfault_http_client_post_chunk();
        }
    	if (count % 4000 == 0) {
    		memfault_log_trigger_collection();
            memfault_metrics_heartbeat_debug_trigger();
    	}
    }

    printf("\nAppTask Done.\n");
    while (1) {
        taskYIELD();
    }
    return;

    exit_cleanup:
    printf("\nError encountered. AppTask Done.\n");
    while (1) {
        taskYIELD();
    }
}

/*******************************************************************************
* Function Name: get_switch_event
****************************************************************************//**
* Summary:
*  Returns how the User button was pressed:
*  - SWITCH_NO_EVENT: No press
*  - SWITCH_QUICK_PRESS: Very quick press
*  - SWITCH_SHORT_PRESS: Short press was detected
*  - SWITCH_LONG_PRESS: Long press was detected
*
* Return:
*  Switch event that occurred, if any.
*
*******************************************************************************/
en_switch_event_t get_switch_event(void)
{
    en_switch_event_t event = SWITCH_NO_EVENT;
    uint32_t pressCount = 0;

    // Check 'extra' keys first:
    if (cyhal_gpio_read(P9_0) == CYBSP_BTN_PRESSED) {
    	event = SWITCH_KEY1;
    }
    if (cyhal_gpio_read(P9_1) == CYBSP_BTN_PRESSED) {
    	event = SWITCH_KEY2;
    }

    /* Check if User button is pressed */
    while (cyhal_gpio_read(CYBSP_USER_BTN) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        cyhal_system_delay_ms(USER_BTN_PRESS_DELAY);

        /* Increment counter. Each count represents 10 ms */
        pressCount++;
    }

    /* Check for how long the button was pressed */
    if (pressCount > LONG_PRESS_COUNT)
    {
        event = SWITCH_LONG_PRESS;
    }
    else if (pressCount > SHORT_PRESS_COUNT)
    {
        event = SWITCH_SHORT_PRESS;
    }
    else if (pressCount > QUICK_PRESS_COUNT)
    {
        event = SWITCH_QUICK_PRESS;
    }

    /* Add a delay to avoid glitches */
    cyhal_system_delay_ms(SHORT_GLITCH_DELAY_MS);

    return event;
}
