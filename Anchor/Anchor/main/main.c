#include "dwm3000.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>

// ==================== Configuration ====================

#define ANCHOR_ID 3
#define RESPONSE_TIMEOUT_MS 10
#define MAX_RETRIES 3

// ==================== Global Variables ====================

static int rx_status;
static int tx_status;

static int curr_stage = 0;

static int t_roundB = 0;
static int t_replyB = 0;

static long long rx = 0;
static long long tx = 0;

static unsigned long last_ranging_time = 0;
static int retry_count = 0;

// ==================== Helper Functions ====================

static unsigned long millis_now(void) {
  return (unsigned long)(esp_timer_get_time() / 1000);
}

// static void reset_radio(void) {
//   printf("[INFO] Performing radio reset...\n");
//   dwm3000_soft_reset();
//   vTaskDelay(pdMS_TO_TICKS(100));
//   dwm3000_clear_system_status();
//   dwm3000_configure_as_tx();
//   dwm3000_standard_rx();
// }

static void reset_radio(void) {
  printf("[INFO] Performing radio reset...\n");
  dwm3000_soft_reset();
  vTaskDelay(pdMS_TO_TICKS(200));
  dwm3000_init();
  dwm3000_setup_gpio();
  dwm3000_set_tx_antenna_delay(16350);
  dwm3000_clear_system_status();
  printf("[INFO] Radio reset complete.\n");
}
// ==================== Main Application ====================

void app_main(void) {
  dwm3000_begin();
  dwm3000_hard_reset();
  vTaskDelay(pdMS_TO_TICKS(200));

  if (!dwm3000_check_spi()) {
    printf("[ERROR] Could not establish SPI Connection to DWM3000!\n");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  while (!dwm3000_check_for_idle()) {
    printf("[ERROR] IDLE1 FAILED\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  dwm3000_soft_reset();
  vTaskDelay(pdMS_TO_TICKS(200));

  if (!dwm3000_check_for_idle()) {
    printf("[ERROR] IDLE2 FAILED\n");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  dwm3000_init();
  dwm3000_setup_gpio();

  dwm3000_set_tx_antenna_delay(16350);
  dwm3000_set_sender_id(ANCHOR_ID);

  printf("> ANCHOR %d - Ready for ranging <\n", ANCHOR_ID);
  printf("Antenna delay set to: %d\n", dwm3000_get_tx_antenna_delay());
  printf("[INFO] Setup finished.\n");

  dwm3000_configure_as_tx();
  dwm3000_clear_system_status();
  dwm3000_standard_rx();

  // Main loop
  while (1) {
    if (dwm3000_received_frame_succ() == 1 && dwm3000_ds_get_stage() == 1 &&
        dwm3000_get_destination_id() == ANCHOR_ID) {
      // Reset session if new ranging request arrives
      if (curr_stage != 0) {
        printf("[INFO] New request - resetting session\n");
        curr_stage = 0;
        t_roundB = 0;
        t_replyB = 0;
      }
    }

    switch (curr_stage) {
    case 0: // Await ranging
      t_roundB = 0;
      t_replyB = 0;
      rx_status = dwm3000_received_frame_succ();
      if (rx_status) {
        dwm3000_clear_system_status();
        if (rx_status == 1) {
          last_ranging_time = millis_now();
          if (dwm3000_get_destination_id() == ANCHOR_ID) {
            if (dwm3000_ds_is_error_frame()) {
              printf("[WARNING] Received error frame!\n");
              curr_stage = 0;
              dwm3000_standard_rx();
            } else if (dwm3000_ds_get_stage() != 1) {
              printf("[WARNING] Unexpected stage: %d\n",
                     dwm3000_ds_get_stage());
              dwm3000_ds_send_error_frame();
              dwm3000_standard_rx();
              curr_stage = 0;
            } else {
              curr_stage = 1;
            }
          } else {
            dwm3000_standard_rx();
          }
        } else {
          printf("[ERROR] Receiver Error occurred!\n");
          dwm3000_clear_system_status();
        }
      } else if (millis_now() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        printf("[ERROR] Resetting radio\n");
        retry_count = 0;
        dwm3000_standard_rx();
        last_ranging_time = millis_now();
      }
      break;

    case 1: // Ranging received. Sending response
      dwm3000_ds_send_frame(2);

      rx = dwm3000_read_rx_timestamp();
      tx = dwm3000_read_tx_timestamp();

      t_replyB = tx - rx;
      curr_stage = 2;
      last_ranging_time = millis_now();
      break;

    case 2: // Awaiting response
      rx_status = dwm3000_received_frame_succ();
      if (rx_status) {
        retry_count = 0;
        dwm3000_clear_system_status();
        if (rx_status == 1) {
          if (dwm3000_ds_is_error_frame()) {
            printf("[WARNING] Received error frame!\n");
            curr_stage = 0;
            dwm3000_standard_rx();
          } else if (dwm3000_ds_get_stage() != 3) {
            printf("[WARNING] Unexpected stage: %d\n", dwm3000_ds_get_stage());
            dwm3000_ds_send_error_frame();
            dwm3000_standard_rx();
            curr_stage = 0;
          } else {
            curr_stage = 3;
          }
        } else {
          printf("[ERROR] Receiver Error occurred!\n");
          dwm3000_clear_system_status();
        }
      } else if (millis_now() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        printf("[ERROR] Resetting radio\n");
        retry_count = 0;
        dwm3000_standard_rx();
        last_ranging_time = millis_now();
      }
      break;

    case 3: // Second response received. Sending information frame
      rx = dwm3000_read_rx_timestamp();
      t_roundB = rx - tx;
      dwm3000_ds_send_rt_info(t_roundB, t_replyB);

      curr_stage = 0;
      dwm3000_standard_rx();
      break;

    default:
      printf("[ERROR] Entered unknown stage (%d). Reverting back to stage 0\n",
             curr_stage);
      curr_stage = 0;
      dwm3000_standard_rx();
      break;
    }
  }
}
