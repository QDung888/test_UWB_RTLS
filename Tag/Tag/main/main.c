#include "dwm3000.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// ==================== Configuration ====================

#define NUM_ANCHORS 3
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1
#define RESPONSE_TIMEOUT_MS 50
#define MAX_RETRIES 3

#define FILTER_SIZE 30
#define MIN_DISTANCE 0
#define MAX_DISTANCE 1000.0

// ==================== Data Structures ====================

typedef struct {
  int anchor_id;

  // Timing measurements
  int t_roundA;
  int t_replyA;
  long long rx;
  long long tx;
  int clock_offset;

  // Distance measurements
  float distance;
  float distance_history[FILTER_SIZE];
  int history_index;
  float filtered_distance;

  // Signal quality metrics
  float signal_strength;
  float fp_signal_strength;
} AnchorData;

// ==================== Global Variables ====================

static int rx_status;
static int tx_status;
static int current_anchor_index = 0;
static int curr_stage = 0;

static unsigned long last_ranging_time = 0;
static int retry_count = 0;

static AnchorData anchors[NUM_ANCHORS];

// ==================== Helper Functions ====================

static unsigned long millis_now(void) {
  return (unsigned long)(esp_timer_get_time() / 1000);
}

static void reset_radio(void) {
  printf("[INFO] Performing radio reset...\n");
  dwm3000_soft_reset();
  vTaskDelay(pdMS_TO_TICKS(200));
  dwm3000_init();
  dwm3000_setup_gpio();
  dwm3000_set_tx_antenna_delay(16350);
  dwm3000_configure_as_tx();
  dwm3000_clear_system_status();
  printf("[INFO] Radio reset complete.\n");
}

static void initialize_anchors(void) {
  for (int i = 0; i < NUM_ANCHORS; i++) {
    memset(&anchors[i], 0, sizeof(AnchorData));
    anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
  }
}

static AnchorData *get_current_anchor(void) {
  return &anchors[current_anchor_index];
}

static int get_current_anchor_id(void) {
  return anchors[current_anchor_index].anchor_id;
}

static void switch_to_next_anchor(void) {
  current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

static bool all_anchors_have_valid_data(void) {
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (anchors[i].filtered_distance <= 0) {
      return false;
    }
  }
  return true;
}

static bool is_valid_distance(float distance) {
  return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

static float calculate_median(float arr[], int size) {
  float temp[FILTER_SIZE];
  for (int i = 0; i < size; i++) {
    temp[i] = arr[i];
  }

  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  if (size % 2 == 0) {
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
  } else {
    return temp[size / 2];
  }
}

static void update_filtered_distance(AnchorData *data) {
  data->distance_history[data->history_index] = data->distance;
  data->history_index = (data->history_index + 1) % FILTER_SIZE;

  float valid_distances[FILTER_SIZE];
  int valid_count = 0;

  for (int i = 0; i < FILTER_SIZE; i++) {
    if (is_valid_distance(data->distance_history[i])) {
      valid_distances[valid_count++] = data->distance_history[i];
    }
  }

  if (valid_count > 0) {
    data->filtered_distance = calculate_median(valid_distances, valid_count);
  } else {
    data->filtered_distance = 0;
  }
}

static void print_all_distances(void) {
  printf("Distances - ");
  for (int i = 0; i < NUM_ANCHORS; i++) {
    printf("A%d: ", anchors[i].anchor_id);
    if (anchors[i].filtered_distance > 0) {
      dwm3000_print_double(anchors[i].filtered_distance, 100, false);
      printf(" cm");
    } else {
      printf("INVALID");
    }

    if (i < NUM_ANCHORS - 1) {
      printf(" | ");
    }
  }
  printf("\n");
}

// ==================== Main Application ====================

void app_main(void) {
  // Initialize anchor array
  initialize_anchors();

  printf("Initialized %d anchors:\n", NUM_ANCHORS);
  for (int i = 0; i < NUM_ANCHORS; i++) {
    printf("  Anchor %d - ID: %d\n", i, anchors[i].anchor_id);
  }

  // Initialize UWB
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
  dwm3000_set_sender_id(TAG_ID);

  printf("> TAG - Three Anchor Ranging System <\n");
  printf("[INFO] Setup is finished.\n");
  printf("Antenna delay set to: %d\n", dwm3000_get_tx_antenna_delay());

  dwm3000_configure_as_tx();
  dwm3000_clear_system_status();

  // Main loop
  while (1) {
    AnchorData *currentAnchor = get_current_anchor();
    int currentAnchorId = get_current_anchor_id();

    switch (curr_stage) {
    case 0: // Start ranging with current target
      currentAnchor->t_roundA = 0;
      currentAnchor->t_replyA = 0;

      dwm3000_set_destination_id(currentAnchorId);
      dwm3000_ds_send_frame(1);
      currentAnchor->tx = dwm3000_read_tx_timestamp();
      curr_stage = 1;
      break;

    case 1: // Await first response
      rx_status = dwm3000_received_frame_succ();
      // last_ranging_time = millis_now();
      if (rx_status) {
        dwm3000_clear_system_status();
        if (rx_status == 1) {
          last_ranging_time = millis_now();
          if (dwm3000_ds_is_error_frame()) {
            printf("[WARNING] Error frame from Anchor %d! Signal strength: ",
                   currentAnchorId);
            dwm3000_print_double(dwm3000_get_signal_strength(), 100, false);
            printf(" dBm\n");
            curr_stage = 0;
          } else if (dwm3000_ds_get_stage() != 2) {
            printf("[WARNING] Unexpected stage from Anchor %d: %d\n",
                   currentAnchorId, dwm3000_ds_get_stage());
            dwm3000_ds_send_error_frame();
            curr_stage = 0;
          } else {
            curr_stage = 2;
          }
        } else {
          printf("[ERROR] Receiver Error from Anchor %d\n", currentAnchorId);
          dwm3000_clear_system_status();
        }
      } else if (millis_now() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        printf("[WARNING] Timeout waiting for ranging request\n");
        last_ranging_time = millis_now();
        if (++retry_count > MAX_RETRIES) {
          printf("[ERROR] Max retries reached, resetting radio\n");
          reset_radio();
          retry_count = 0;
          curr_stage = 0;
        }
      }
      break;

    case 2: // Response received. Send second ranging
      currentAnchor->rx = dwm3000_read_rx_timestamp();
      dwm3000_ds_send_frame(3);
      currentAnchor->t_roundA = currentAnchor->rx - currentAnchor->tx;
      currentAnchor->tx = dwm3000_read_tx_timestamp();
      currentAnchor->t_replyA = currentAnchor->tx - currentAnchor->rx;
      last_ranging_time = millis_now();
      curr_stage = 3;
      break;

    case 3: // Await second response
      rx_status = dwm3000_received_frame_succ();
      if (rx_status) {
        dwm3000_clear_system_status();
        if (rx_status == 1) {
          if (dwm3000_ds_is_error_frame()) {
            printf("[WARNING] Error frame from Anchor %d\n", currentAnchorId);
            curr_stage = 0;
          } else {
            currentAnchor->clock_offset = dwm3000_get_raw_clock_offset();
            curr_stage = 4;
          }
        } else {
          printf("[ERROR] Receiver Error from Anchor %d\n", currentAnchorId);
          dwm3000_clear_system_status();
        }
      } else if (millis_now() - last_ranging_time > RESPONSE_TIMEOUT_MS) {
        printf("[WARNING] Timeout waiting for ranging request\n");
        last_ranging_time = millis_now();
        if (++retry_count > MAX_RETRIES) {
          printf("[ERROR] Max retries reached, resetting radio\n");
          reset_radio();
          retry_count = 0;
          curr_stage = 0;
        }
      }
      break;

    case 4: // Response received. Calculating results
    {
      int ranging_time = dwm3000_ds_process_rt_info(
          currentAnchor->t_roundA, currentAnchor->t_replyA,
          dwm3000_read_reg(0x12, 0x04), dwm3000_read_reg(0x12, 0x08),
          currentAnchor->clock_offset);

      currentAnchor->distance = dwm3000_convert_to_cm(ranging_time);
      currentAnchor->signal_strength = dwm3000_get_signal_strength();
      currentAnchor->fp_signal_strength =
          dwm3000_get_first_path_signal_strength();
      update_filtered_distance(currentAnchor);
    }

      print_all_distances();

      // Switch to next anchor
      switch_to_next_anchor();
      curr_stage = 0;
      break;

    default:
      printf("Entered stage (%d). Reverting back to stage 0\n", curr_stage);
      curr_stage = 0;
      break;
    }
  }
}
