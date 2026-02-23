#include "dwm3000.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


// ==================== Static Variables ====================

static int config[] = {
    CHANNEL_5,         // Channel
    PREAMBLE_128,      // Preamble Length
    9,                 // Preamble Code (Same for RX and TX!)
    PAC8,              // PAC
    DATARATE_6_8MB,    // Datarate
    PHR_MODE_STANDARD, // PHR Mode
    PHR_RATE_850KB     // PHR Rate
};

static int ANTENNA_DELAY = 16350;
static int led_status = 0;
static int destination = 0x0;
static int sender = 0x0;

static spi_device_handle_t spi_handle = NULL;

// ==================== Private Function Declarations ====================

static void set_bit(int reg_addr, int sub_addr, int shift, bool b);
static void set_bit_low(int reg_addr, int sub_addr, int shift);
static void set_bit_high(int reg_addr, int sub_addr, int shift);
static void write_fast_command(int cmd);
static uint32_t read_or_write_full_address(uint32_t base, uint32_t sub,
                                           uint32_t data, uint32_t data_len,
                                           uint32_t readWriteBit);
static uint32_t send_bytes(int b[], int lenB, int recLen);
static void clear_aon_config(void);
static unsigned int count_bits(unsigned int number);
static int check_for_dev_id(void);

// ==================== Helper ====================

static void delay_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ==================== Chip Setup ====================

void dwm3000_spi_select(uint8_t cs) {
  gpio_set_direction(cs, GPIO_MODE_OUTPUT);
  gpio_set_level(cs, 1);
  delay_ms(5);
}

void dwm3000_begin(void) {
  delay_ms(5);

  gpio_set_direction(DWM3000_PIN_CS, GPIO_MODE_OUTPUT);

  spi_bus_config_t buscfg = {
      .mosi_io_num = DWM3000_PIN_MOSI,
      .miso_io_num = DWM3000_PIN_MISO,
      .sclk_io_num = DWM3000_PIN_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
      .mode = 0,                         // SPI mode 0
      .spics_io_num = -1,                // Manual CS control
      .queue_size = 1,
  };

  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);

  delay_ms(5);

  dwm3000_spi_select(DWM3000_PIN_CS);
  printf("[INFO] SPI ready\n");
}

void dwm3000_init(void) {
  if (!check_for_dev_id()) {
    printf("[ERROR] Dev ID is wrong! Aborting!\n");
    return;
  }

  set_bit_high(GEN_CFG_AES_LOW_REG, 0x10, 4);

  while (!dwm3000_check_for_idle()) {
    printf("[WARNING] IDLE FAILED (stage 1)\n");
    delay_ms(100);
  }

  dwm3000_soft_reset();
  delay_ms(200);

  while (!dwm3000_check_for_idle()) {
    printf("[WARNING] IDLE FAILED (stage 2)\n");
    delay_ms(100);
  }

  uint32_t ldo_low = dwm3000_read_otp(0x04);
  uint32_t ldo_high = dwm3000_read_otp(0x05);
  uint32_t bias_tune = dwm3000_read_otp(0xA);
  bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

  if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
    dwm3000_write_reg_auto(0x11, 0x1F, bias_tune);
    dwm3000_write_reg_auto(0x0B, 0x08, 0x0100);
  }

  int xtrim_value = dwm3000_read_otp(0x1E);
  xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;
  dwm3000_write_reg_auto(FS_CTRL_REG, 0x14, xtrim_value);
  if (DEBUG_OUTPUT)
    printf("xtrim: %d\n", xtrim_value);

  dwm3000_write_sys_config();
  dwm3000_write_reg_auto(0x00, 0x3C, 0xFFFFFFFF);
  dwm3000_write_reg_auto(0x00, 0x40, 0xFFFF);
  dwm3000_write_reg(0x0A, 0x00, 0x000900, 3);

  dwm3000_write_reg_auto(0x3, 0x1C, 0x10000240);
  dwm3000_write_reg_auto(0x3, 0x20, 0x1B6DA489);
  dwm3000_write_reg_auto(0x3, 0x38, 0x0001C0FD);
  dwm3000_write_reg_auto(0x3, 0x3C, 0x0001C43E);
  dwm3000_write_reg_auto(0x3, 0x40, 0x0001C6BE);
  dwm3000_write_reg_auto(0x3, 0x44, 0x0001C77E);
  dwm3000_write_reg_auto(0x3, 0x48, 0x0001CF36);
  dwm3000_write_reg_auto(0x3, 0x4C, 0x0001CFB5);
  dwm3000_write_reg_auto(0x3, 0x50, 0x0001CFF5);
  dwm3000_write_reg_auto(0x3, 0x18, 0xE5E5);
  int f = dwm3000_read_reg(0x4, 0x20);
  (void)f;
  dwm3000_write_reg_auto(0x6, 0x0, 0x81101C);
  dwm3000_write_reg_auto(0x07, 0x34, 0x4);
  dwm3000_write_reg_auto(0x07, 0x48, 0x14);
  dwm3000_write_reg_auto(0x07, 0x1A, 0x0E);
  dwm3000_write_reg_auto(0x07, 0x1C, 0x1C071134);
  dwm3000_write_reg_auto(0x09, 0x00, 0x1F3C);
  dwm3000_write_reg_auto(0x09, 0x80, 0x81);
  dwm3000_write_reg_auto(0x11, 0x04, 0xB40200);
  dwm3000_write_reg_auto(0x11, 0x08, 0x80030738);
  printf("[INFO] Initialization finished.\n\n");
}

void dwm3000_write_sys_config(void) {
  int usr_cfg =
      (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
  dwm3000_write_reg_auto(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);

  if (config[2] > 24) {
    printf("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!\n");
  }

  int otp_write = 0x1400;
  if (config[1] >= 256) {
    otp_write |= 0x04;
  }

  dwm3000_write_reg_auto(OTP_IF_REG, 0x08, otp_write);
  dwm3000_write_reg(DRX_REG, 0x00, 0x00, 1);
  dwm3000_write_reg_auto(DRX_REG, 0x0, config[3]);
  dwm3000_write_reg_auto(STS_CFG_REG, 0x0, 64 / 8 - 1);
  dwm3000_write_reg(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
  dwm3000_write_reg_auto(DRX_REG, 0x0C, 0xAF5F584C);

  int chan_ctrl_val = dwm3000_read_reg(GEN_CFG_AES_HIGH_REG, 0x14);
  chan_ctrl_val &= (~0x1FFF);
  chan_ctrl_val |= config[0];
  chan_ctrl_val |= 0x1F00 & (config[2] << 8);
  chan_ctrl_val |= 0xF8 & (config[2] << 3);
  chan_ctrl_val |= 0x06 & (0x01 << 1);
  dwm3000_write_reg_auto(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val);

  int tx_fctrl_val = dwm3000_read_reg(GEN_CFG_AES_LOW_REG, 0x24);
  tx_fctrl_val |= (config[1] << 12);
  tx_fctrl_val |= (config[4] << 10);
  dwm3000_write_reg_auto(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val);
  dwm3000_write_reg_auto(DRX_REG, 0x02, 0x81);

  int rf_tx_ctrl_2 = 0x1C071134;
  int pll_conf = 0x0F3C;
  if (config[0]) {
    rf_tx_ctrl_2 &= ~0x00FFFF;
    rf_tx_ctrl_2 |= 0x000001;
    pll_conf &= 0x00FF;
    pll_conf |= 0x001F;
  }

  dwm3000_write_reg_auto(RF_CONF_REG, 0x1C, rf_tx_ctrl_2);
  dwm3000_write_reg_auto(FS_CTRL_REG, 0x00, pll_conf);
  dwm3000_write_reg_auto(RF_CONF_REG, 0x51, 0x14);
  dwm3000_write_reg_auto(RF_CONF_REG, 0x1A, 0x0E);
  dwm3000_write_reg_auto(FS_CTRL_REG, 0x08, 0x81);
  dwm3000_write_reg_auto(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
  dwm3000_write_reg_auto(PMSC_REG, 0x04, 0x300200);
  dwm3000_write_reg_auto(PMSC_REG, 0x08, 0x0138);

  int success = 0;
  for (int i = 0; i < 100; i++) {
    if (dwm3000_read_reg(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) {
      success = 1;
      break;
    }
  }

  if (!success) {
    printf("[ERROR] Couldn't lock PLL Clock!\n");
  } else {
    printf("[INFO] PLL is now locked.\n");
  }

  int otp_val = dwm3000_read_reg(OTP_IF_REG, 0x08);
  otp_val |= 0x40;
  if (config[0])
    otp_val |= 0x2000;
  dwm3000_write_reg_auto(OTP_IF_REG, 0x08, otp_val);
  dwm3000_write_reg_auto(RX_TUNE_REG, 0x19, 0xF0);

  int ldo_ctrl_val = dwm3000_read_reg(RF_CONF_REG, 0x48);
  int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);
  dwm3000_write_reg_auto(RF_CONF_REG, 0x48, tmp_ldo);
  dwm3000_write_reg_auto(EXT_SYNC_REG, 0x0C, 0x020000);
  int l = dwm3000_read_reg(0x04, 0x0C);
  (void)l;
  delay_ms(20);
  dwm3000_write_reg_auto(EXT_SYNC_REG, 0x0C, 0x11);

  int succ = 0;
  for (int i = 0; i < 100; i++) {
    if (dwm3000_read_reg(EXT_SYNC_REG, 0x20)) {
      succ = 1;
      break;
    }
    delay_ms(10);
  }

  if (succ) {
    printf("[INFO] PGF calibration complete.\n");
  } else {
    printf("[ERROR] PGF calibration failed!\n");
  }

  dwm3000_write_reg_auto(EXT_SYNC_REG, 0x0C, 0x00);
  dwm3000_write_reg_auto(EXT_SYNC_REG, 0x20, 0x01);

  int rx_cal_res = dwm3000_read_reg(EXT_SYNC_REG, 0x14);
  if (rx_cal_res == 0x1fffffff) {
    printf("[ERROR] PGF_CAL failed in stage I!\n");
  }
  rx_cal_res = dwm3000_read_reg(EXT_SYNC_REG, 0x1C);
  if (rx_cal_res == 0x1fffffff) {
    printf("[ERROR] PGF_CAL failed in stage Q!\n");
  }

  dwm3000_write_reg_auto(RF_CONF_REG, 0x48, ldo_ctrl_val);
  dwm3000_write_reg_auto(0x0E, 0x02, 0x01);
  dwm3000_set_tx_antenna_delay(ANTENNA_DELAY);
}

void dwm3000_configure_as_tx(void) {
  dwm3000_write_reg_auto(RF_CONF_REG, 0x1C, 0x34);
  dwm3000_write_reg_auto(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}

void dwm3000_setup_gpio(void) { dwm3000_write_reg_auto(0x05, 0x08, 0xF0); }

// ==================== Double-Sided Ranging ====================

void dwm3000_ds_send_frame(int stage) {
  dwm3000_set_mode(1);
  dwm3000_write_reg_auto(0x14, 0x01, sender & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x02, destination & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x03, stage & 0x7);
  dwm3000_set_frame_length(4);
  dwm3000_tx_instant_rx();

  bool error = true;
  for (int i = 0; i < 50; i++) {
    if (dwm3000_sent_frame_succ()) {
      error = false;
      break;
    }
  }
  if (error) {
    printf("[ERROR] Could not send frame successfully!\n");
  }
}

void dwm3000_ds_send_rt_info(int t_roundB, int t_replyB) {
  dwm3000_set_mode(1);
  dwm3000_write_reg_auto(0x14, 0x01, destination & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x02, sender & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x03, 4);
  dwm3000_write_reg_auto(0x14, 0x04, t_roundB);
  dwm3000_write_reg_auto(0x14, 0x08, t_replyB);
  dwm3000_set_frame_length(12);
  dwm3000_tx_instant_rx();
}

int dwm3000_ds_process_rt_info(int t_roundA, int t_replyA, int t_roundB,
                               int t_replyB, int clk_offset) {
  if (DEBUG_OUTPUT) {
    printf("\nProcessing Information:\n");
    printf("t_roundA: %d\n", t_roundA);
    printf("t_replyA: %d\n", t_replyA);
    printf("t_roundB: %d\n", t_roundB);
    printf("t_replyB: %d\n", t_replyB);
  }

  int reply_diff = t_replyA - t_replyB;
  long double clock_offset =
      t_replyA > t_replyB ? 1.0 + dwm3000_get_clock_offset_ext(clk_offset)
                          : 1.0 - dwm3000_get_clock_offset_ext(clk_offset);
  int first_rt = t_roundA - t_replyB;
  int second_rt = t_roundB - t_replyA;
  int combined_rt =
      (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
  int combined_rt_raw = (first_rt + second_rt) / 2;
  (void)combined_rt_raw;
  return combined_rt / 2;
}

int dwm3000_ds_get_stage(void) { return dwm3000_read_reg(0x12, 0x03) & 0b111; }

bool dwm3000_ds_is_error_frame(void) {
  return ((dwm3000_read_reg(0x12, 0x00) & 0x7) == 7);
}

void dwm3000_ds_send_error_frame(void) {
  printf("[WARNING] Error Frame sent. Reverting back to stage 0.\n");
  dwm3000_set_mode(7);
  dwm3000_set_frame_length(3);
  dwm3000_standard_tx();
}

// ==================== Radio Settings ====================

void dwm3000_set_channel(uint8_t data) {
  if (data == CHANNEL_5 || data == CHANNEL_9)
    config[0] = data;
}

void dwm3000_set_preamble_length(uint8_t data) {
  if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 ||
      data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 ||
      data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096)
    config[1] = data;
}

void dwm3000_set_preamble_code(uint8_t data) {
  if (data <= 12 && data >= 9)
    config[2] = data;
}

void dwm3000_set_pac_size(uint8_t data) {
  if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32)
    config[3] = data;
}

void dwm3000_set_datarate(uint8_t data) {
  if (data == DATARATE_6_8MB || data == DATARATE_850KB)
    config[4] = data;
}

void dwm3000_set_phr_mode(uint8_t data) {
  if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG)
    config[5] = data;
}

void dwm3000_set_phr_rate(uint8_t data) {
  if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB)
    config[6] = data;
}

// ==================== Protocol Settings ====================

void dwm3000_set_mode(int mode) {
  dwm3000_write_reg_auto(0x14, 0x00, mode & 0x7);
}

void dwm3000_set_tx_frame(unsigned long long frame_data) {
  if (frame_data > ((unsigned long long)(pow(2, 8 * 8) - FCS_LEN))) {
    printf("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!\n");
    return;
  }
  dwm3000_write_reg_auto(TX_BUFFER_REG, 0x00, (uint32_t)frame_data);
}

void dwm3000_set_frame_length(int frameLen) {
  frameLen = frameLen + FCS_LEN;
  int curr_cfg = dwm3000_read_reg(0x00, 0x24);
  if (frameLen > 1023) {
    printf(
        "[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!\n");
    return;
  }
  int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frameLen;
  dwm3000_write_reg_auto(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg);
}

void dwm3000_set_tx_antenna_delay(int delay) {
  ANTENNA_DELAY = delay;
  dwm3000_write_reg_auto(0x01, 0x04, delay);
}

void dwm3000_set_sender_id(int senderID) { sender = senderID; }

void dwm3000_set_destination_id(int destID) { destination = destID; }

// ==================== Status Checks ====================

int dwm3000_received_frame_succ(void) {
  int sys_stat = dwm3000_read_reg(GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) {
    return 1;
  } else if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
    return 2;
  }
  return 0;
}

int dwm3000_sent_frame_succ(void) {
  int sys_stat = dwm3000_read_reg(GEN_CFG_AES_LOW_REG, 0x44);
  if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) {
    return 1;
  }
  return 0;
}

int dwm3000_get_sender_id(void) { return dwm3000_read_reg(0x12, 0x01) & 0xFF; }

int dwm3000_get_destination_id(void) {
  return dwm3000_read_reg(0x12, 0x02) & 0xFF;
}

bool dwm3000_check_for_idle(void) {
  return (dwm3000_read_reg(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) ==
                     PMSC_STATE_IDLE ||
                 (dwm3000_read_reg(0x00, 0x44) >> 16 &
                  (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK)
             ? 1
             : 0;
}

bool dwm3000_check_spi(void) { return check_for_dev_id(); }

// ==================== Radio Analytics ====================

double dwm3000_get_signal_strength(void) {
  int CIRpower = dwm3000_read_reg(0x0C, 0x2C) & 0x1FF;
  int PAC_val = dwm3000_read_reg(0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (dwm3000_read_reg(0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;
  return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) +
         (6 * DGC_decision) - PRF_const;
}

double dwm3000_get_first_path_signal_strength(void) {
  float f1 = (dwm3000_read_reg(0x0C, 0x30) & 0x3FFFFF) >> 2;
  float f2 = (dwm3000_read_reg(0x0C, 0x34) & 0x3FFFFF) >> 2;
  float f3 = (dwm3000_read_reg(0x0C, 0x38) & 0x3FFFFF) >> 2;
  int PAC_val = dwm3000_read_reg(0x0C, 0x58) & 0xFFF;
  unsigned int DGC_decision = (dwm3000_read_reg(0x03, 0x60) >> 28) & 0x7;
  double PRF_const = 121.7;
  return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) +
         (6 * DGC_decision) - PRF_const;
}

int dwm3000_get_tx_antenna_delay(void) {
  int delay = dwm3000_read_reg(0x01, 0x04) & 0xFFFF;
  return delay;
}

long double dwm3000_get_clock_offset(void) {
  if (config[0] == CHANNEL_5) {
    return dwm3000_get_raw_clock_offset() * CLOCK_OFFSET_CHAN_5_CONSTANT /
           1000000;
  } else {
    return dwm3000_get_raw_clock_offset() * CLOCK_OFFSET_CHAN_9_CONSTANT /
           1000000;
  }
}

long double dwm3000_get_clock_offset_ext(int32_t sec_clock_offset) {
  if (config[0] == CHANNEL_5) {
    return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
  } else {
    return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
  }
}

int dwm3000_get_raw_clock_offset(void) {
  int raw_offset = dwm3000_read_reg(0x06, 0x29) & 0x1FFFFF;
  if (raw_offset & (1 << 20)) {
    raw_offset |= ~((1 << 21) - 1);
  }
  if (DEBUG_OUTPUT) {
    printf("Raw offset: %d\n", raw_offset);
  }
  return raw_offset;
}

float dwm3000_get_temp_in_c(void) {
  dwm3000_write_reg_auto(0x07, 0x34, 0x04);
  dwm3000_write_reg_auto(0x08, 0x00, 0x01);
  while (!(dwm3000_read_reg(0x08, 0x04) & 0x01)) {
  }
  int res = dwm3000_read_reg(0x08, 0x08);
  res = (res & 0xFF00) >> 8;
  int otp_temp = dwm3000_read_otp(0x09) & 0xFF;
  float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;
  dwm3000_write_reg(0x08, 0x00, 0x00, 1);
  return tmp;
}

unsigned long long dwm3000_read_rx_timestamp(void) {
  uint32_t ts_low = dwm3000_read_reg(0x0C, 0x00);
  unsigned long long ts_high = dwm3000_read_reg(0x0C, 0x04) & 0xFF;
  unsigned long long rx_timestamp = (ts_high << 32) | ts_low;
  return rx_timestamp;
}

unsigned long long dwm3000_read_tx_timestamp(void) {
  unsigned long long ts_low = dwm3000_read_reg(0x00, 0x74);
  unsigned long long ts_high = dwm3000_read_reg(0x00, 0x78) & 0xFF;
  unsigned long long tx_timestamp = (ts_high << 32) + ts_low;
  return tx_timestamp;
}

// ==================== Chip Interaction ====================

uint32_t dwm3000_write_reg(int base, int sub, uint32_t data, int dataLen) {
  return read_or_write_full_address(base, sub, data, dataLen, 1);
}

uint32_t dwm3000_write_reg_auto(int base, int sub, uint32_t data) {
  return read_or_write_full_address(base, sub, data, 0, 1);
}

uint32_t dwm3000_read_reg(int base, int sub) {
  uint32_t tmp;
  tmp = read_or_write_full_address(base, sub, 0, 0, 0);
  if (DEBUG_OUTPUT)
    printf("\n");
  return tmp;
}

uint8_t dwm3000_read8bit(int base, int sub) {
  return (uint8_t)(dwm3000_read_reg(base, sub) >> 24);
}

uint32_t dwm3000_read_otp(uint8_t addr) {
  dwm3000_write_reg_auto(OTP_IF_REG, 0x04, addr);
  dwm3000_write_reg_auto(OTP_IF_REG, 0x08, 0x02);
  return dwm3000_read_reg(OTP_IF_REG, 0x10);
}

// ==================== Delayed Sending Settings ====================

void dwm3000_write_tx_delay(uint32_t delay) {
  dwm3000_write_reg_auto(0x00, 0x2C, delay);
}

void dwm3000_prepare_delayed_tx(void) {
  long long rx_ts = dwm3000_read_rx_timestamp();
  uint32_t exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;
  long long calc_tx_timestamp =
      ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + ANTENNA_DELAY;
  uint32_t reply_delay = calc_tx_timestamp - rx_ts;
  dwm3000_write_reg_auto(0x14, 0x01, sender & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x02, destination & 0xFF);
  dwm3000_write_reg_auto(0x14, 0x03, reply_delay);
  dwm3000_set_frame_length(7);
  dwm3000_write_tx_delay(exact_tx_timestamp);
}

// ==================== Radio Stage Settings ====================

void dwm3000_delayed_tx_then_rx(void) { write_fast_command(0x0F); }

void dwm3000_delayed_tx(void) { write_fast_command(0x3); }

void dwm3000_standard_tx(void) { write_fast_command(0x01); }

void dwm3000_standard_rx(void) { write_fast_command(0x02); }

void dwm3000_tx_instant_rx(void) { write_fast_command(0x0C); }

// ==================== DWM3000 Firmware Interaction ====================

void dwm3000_soft_reset(void) {
  clear_aon_config();
  dwm3000_write_reg_auto(PMSC_REG, 0x04, 0x1);
  dwm3000_write_reg(PMSC_REG, 0x00, 0x00, 2);
  delay_ms(100);
  dwm3000_write_reg_auto(PMSC_REG, 0x00, 0xFFFF);
  dwm3000_write_reg(PMSC_REG, 0x04, 0x00, 1);
}

void dwm3000_hard_reset(void) {
  gpio_set_direction(DWM3000_RST_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DWM3000_RST_PIN, 0);
  delay_ms(10);
  gpio_set_direction(DWM3000_RST_PIN, GPIO_MODE_INPUT);
}

void dwm3000_clear_system_status(void) {
  dwm3000_write_reg_auto(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF);
}

// ==================== Hardware Status Information ====================

void dwm3000_pull_led_high(int led) {
  if (led > 2)
    return;
  led_status = led_status | (1 << led);
  dwm3000_write_reg_auto(0x05, 0x0C, led_status);
}

void dwm3000_pull_led_low(int led) {
  if (led > 2)
    return;
  led_status = led_status & ~((int)1 << led);
  dwm3000_write_reg_auto(0x05, 0x0C, led_status);
}

// ==================== Calculation and Conversion ====================

double dwm3000_convert_to_cm(int dwm3000_ps_units) {
  return (double)dwm3000_ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

void dwm3000_calculate_tx_rx_diff(void) {
  unsigned long long ping_tx = dwm3000_read_tx_timestamp();
  unsigned long long ping_rx = dwm3000_read_rx_timestamp();
  long double clk_offset = dwm3000_get_clock_offset();
  long double clock_offset = 1.0 + clk_offset;
  long long t_reply = dwm3000_read_reg(RX_BUFFER_0_REG, 0x03);

  if (t_reply == 0) {
    return;
  }

  long long t_round = ping_rx - ping_tx;
  long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);
  long double t_prop_ps = t_prop * PS_UNIT;
  long double t_prop_cm = t_prop_ps * SPEED_OF_LIGHT;
  if (t_prop_cm >= 0) {
    dwm3000_print_double(t_prop_cm, 100, false);
    printf("cm\n");
  }
}

// ==================== Printing ====================

void dwm3000_print_round_trip_information(void) {
  printf("\nRound Trip Information:\n");
  long long tx_ts = dwm3000_read_tx_timestamp();
  long long rx_ts = dwm3000_read_rx_timestamp();
  printf("TX Timestamp: %lld\n", tx_ts);
  printf("RX Timestamp: %lld\n", rx_ts);
}

void dwm3000_print_double(double val, unsigned int precision, bool linebreak) {
  printf("%d.", (int)val);
  unsigned int frac;
  if (val >= 0) {
    frac = (val - (int)val) * precision;
  } else {
    frac = ((int)val - val) * precision;
  }
  if (linebreak) {
    printf("%u\n", frac);
  } else {
    printf("%u", frac);
  }
}

// ==================== Private Functions ====================

static void set_bit(int reg_addr, int sub_addr, int shift, bool b) {
  uint8_t tmpByte = dwm3000_read8bit(reg_addr, sub_addr);
  if (b) {
    tmpByte |= (1 << shift); // bitSet
  } else {
    tmpByte &= ~(1 << shift); // bitClear
  }
  dwm3000_write_reg_auto(reg_addr, sub_addr, tmpByte);
}

static void set_bit_low(int reg_addr, int sub_addr, int shift) {
  set_bit(reg_addr, sub_addr, shift, 0);
}

static void set_bit_high(int reg_addr, int sub_addr, int shift) {
  set_bit(reg_addr, sub_addr, shift, 1);
}

static void write_fast_command(int cmd) {
  if (DEBUG_OUTPUT)
    printf("[INFO] Executing short command: ");
  int header = 0;
  header = header | 0x1;
  header = header | (cmd & 0x1F) << 1;
  header = header | 0x80;
  if (DEBUG_OUTPUT)
    printf("0x%X\n", header);
  int header_arr[] = {header};
  send_bytes(header_arr, 1, 0);
}

static uint32_t read_or_write_full_address(uint32_t base, uint32_t sub,
                                           uint32_t data, uint32_t dataLen,
                                           uint32_t readWriteBit) {
  uint32_t header = 0x00;
  if (readWriteBit)
    header = header | 0x80;
  header = header | ((base & 0x1F) << 1);
  if (sub > 0) {
    header = header | 0x40;
    header = header << 8;
    header = header | ((sub & 0x7F) << 2);
  }
  uint32_t header_size = header > 0xFF ? 2 : 1;
  uint32_t res = 0;

  if (!readWriteBit) {
    int headerArr[2];
    if (header_size == 1) {
      headerArr[0] = header;
    } else {
      headerArr[0] = (header & 0xFF00) >> 8;
      headerArr[1] = header & 0xFF;
    }
    res = (uint32_t)send_bytes(headerArr, header_size, 4);
    return res;
  } else {
    uint32_t payload_bytes = 0;
    if (dataLen == 0) {
      if (data > 0) {
        uint32_t payload_bits = count_bits(data);
        payload_bytes = (payload_bits - (payload_bits % 8)) / 8;
        if ((payload_bits % 8) > 0) {
          payload_bytes++;
        }
      } else {
        payload_bytes = 1;
      }
    } else {
      payload_bytes = dataLen;
    }

    int payload[6]; // max header_size(2) + payload_bytes(4)
    if (header_size == 1) {
      payload[0] = header;
    } else {
      payload[0] = (header & 0xFF00) >> 8;
      payload[1] = header & 0xFF;
    }
    for (uint32_t i = 0; i < payload_bytes; i++) {
      payload[header_size + i] = (data >> i * 8) & 0xFF;
    }
    res = (uint32_t)send_bytes(payload, 2 + payload_bytes, 0);
    return res;
  }
}

static uint32_t send_bytes(int b[], int lenB, int recLen) {
  gpio_set_level(DWM3000_PIN_CS, 0);

  uint32_t val = 0;
  int total_len = lenB + (recLen > 0 ? recLen : 0);

  uint8_t *tx_buf = (uint8_t *)calloc(total_len, sizeof(uint8_t));
  uint8_t *rx_buf = (uint8_t *)calloc(total_len, sizeof(uint8_t));

  for (int i = 0; i < lenB; i++) {
    tx_buf[i] = (uint8_t)b[i];
  }
  // Remaining bytes (for reading) are already 0x00 from calloc

  spi_transaction_t t = {
      .length = total_len * 8,
      .tx_buffer = tx_buf,
      .rx_buffer = rx_buf,
  };

  spi_device_transmit(spi_handle, &t);

  if (recLen > 0) {
    for (int i = 0; i < recLen; i++) {
      uint32_t tmp = rx_buf[lenB + i];
      if (i == 0) {
        val = tmp;
      } else {
        val |= (uint32_t)tmp << (8 * i);
      }
    }
  }

  gpio_set_level(DWM3000_PIN_CS, 1);

  free(tx_buf);
  free(rx_buf);

  return val;
}

static void clear_aon_config(void) {
  dwm3000_write_reg(AON_REG, NO_OFFSET, 0x00, 2);
  dwm3000_write_reg(AON_REG, 0x14, 0x00, 1);
  dwm3000_write_reg(AON_REG, 0x04, 0x00, 1);
  dwm3000_write_reg_auto(AON_REG, 0x04, 0x02);
  delay_ms(1);
}

static unsigned int count_bits(unsigned int number) {
  return (int)log2(number) + 1;
}

static int check_for_dev_id(void) {
  int res = dwm3000_read_reg(GEN_CFG_AES_LOW_REG, NO_OFFSET);
  if (res != (int)0xDECA0302 && res != (int)0xDECA0312) {
    printf("[ERROR] DEV_ID IS WRONG!\n");
    return 0;
  }
  return 1;
}
