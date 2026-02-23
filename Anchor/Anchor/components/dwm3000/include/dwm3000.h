#ifndef DWM3000_H
#define DWM3000_H

#include <stdint.h>
#include <stdbool.h>

// ==================== Constants ====================

// SPI Pin Configuration (ESP32-S3)
#define DWM3000_PIN_CLK    10
#define DWM3000_PIN_MISO   11
#define DWM3000_PIN_MOSI   12
#define DWM3000_PIN_CS     13
#define DWM3000_RST_PIN    9

// UWB Configuration
#define LEN_RX_CAL_CONF       4
#define LEN_TX_FCTRL_CONF     6
#define LEN_AON_DIG_CFG_CONF  3
#define PMSC_STATE_IDLE        0x3
#define FCS_LEN                2

#define STDRD_SYS_CONFIG       0x188
#define DTUNE0_CONFIG          0x0F

#define SYS_STATUS_FRAME_RX_SUCC  0x2000
#define SYS_STATUS_RX_ERR         0x4279000
#define SYS_STATUS_FRAME_TX_SUCC  0x80

// Preamble Lengths
#define PREAMBLE_32    4
#define PREAMBLE_64    8
#define PREAMBLE_128   5
#define PREAMBLE_256   9
#define PREAMBLE_512   11
#define PREAMBLE_1024  2
#define PREAMBLE_2048  10
#define PREAMBLE_4096  3
#define PREAMBLE_1536  6

// Channels
#define CHANNEL_5  0x0
#define CHANNEL_9  0x1

// PAC Sizes
#define PAC4   0x03
#define PAC8   0x00
#define PAC16  0x01
#define PAC32  0x02

// Data Rates
#define DATARATE_6_8MB  0x1
#define DATARATE_850KB  0x0

// PHR Modes
#define PHR_MODE_STANDARD  0x0
#define PHR_MODE_LONG      0x1

// PHR Rates
#define PHR_RATE_6_8MB  0x1
#define PHR_RATE_850KB  0x0

// Masks
#define SPIRDY_MASK          0x80
#define RCINIT_MASK          0x100
#define BIAS_CTRL_BIAS_MASK  0x1F

// Register Addresses
#define GEN_CFG_AES_LOW_REG   0x00
#define GEN_CFG_AES_HIGH_REG  0x01
#define STS_CFG_REG           0x2
#define RX_TUNE_REG           0x3
#define EXT_SYNC_REG          0x4
#define GPIO_CTRL_REG         0x5
#define DRX_REG               0x6
#define RF_CONF_REG           0x7
#define RF_CAL_REG            0x8
#define FS_CTRL_REG           0x9
#define AON_REG               0xA
#define OTP_IF_REG            0xB
#define CIA_REG1              0xC
#define CIA_REG2              0xD
#define CIA_REG3              0xE
#define DIG_DIAG_REG          0xF
#define PMSC_REG              0x11
#define RX_BUFFER_0_REG       0x12
#define RX_BUFFER_1_REG       0x13
#define TX_BUFFER_REG         0x14
#define ACC_MEM_REG           0x15
#define SCRATCH_RAM_REG       0x16
#define AES_RAM_REG           0x17
#define SET_1_2_REG           0x18
#define INDIRECT_PTR_A_REG    0x1D
#define INDIRECT_PTR_B_REG    0x1E
#define IN_PTR_CFG_REG        0x1F

// Timing Constants
#define TRANSMIT_DELAY  0x3B9ACA00
#define TRANSMIT_DIFF   0x1FF

// Unit Conversions
#define NS_UNIT                         4.0064102564102564
#define PS_UNIT                         15.6500400641025641
#define SPEED_OF_LIGHT                  0.029979245800
#define CLOCK_OFFSET_CHAN_5_CONSTANT    (-0.5731e-3f)
#define CLOCK_OFFSET_CHAN_9_CONSTANT    (-0.1252e-3f)

// Offsets
#define NO_OFFSET  0x0

// Debug
#define DEBUG_OUTPUT  0

// ==================== Public API ====================

// Chip Setup
void dwm3000_spi_select(uint8_t cs);
void dwm3000_begin(void);
void dwm3000_init(void);
void dwm3000_write_sys_config(void);
void dwm3000_configure_as_tx(void);
void dwm3000_setup_gpio(void);

// Double-Sided Ranging
void dwm3000_ds_send_frame(int stage);
void dwm3000_ds_send_rt_info(int t_roundB, int t_replyB);
int  dwm3000_ds_process_rt_info(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
int  dwm3000_ds_get_stage(void);
bool dwm3000_ds_is_error_frame(void);
void dwm3000_ds_send_error_frame(void);

// Radio Settings
void dwm3000_set_channel(uint8_t data);
void dwm3000_set_preamble_length(uint8_t data);
void dwm3000_set_preamble_code(uint8_t data);
void dwm3000_set_pac_size(uint8_t data);
void dwm3000_set_datarate(uint8_t data);
void dwm3000_set_phr_mode(uint8_t data);
void dwm3000_set_phr_rate(uint8_t data);

// Protocol Settings
void dwm3000_set_mode(int mode);
void dwm3000_set_tx_frame(unsigned long long frame_data);
void dwm3000_set_frame_length(int frame_len);
void dwm3000_set_tx_antenna_delay(int delay);
void dwm3000_set_sender_id(int senderID);
void dwm3000_set_destination_id(int destID);

// Status Checks
int  dwm3000_received_frame_succ(void);
int  dwm3000_sent_frame_succ(void);
int  dwm3000_get_sender_id(void);
int  dwm3000_get_destination_id(void);
bool dwm3000_check_for_idle(void);
bool dwm3000_check_spi(void);

// Radio Analytics
double             dwm3000_get_signal_strength(void);
double             dwm3000_get_first_path_signal_strength(void);
int                dwm3000_get_tx_antenna_delay(void);
long double        dwm3000_get_clock_offset(void);
long double        dwm3000_get_clock_offset_ext(int32_t ext_clock_offset);
int                dwm3000_get_raw_clock_offset(void);
float              dwm3000_get_temp_in_c(void);
unsigned long long dwm3000_read_rx_timestamp(void);
unsigned long long dwm3000_read_tx_timestamp(void);

// Chip Interaction
uint32_t dwm3000_write_reg(int base, int sub, uint32_t data, int data_len);
uint32_t dwm3000_write_reg_auto(int base, int sub, uint32_t data);
uint32_t dwm3000_read_reg(int base, int sub);
uint8_t  dwm3000_read8bit(int base, int sub);
uint32_t dwm3000_read_otp(uint8_t addr);

// Delayed Sending Settings
void dwm3000_write_tx_delay(uint32_t delay);
void dwm3000_prepare_delayed_tx(void);

// Radio Stage Settings / Transfer and Receive Modes
void dwm3000_delayed_tx_then_rx(void);
void dwm3000_delayed_tx(void);
void dwm3000_standard_tx(void);
void dwm3000_standard_rx(void);
void dwm3000_tx_instant_rx(void);

// DWM3000 Firmware Interaction
void dwm3000_soft_reset(void);
void dwm3000_hard_reset(void);
void dwm3000_clear_system_status(void);

// Hardware Status Information
void dwm3000_pull_led_high(int led);
void dwm3000_pull_led_low(int led);

// Calculation and Conversion
double dwm3000_convert_to_cm(int dwm3000_ps_units);
void   dwm3000_calculate_tx_rx_diff(void);

// Printing
void dwm3000_print_round_trip_information(void);
void dwm3000_print_double(double val, unsigned int precision, bool linebreak);

#endif // DWM3000_H
