/**
 * @file      sx126x.c
 *
 * @brief     SX126x radio driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <string.h>  // memcpy
#include <Arduino.h>
#include "LoRa_Library.h"
#include "sx126x_hal.h"
#include "sx126x_regs.h"


/*
* -----------------------------------------------------------------------------
* --- PRIVATE CONSTANTS -------------------------------------------------------
*/

/**
* @brief Internal frequency of the radio
*/
#define SX126X_XTAL_FREQ 32000000UL

/**
* @brief Internal frequency of the radio
*/
#define SX126X_RTC_FREQ_IN_HZ 64000UL

/**
* @brief Scaling factor used to perform fixed-point operations
*/
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
* @brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
*/
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*!
* @brief Frequency step in MHz used to compute the image calibration parameter
*
* @see sx126x_cal_img_in_mhz
*/
#define SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ 4

#define SX126X_CHIP_MODES_POS ( 4U )
#define SX126X_CHIP_MODES_MASK ( 0x07UL << SX126X_CHIP_MODES_POS )

#define SX126X_CMD_STATUS_POS ( 1U )
#define SX126X_CMD_STATUS_MASK ( 0x07UL << SX126X_CMD_STATUS_POS )

#define SX126X_GFSK_RX_STATUS_PKT_SENT_POS ( 0U )
#define SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_SENT_POS )

#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS ( 1U )
#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS )

#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS ( 2U )
#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS ( 3U )
#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_CRC_ERROR_POS ( 4U )
#define SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_CRC_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS ( 5U )
#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS )

/*
* -----------------------------------------------------------------------------
* --- PRIVATE TYPES -----------------------------------------------------------
*/

/**
* @brief SX126X power amplifier configuration parameters structure definition
*/
typedef struct sx126x_pa_cfg_params_s
{
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
} sx126x_pa_cfg_params_t;


/**
* @brief SX126X power amplifier ramp-up timings enumeration definition
*/
typedef enum sx126x_ramp_time_e
{
    SX126X_RAMP_10_US   = 0x00,
    SX126X_RAMP_20_US   = 0x01,
    SX126X_RAMP_40_US   = 0x02,
    SX126X_RAMP_80_US   = 0x03,
    SX126X_RAMP_200_US  = 0x04,
    SX126X_RAMP_800_US  = 0x05,
    SX126X_RAMP_1700_US = 0x06,
    SX126X_RAMP_3400_US = 0x07,
} sx126x_ramp_time_t;

/**
* @brief SX126X GFSK modulation parameters structure definition
*/
typedef struct sx126x_mod_params_gfsk_s
{
    uint32_t                  br_in_bps; // 300000 - 600 bps
    uint32_t                  fdev_in_hz; // 200000 - 600 hz
    sx126x_gfsk_pulse_shape_t pulse_shape;
    sx126x_gfsk_bw_t          bw_dsb_param; // Victor Kalenda Modification
} sx126x_mod_params_gfsk_t;

/**
* @brief SX126X LoRa modulation parameters structure definition
*/
typedef struct sx126x_mod_params_lora_s
{
    sx126x_lora_sf_t sf;       //!< LoRa Spreading Factor
    sx126x_lora_bw_t bw;    //!< LoRa Bandwidth
    sx126x_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t          ldro;               //!< Low DataRate Optimization configuration
} sx126x_mod_params_lora_t;

/**
* @brief SX126X LoRa packet parameters structure definition
*/
typedef struct sx126x_pkt_params_lora_s
{
    uint16_t                    preamble_len_in_symb;                        //!< Preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type;           //!< Header type
    uint8_t                     pld_len_in_bytes;                             //!< Payload length in bytes
    bool                        crc_is_on;                                //!< CRC activation
    bool                        invert_iq_is_on;                          //!< IQ polarity setup
} sx126x_pkt_params_lora_t;

/**
* @brief SX126X GFSK packet parameters structure definition
*/
typedef struct sx126x_pkt_params_gfsk_s
{
    uint16_t                        preamble_len_in_bits;                                       //!< Preamble length in bits
    sx126x_gfsk_preamble_detector_t preamble_detector;          //!< Preamble detection length
    uint8_t                         sync_word_len_in_bits;                                      //!< Sync word length in bits
    sx126x_gfsk_address_filtering_t address_filtering;      //!< Address filtering configuration
    sx126x_gfsk_pkt_len_modes_t     header_type;                          //!< Header type
    uint8_t                         pld_len_in_bytes;                                           //!< Payload length in bytes
    sx126x_gfsk_crc_types_t         crc_type;                                 //!< CRC type configuration
    sx126x_gfsk_dc_free_t           dc_free;                              //!< Whitening configuration
} sx126x_pkt_params_gfsk_t;


/**
* Commands Interface
*/
typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
* Commands Interface buffer sizes
*/
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 2,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_GFSK_ADDRESS           = 2, // Victor Kalenda Addition
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 2,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;


typedef struct
{
    uint32_t bw;
    uint8_t  param;
} gfsk_bw_t;

gfsk_bw_t gfsk_bw[] = {
    { 4800, SX126X_GFSK_BW_4800 },     { 5800, SX126X_GFSK_BW_5800 },     { 7300, SX126X_GFSK_BW_7300 },
    { 9700, SX126X_GFSK_BW_9700 },     { 11700, SX126X_GFSK_BW_11700 },   { 14600, SX126X_GFSK_BW_14600 },
    { 19500, SX126X_GFSK_BW_19500 },   { 23400, SX126X_GFSK_BW_23400 },   { 29300, SX126X_GFSK_BW_29300 },
    { 39000, SX126X_GFSK_BW_39000 },   { 46900, SX126X_GFSK_BW_46900 },   { 58600, SX126X_GFSK_BW_58600 },
    { 78200, SX126X_GFSK_BW_78200 },   { 93800, SX126X_GFSK_BW_93800 },   { 117300, SX126X_GFSK_BW_117300 },
    { 156200, SX126X_GFSK_BW_156200 }, { 187200, SX126X_GFSK_BW_187200 }, { 234300, SX126X_GFSK_BW_234300 },
    { 312000, SX126X_GFSK_BW_312000 }, { 373600, SX126X_GFSK_BW_373600 }, { 467000, SX126X_GFSK_BW_467000 },
};











// Victor Kalenda Addition Start

/**
 * @brief sx126x bandwidth list
 * Used to enable Low Data Rate Optimization automatically for a given transmission
 */

typedef struct
{
  double bw;
  uint8_t param;
} lora_bw_t;

lora_bw_t lora_bw[] = {
  { 7, SX126X_LORA_BW_007 }, { 10, SX126X_LORA_BW_010 }, { 15, SX126X_LORA_BW_015 },
  { 20, SX126X_LORA_BW_020 }, { 31, SX126X_LORA_BW_031 }, { 41, SX126X_LORA_BW_041 },
  { 62, SX126X_LORA_BW_062 }, { 125, SX126X_LORA_BW_125 }, { 250, SX126X_LORA_BW_250 },
  { 500, SX126X_LORA_BW_500 },
};

/**
 * @brief sx126x Settings Structure
 * This structure was designed to let users have a wider range of variable naming options.
 */
typedef struct sx126x_s
{
  const void* reserved;
  // for identifying the type of chip used (sx1262, sx1261)
  chip_type_t chip;
  uint8_t tx_base;
  uint8_t rx_base;
  bool reset_params;
  bool auto_ldro;
  uint32_t crystal_frequency_error;
  uint8_t lora_sync_word;
  uint8_t gfsk_sync_word[8];

  sx126x_irq_mask_t IRQ;
  sx126x_irq_t interrupts;

  // real-time chip status
  sx126x_status_t api_status;
  sx126x_pkt_type_t packet_type;
  sx126x_chip_status_t chip_status;


  // RF Parameters
  uint32_t frequency;
  uint32_t transmit_timeout;
  uint32_t receive_timeout;

  // LoRa Parameters
  sx126x_mod_params_lora_t lora_modulation_params;
  sx126x_pkt_params_lora_t lora_packet_params;
  // Channel Activity Detection Parameters
  sx126x_cad_params_t lora_cad_params;

  // GFSK Parameters
  sx126x_mod_params_gfsk_t gfsk_modulation_params;
  sx126x_pkt_params_gfsk_t gfsk_packet_params;

  // Power
  sx126x_tx_power_t power;

}sx126x_t;

/*
* -----------------------------------------------------------------------------
* -----------------------------------------------------------------------------
* -----------------------------------------------------------------------------
* --- PRIVATE VARIABLES -------------------------------------------------------
*/

sx126x_t sx126x;

/*
* -----------------------------------------------------------------------------
* -----------------------------------------------------------------------------
* -----------------------------------------------------------------------------
* --- PRIVATE FUNCTIONS PROTOTYPES -------------------------------------------
*/

void set_lora();
void set_gfsk();
bool correct_bandwidth(uint32_t raw_br, uint32_t fdev);
void check_IRQ();
void check_if_reset_is_required_now();


// Victor Kalenda Addition End










//
// Buffer Access
//

/**
* @brief Write data into radio Tx buffer memory space.
*
* @param [in] context Chip implementation context
* @param [in] offset Start address in the Tx buffer of the chip
* @param [in] buffer The buffer of bytes to write into radio buffer
* @param [in] size The number of bytes to write into Tx radio buffer
*
* @returns Void
*
* @see sx126x_read_buffer
*/
void sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                    const uint8_t size );

/**
* @brief Read data from radio Rx buffer memory space.
*
* @param [in] context Chip implementation context
* @param [in] offset Start address in the Rx buffer of the chip
* @param [in] buffer The buffer of bytes to be filled with content from Rx radio buffer
* @param [in] size The number of bytes to read from the Rx radio buffer
*
* @returns Void
*
* @see sx126x_write_buffer
*/
void sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );

/**
* @brief Set buffer start addresses for both Tx and Rx operations
*
* @param [in] context Chip implementation context
* @param [in] tx_base_address The start address used for Tx operations
* @param [in] rx_base_address The start address used for Rx operations
*
* @returns Operational Status
*/
sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

//
// RF Modulation and Packet-Related Functions
//

/**
* @brief Set the chip in transmission mode
*
* @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
*
* @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
* has not been completely transmitted before the timeout. This behavior can be altered by @ref
* sx126x_set_rx_tx_fallback_mode.
*
* @remark The timeout duration can be computed with the formula:
* \f$ timeout\_duration\_ms = timeout_in_rtc_step \times * \frac{1}{64} \f$
*
* @remark Maximal value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms)
*
* @remark If the timeout argument is 0, then no timeout is used.
*
* @param [in] context Chip implementation context
* @param [in] timeout_in_rtc_step The timeout configuration for Tx operation
*
* @returns Operational Status
*/
sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
* @brief Set the chip in reception mode
*
* @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
*
* @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
* or if no packet has been received before the timeout. This behavior can be altered by @ref
* sx126x_set_rx_tx_fallback_mode.
*
* @remark The timeout duration is obtained by:
* \f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{64} \f$
*
* @remark Maximal timeout value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms).
*
* @remark The timeout argument can have the following special values:
*
* | Special values        | Meaning                                                                               |
* | ----------------------| --------------------------------------------------------------------------------------|
* | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
* | SX126X_RX_CONTINUOUS  | Continuous: the chip stays in RX mode even after reception of a packet                |
*
* @param [in] context Chip implementation context
* @param [in] timeout_in_rtc_step The timeout configuration for Rx operation
*
* @returns Operational Status
*/
sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
* @brief Set the RF frequency for future radio operations.
*
* @remark This commands shall be called only after a packet type is selected.
*
* @param [in] context Chip implementation context
* @param [in] freq_in_hz The frequency in Hz to set for radio operations
*
* @returns Void
*/
void sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
* @brief Set the packet type
*
* @param [in] context Chip implementation context
*
* @param [in] pkt_type Packet type to set
*
* @returns Void
*/
void sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type );

/**
* @brief Set the chip in reception mode
*
* @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
*
* @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
* or if no packet has been received before the timeout. This behavior can be altered by @ref
* sx126x_set_rx_tx_fallback_mode.
*
* @remark The timeout argument can have the following special values:
*
* | Special values        | Meaning                                                                               |
* | ----------------------| --------------------------------------------------------------------------------------|
* | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
*
* @param [in] context Chip implementation context
* @param [in] timeout_in_ms The timeout configuration in millisecond for Rx operation
*
* @returns Void
*/
void sx126x_set_rx( const void* context, const uint32_t timeout_in_ms );

/**
* @brief Set the chip in transmission mode
*
* @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
*
* @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
* has not been completely transmitted before the timeout. This behavior can be altered by @ref
* sx126x_set_rx_tx_fallback_mode.
*
* @remark If the timeout argument is 0, then no timeout is used.
*
* @param [in] context Chip implementation context
* @param [in] timeout_in_ms The timeout configuration in millisecond for Tx operation
*
* @returns Void
*/
void sx126x_set_tx( const void* context, const uint32_t timeout_in_ms );

/**
* @brief Configure the regulator mode to be used
*
* @remark This function shall be called to set the correct regulator mode, depending on the usage of LDO or DC/DC on
* the PCB implementation.
*
* @param [in] context Chip implementation context
* @param [in] mode Regulator mode configuration
*
* @returns Void
*/
void sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode );

/**
* @brief Launch an image calibration valid for all frequencies inside an interval, in steps
*
* @param [in] context Chip implementation context
* @param [in] freq1 Image calibration interval lower bound, in steps
* @param [in] freq2 Image calibration interval upper bound, in steps
*
* @remark freq1 must be less than or equal to freq2
*
* @returns Operational Status
*/
sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 );

/**
* @brief Set the chip in reception mode with duty cycling
*
* @remark The Rx mode duration is defined by:
* \f$ rx\_duration\_ms = rx_time \times \frac{1}{64} \f$
*
* @remark The sleep mode duration is defined by:
* \f$ sleep\_duration\_ms = sleep_time \times \frac{1}{64} \f$
*
* @remark Maximal timeout value is 0xFFFFFF (i.e. 511 seconds).
*
* @param [in] context Chip implementation context
* @param [in] rx_time The timeout of Rx period
* @param [in] sleep_time The length of sleep period
*
* @returns Operational Status
*/
sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                  const uint32_t rx_time_in_rtc_step,
                                                                  const uint32_t sleep_time_in_rtc_step );
/**
* @brief Configure the PA (Power Amplifier)
*
* @remark The parameters depend on the chip being used
*
* @param [in] context Chip implementation context
* @param [in] params Power amplifier configuration parameters
*
* @returns Void
*/
void sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params );

/**
* @brief Write data into register memory space.
*
* @param [in] context Chip implementation context
* @param [in] address The register memory address to start writing operation
* @param [in] buffer The buffer of bytes to write into memory
* @param [in] size Number of bytes to write into memory, starting from address
*
* @returns Operational Status
*
* @see sx126x_read_register
*/
sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                      const uint8_t size );

/**
* @brief Read data from register memory space.
*
* @param [in] context Chip implementation context
* @param [in] address The register memory address to start reading operation
* @param [in] buffer The buffer of bytes to be filled with data from registers
* @param [in] size Number of bytes to read from memory, starting from address
*
* @returns Operational Status
*
* @see sx126x_write_register
*/
sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint8_t size );

/**
* @brief Set the RF frequency for future radio operations - parameter in PLL steps
*
* @remark This commands shall be called only after a packet type is selected.
*
* @param [in] context Chip implementation context
* @param [in] freq The frequency in PLL steps to set for radio operations
*
* @returns Operational Status
*/
sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
* @brief Set the parameters for TX power and power amplifier ramp time
*
* @param [in] context Chip implementation context
* @param [in] pwr_in_dbm The Tx output power in dBm
* @param [in] ramp_time The ramping time configuration for the PA
*
* @returns Void
*/
void sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const sx126x_ramp_time_t ramp_time );

/**
* @brief Set the modulation parameters for GFSK packets
*
* @remark The command @ref sx126x_set_pkt_type must be called prior to this
* one.
*
* @param [in] context Chip implementation context
* @param [in] params The structure of GFSK modulation configuration
*
* @returns Void
*/
void sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params );

/**
* @brief Set the modulation parameters for LoRa packets
*
* @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
*
* @param [in] context Chip implementation context
* @param [in] params The structure of LoRa modulation configuration
*
* @returns Void
*/
void sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params );

/**
* @brief Set the packet parameters for GFSK packets
*
* @remark The command @ref sx126x_set_pkt_type must be called prior to this
* one.
*
* @param [in] context Chip implementation context
* @param [in] params The structure of GFSK packet configuration
*
* @returns Void
*/
void sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params );

/**
* @brief Set the packet parameters for LoRa packets
*
* @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
*
* @param [in] context Chip implementation context
* @param [in] params The structure of LoRa packet configuration
*
* @returns Void
*/
void sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params );

/**
* @brief 15.1.2 Workaround
*
* @remark Before any packet transmission, bit #2 of SX126X_REG_TX_MODULATION shall be set to:
* 0 if the LoRa BW = 500 kHz
* 1 for any other LoRa BW
* 1 for any (G)FSK configuration
*
* @param [in] context Chip implementation context.
* @param [in] pkt_type The modulation type (G)FSK/LoRa
* @param [in] bw In case of LoRa modulation the bandwith must be specified
*
* @returns Operational Status
*/
static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw );

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type );

/**
* @brief Compute the numerator for LoRa time-on-air computation.
*
* @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
*
* @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
* @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
*
* @returns LoRa time-on-air numerator
*/
uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p );

/**
* @brief Get the time on air in ms for LoRa transmission
*
* @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
* @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
*
* @returns Time-on-air value in ms for LoRa transmission
*/
uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p );


/**
* @brief Compute the numerator for GFSK time-on-air computation.
*
* @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
* second.
*
* @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
*
* @returns GFSK time-on-air numerator
*/
uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p );

/**
* @brief Get the time on air in ms for GFSK transmission
*
* @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
* @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
*
* @returns Time-on-air value in ms for GFSK transmission
*/
uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p );

/**
* @brief Get the number of PLL steps for a given frequency in Hertz
*
* @param [in] freq_in_hz Frequency in Hertz
*
* @returns Number of PLL steps
*/
uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

/**
* @brief Get the number of RTC steps for a given timeout in millisecond
*
* @param [in] timeout_in_ms Timeout in millisecond
*
* @returns Number of RTC steps
*/
uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms );

/**
* @brief Configure the Tx PA clamp
*
* @remark Workaround - With a SX1262, during the chip initialization, calling this function optimizes the PA clamping
* threshold. The call must be done after a Power On Reset or a wake-up from cold start (see DS_SX1261-2_V1.2 datasheet
* chapter 15.2)
*
* @param [in] context Chip implementation context
*
* @returns Void
*/
void sx126x_cfg_tx_clamp( const void* context );

/**
* @brief Stop the RTC and clear the related event
*
* @remark Workaround - It is advised to call this function after ANY reception with timeout active sequence, which
* stop the RTC and clear the timeout event, if any (see DS_SX1261-2_V1.2 datasheet chapter 15.4)
*
* @param [in] context Chip implementation context
*
* @returns Void
*/
void sx126x_stop_rtc( const void* context );

/**
* @brief Get the chip status
*
* @param [in] context Chip implementation context
* @param [out] radio_status Pointer to a structure holding the radio status
*
* @returns Void
*/
void sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status );












/*
 * -----------------------------------------------------------------------------
 * -----------------------------------------------------------------------------
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

// Victor Kalenda Addition Start

void initialize_chip(start_params_t setup)
{
  // Setup the default settings of the chip
  sx126x.tx_base = 0x00;
  sx126x.rx_base = 0x00;
  sx126x.reset_params = true;
  sx126x.auto_ldro = false;
  sx126x.lora_sync_word = 0x14;
  sx126x.frequency = 915000000;
  sx126x.transmit_timeout = SX126X_MAX_TIMEOUT_IN_MS;
  sx126x.receive_timeout = SX126X_RX_SINGLE_MODE;
  sx126x.lora_modulation_params = {SX126X_LORA_SF7, SX126X_LORA_BW_500, SX126X_LORA_CR_4_8, false}; 
  sx126x.lora_packet_params = {16, SX126X_LORA_PKT_EXPLICIT, 0, true, false};
  sx126x.gfsk_modulation_params = {600, 600, SX126X_GFSK_PULSE_SHAPE_OFF, SX126X_GFSK_BW_373600};
  sx126x.gfsk_packet_params = {32, SX126X_GFSK_PREAMBLE_DETECTOR_OFF, 64, SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
            SX126X_GFSK_PKT_FIX_LEN, 0, SX126X_GFSK_CRC_OFF, SX126X_GFSK_DC_FREE_OFF};

  // Begin communicating to the chip and defining critical parameters
  sx126x.chip = setup.chip;
  sx126x.crystal_frequency_error = setup.crystal_frequency_error;

  if(setup.dio1 != -1)
  {
    attachInterrupt(digitalPinToInterrupt(setup.dio1), check_IRQ, RISING);
  }

  sx126x_hal_init(setup.NSS, setup.reset, setup.busy, setup.dio1);
  
  // POR Sequence
  sx126x_hal_reset(sx126x.reserved);
  sx126x_wakeup(sx126x.reserved);

  // Refer to section 5.1.3, must be executed in STDBY_RC mode (default at startup)
  sx126x_set_reg_mode(sx126x.reserved, setup.regulator);

  // Set the buffer base address for tx and rx, default is 0x00 for both. If rx is large enough it will overflow tx if tx base address is after rx base address
  sx126x_set_buffer_base_address(sx126x.reserved, sx126x.tx_base, sx126x.rx_base);
  
  // Refer to section 13.3.6, only us if TCXO oscillator is used, timeout value dependent on PCB efficiency
  if(setup.crystal == TCXO)
  {
    sx126x_set_dio3_as_tcxo_ctrl(sx126x.reserved, SX126X_TCXO_CTRL_1_7V, (uint32_t) 320);
  }

  // Refer to section 9.2
  sx126x_cal(sx126x.reserved, SX126X_CAL_ALL);
  delayMicroseconds(10000);

  // Refer to section 9.6
  sx126x_cfg_rx_boosted(sx126x.reserved, true);

  // Refer to section 15.2
  sx126x_cfg_tx_clamp(sx126x.reserved);

  // Set standby, Refer to 13.3.6 
  sx126x_set_rx_tx_fallback_mode(sx126x.reserved, SX126X_FALLBACK_STDBY_XOSC);
  sx126x_set_standby(sx126x.reserved, SX126X_STANDBY_CFG_XOSC);

  // Refer to section 13.1.14, Pa defaulted to 22 dbm for sx1262 and 14dbm for sx1261  
  // Refer to section 5.1, OCP defaulted to 140mA for sx1262 and 60mA for sx1261
  set_power(SX126X_TX_14DBM);
  
  if(setup.dio1 != -1)
  {
    // Refer to section 13.3.2
    sx126x_set_dio_irq_params(sx126x.reserved, SX126X_IRQ_ALL, SX126X_IRQ_ALL, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    // Refer to section 13.3.5
    sx126x_set_dio2_as_rf_sw_ctrl(sx126x.reserved, true);
  }

  // Refer to section 13.1.5, this is a default setting used for battery conscious applications
  // Refer to section 13.4.9, 0x00 will deactivate the timeout to locking the LoRa modem (making this more susceptible to false detections)
  sx126x_set_lora_symb_nb_timeout(sx126x.reserved, 0);

  // Refer to section 12.1, this will identify your transmissions as part of a private network
  sx126x_set_lora_sync_word(sx126x.reserved, sx126x.lora_sync_word);

  uint8_t sync_word[8] = {1,2,3,4,5,6,7,8};
  memcpy(sx126x.gfsk_sync_word, sync_word, sx126x.gfsk_packet_params.sync_word_len_in_bits / 8);
  sx126x_set_gfsk_sync_word(sx126x.reserved, sx126x.gfsk_sync_word, 8);
}

void lora_crc_on(bool crc_is_on)
{
  if(sx126x.lora_packet_params.crc_is_on != crc_is_on)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_packet_params.crc_is_on = crc_is_on;  
    check_if_reset_is_required_now();
  }
}

// Packet parameter setters lora
void lora_invert_iq(bool invert_iq_is_on)
{
  if(sx126x.lora_packet_params.invert_iq_is_on != invert_iq_is_on)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_packet_params.invert_iq_is_on = invert_iq_is_on;
    check_if_reset_is_required_now();
  }
}

void lora_set_header_type(sx126x_lora_pkt_len_modes_t header_type)
{
  if(sx126x.lora_packet_params.header_type != header_type)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_packet_params.header_type = header_type;
    check_if_reset_is_required_now();
  }
}

void lora_set_pld_len(uint8_t payload_length_in_bytes)
{
  if(sx126x.lora_packet_params.pld_len_in_bytes != payload_length_in_bytes)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_packet_params.pld_len_in_bytes = payload_length_in_bytes;
    check_if_reset_is_required_now();
  }
}

bool lora_set_pre_len(uint16_t preamble_length_in_symbols)
{
  // Refer to section 6.1.3
  if(preamble_length_in_symbols >= 10 && preamble_length_in_symbols <= 65535)
  {
    if(sx126x.lora_packet_params.preamble_len_in_symb != preamble_length_in_symbols)
    {
      sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
      sx126x.lora_packet_params.preamble_len_in_symb = preamble_length_in_symbols;
      check_if_reset_is_required_now();
    }
    return true;
  }
  return false;
}

// Modulation parameter setters lora
void lora_set_sf(sx126x_lora_sf_t spreadfactor)
{
  // Refer to section 6.1.1.1
  if(sx126x.lora_modulation_params.sf != spreadfactor)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_modulation_params.sf = spreadfactor;
    if((spreadfactor == SX126X_LORA_SF5 || spreadfactor == SX126X_LORA_SF6) && sx126x.lora_packet_params.preamble_len_in_symb < 12)
    {
      sx126x.lora_packet_params.preamble_len_in_symb = 12;
    }
    check_if_reset_is_required_now();
  }
}

void lora_set_bw(sx126x_lora_bw_t bandwidth)
{
  if(sx126x.lora_modulation_params.bw != bandwidth)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_modulation_params.bw = bandwidth;
    check_if_reset_is_required_now();
  }
}

void lora_set_cr(sx126x_lora_cr_t coderate)
{
  if(sx126x.lora_modulation_params.cr != coderate)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_modulation_params.cr = coderate;
    check_if_reset_is_required_now();
  }
}

void lora_set_ldro(bool ldro_is_on)
{
  if(sx126x.lora_modulation_params.ldro != ldro_is_on)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_LORA);
    sx126x.lora_modulation_params.ldro = ldro_is_on;
    check_if_reset_is_required_now();
  }
}

void lora_set_auto_ldro(bool auto_ldro_is_on)
{
  sx126x.auto_ldro = auto_ldro_is_on;
}



// Packet parameter setters gfsk (sync word length in bits handled in sx126x_set_gfsk_sync_word())

bool gfsk_set_pre_len(uint16_t preamble_len_in_bits)
{
  // Refer to section 6.2.3.1
  if(preamble_len_in_bits >= 8 && preamble_len_in_bits <= 65535)
  {
    if(sx126x.gfsk_packet_params.preamble_len_in_bits != preamble_len_in_bits)
    {
      sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
      sx126x.gfsk_packet_params.preamble_len_in_bits = preamble_len_in_bits;
      check_if_reset_is_required_now();
    }
    return true;
  }
  return false;
}

void gfsk_set_pre_detector(sx126x_gfsk_preamble_detector_t preamble_detection_length)
{
  if(sx126x.gfsk_packet_params.preamble_detector != preamble_detection_length)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.preamble_detector = preamble_detection_length;
    check_if_reset_is_required_now();
  }
}

void gfsk_addr_filter(sx126x_gfsk_address_filtering_t address_filtering)
{
  if(sx126x.gfsk_packet_params.address_filtering != address_filtering)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.address_filtering = address_filtering;
    check_if_reset_is_required_now();
  }
}

void gfsk_set_header_type(sx126x_gfsk_pkt_len_modes_t header_type)
{
  if(sx126x.gfsk_packet_params.header_type != header_type)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.header_type = header_type;
    check_if_reset_is_required_now();
  }
}

bool gfsk_set_pld_len(uint8_t pld_len_in_bytes)
{
  if(sx126x.gfsk_packet_params.pld_len_in_bytes != pld_len_in_bytes)
  {
    // Refer to section 6.2.3.1 and 6.2.3.2
    if(sx126x.gfsk_packet_params.address_filtering != SX126X_GFSK_ADDRESS_FILTERING_DISABLE && pld_len_in_bytes < 254)
    {
      sx126x.gfsk_packet_params.pld_len_in_bytes = 254;
      return false;
    }
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.pld_len_in_bytes = pld_len_in_bytes;
    check_if_reset_is_required_now();
  }
  return true;
}

void gfsk_crc(sx126x_gfsk_crc_types_t crc_type)
{
  if(sx126x.gfsk_packet_params.crc_type != crc_type)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.crc_type = crc_type;
    check_if_reset_is_required_now();
  }
}

void gfsk_whitening_on(sx126x_gfsk_dc_free_t dc_free)
{
  if(sx126x.gfsk_packet_params.dc_free != dc_free)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_packet_params.dc_free = dc_free;
    check_if_reset_is_required_now();
  }
}

// Modulation parameter setters gfsk

bool gfsk_set_br(uint32_t br_in_bps) // between 600 - 300000 bps
{
  // Refer to section 6.2.1
  if(br_in_bps > 600 && br_in_bps < 300000)
  {
    // get the bitrate and frequency deviation the chip is set to
    const uint32_t raw_br = ( uint32_t )( 32 * SX126X_XTAL_FREQ / br_in_bps);
    const uint32_t fdev = sx126x_convert_freq_in_hz_to_pll_step(sx126x.gfsk_modulation_params.fdev_in_hz);
    // find the current bandwidth the chip is set to
    uint32_t bandwidth = 0;
    for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
    {
        if( sx126x.gfsk_modulation_params.bw_dsb_param == gfsk_bw[i].param )
        {
            bandwidth = gfsk_bw[i].bw;
            break;
        }
    }    
    if((raw_br + (2 * fdev) + (2 * sx126x.crystal_frequency_error)) <= bandwidth)
    {
      sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
      sx126x.gfsk_modulation_params.br_in_bps = br_in_bps;
      check_if_reset_is_required_now();
      return true;
    }
    else
    {
      Serial.println(F("Bandwidth Incompatible with BitRate"));
      if(correct_bandwidth(raw_br, fdev))
      {
        Serial.println(F("Bandwidth Corrected"));
        sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
        sx126x.gfsk_modulation_params.br_in_bps = br_in_bps;
        check_if_reset_is_required_now();
        return true;
      }
    }
  }
  return false;
}

bool gfsk_set_fdev(uint32_t fdev_in_hz) // between 600 - 200000 hz
{
  // Refer to section 6.2.1
  if(fdev_in_hz > 600 && fdev_in_hz < 200000)
  {
    // get the current bitrate and frequency deviation the chip is set to
    const uint32_t raw_br = ( uint32_t )( 32 * SX126X_XTAL_FREQ / sx126x.gfsk_modulation_params.br_in_bps);
    const uint32_t fdev = sx126x_convert_freq_in_hz_to_pll_step(fdev_in_hz);
    // find the current bandwidth the chip is set to
    uint32_t bandwidth = 0;
    for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
    {
        if( sx126x.gfsk_modulation_params.bw_dsb_param == gfsk_bw[i].param )
        {
            bandwidth = gfsk_bw[i].bw;
            break;
        }
    } 
    if((raw_br + (2 * fdev) + (2 * sx126x.crystal_frequency_error)) <= bandwidth)
    {
      sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
      sx126x.gfsk_modulation_params.fdev_in_hz = fdev_in_hz;
      check_if_reset_is_required_now();
      return true;
    }
    else
    {
      Serial.println(F("Bandwidth Incompatible with Frequency Deviation"));
      if(correct_bandwidth(raw_br, fdev))
      {
        Serial.println(F("Bandwidth Corrected"));
        sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
        sx126x.gfsk_modulation_params.fdev_in_hz = fdev_in_hz;
        check_if_reset_is_required_now();
        return true;
      }
    }
  }
  return false;
}

void gfsk_set_ps(sx126x_gfsk_pulse_shape_t pulse_shape)
{
  if(sx126x.gfsk_modulation_params.pulse_shape != pulse_shape)
  {
    sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
    sx126x.gfsk_modulation_params.pulse_shape = pulse_shape;
    check_if_reset_is_required_now();
  }
}

bool gfsk_set_bw(sx126x_gfsk_bw_t bw_dsb_param)
{
  // Refer to section 6.2.1
  if(sx126x.gfsk_modulation_params.bw_dsb_param != bw_dsb_param)
  {
    // get the current bitrate and frequency deviation the chip is set to
    const uint32_t raw_br = ( uint32_t )( 32 * SX126X_XTAL_FREQ / sx126x.gfsk_modulation_params.br_in_bps);
    const uint32_t fdev = sx126x_convert_freq_in_hz_to_pll_step(sx126x.gfsk_modulation_params.fdev_in_hz);
    // find the current bandwidth the chip is set to
    uint32_t bandwidth = 0;
    for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
    {
        if( bw_dsb_param == gfsk_bw[i].param )
        {
            bandwidth = gfsk_bw[i].bw;
            break;
        }
    } 
    if((raw_br + (2 * fdev) + (2 * sx126x.crystal_frequency_error)) <= bandwidth)
    {
      sx126x.reset_params = (sx126x.packet_type == SX126X_PKT_TYPE_GFSK);
      sx126x.gfsk_modulation_params.bw_dsb_param = bw_dsb_param;
      check_if_reset_is_required_now();
      return true;
    }
    else
    {
      Serial.println(F("Incompatible Bandwidth"));
      return false;
    }
  }
  return true;
}

// Modulation type setters
void set_packet_type(sx126x_pkt_type_t type)
{
  sx126x.packet_type = type;
  switch(type)
  {
    case SX126X_PKT_TYPE_GFSK:
    {
      set_gfsk();
      break;
    }
    case SX126X_PKT_TYPE_LORA:
    {
      set_lora();
      break;
    }
    default:
    {
      Serial.println(F("Invalid Type"));
    }
  }
}

// Transmit and Receive functions
void transmit(uint8_t *buffer)
{
  // write what you want to transmit to the chip

  if(sx126x.packet_type == SX126X_PKT_TYPE_LORA)
  {
    sx126x_write_buffer(sx126x.reserved, sx126x.tx_base, buffer, sx126x.lora_packet_params.pld_len_in_bytes);
  }
  else
  {
    sx126x_write_buffer(sx126x.reserved, sx126x.tx_base, buffer, sx126x.gfsk_packet_params.pld_len_in_bytes);
  }

  // Refer to section 13.4.1, Refer to 5.1 for why it must be configured before every reception and transmission
  sx126x_set_rf_freq(sx126x.reserved, sx126x.frequency);

  // run set_packet after any change in packet or modulation params
  if(sx126x.reset_params)
  {
    set_packet_type(sx126x.packet_type);
    sx126x.reset_params = false;
  }

  sx126x_set_tx(sx126x.reserved, sx126x.transmit_timeout);
}

void receive_mode()
{
  // Refer to section 13.4.1, Refer to 5.1 for why it must be configured before every reception and transmission
  sx126x_set_rf_freq(sx126x.reserved, sx126x.frequency);

  // run set_packet after any change in packet or modulation params
  if(sx126x.reset_params)
  {
    set_packet_type(sx126x.packet_type);
    sx126x.reset_params = false;
  }

  sx126x_set_rx(sx126x.reserved, sx126x.receive_timeout);
}

void receive_mode_duty_cycle(uint32_t rx_time, uint32_t sleep_time)
{
  // Refer to section 13.4.1, Refer to 5.1 for why it must be configured before every reception and transmission
  sx126x_set_rf_freq(sx126x.reserved, sx126x.frequency);

  // run set_packet after any change in packet or modulation params
  if(sx126x.reset_params)
  {
    set_packet_type(sx126x.packet_type);
    sx126x.reset_params = false;
  }

  sx126x_set_rx_duty_cycle(sx126x.reserved, rx_time, sleep_time);
}

void read_transmit_buffer(uint8_t* buffer)
{
  if(sx126x.packet_type == SX126X_PKT_TYPE_LORA)
  {
    sx126x_read_buffer( sx126x.reserved, sx126x.tx_base, buffer, sx126x.lora_packet_params.pld_len_in_bytes);
  }
  else
  {
    sx126x_read_buffer( sx126x.reserved, sx126x.tx_base, buffer, sx126x.gfsk_packet_params.pld_len_in_bytes);
  }
}

void set_transmit_timeout(uint32_t timeout_in_ms) // 262 142 ms is the max timeout used the for transmit function
{
  if(timeout_in_ms < SX126X_MAX_TIMEOUT_IN_MS)
  {
    sx126x.transmit_timeout = timeout_in_ms;
  }
  else
  {
    Serial.println(F("Invalid timeout"));
    sx126x.transmit_timeout = 0;
  }
}

void set_transmit_buffer_address(uint8_t address)
{
  sx126x.tx_base = address;
  sx126x.api_status = sx126x_set_buffer_base_address(sx126x.reserved, sx126x.tx_base, sx126x.rx_base);
}

void read_receive_buffer(uint8_t* buffer)
{
  if(sx126x.packet_type == SX126X_PKT_TYPE_LORA)
  {
    sx126x_read_buffer(sx126x.reserved, sx126x.rx_base, buffer, sx126x.lora_packet_params.pld_len_in_bytes);
  }
  else
  {
    sx126x_read_buffer(sx126x.reserved, sx126x.rx_base, buffer, sx126x.gfsk_packet_params.pld_len_in_bytes);
  }
}

void set_receive_timeout(uint32_t timeout_in_ms) // 262 142 ms is the max timeout used for the receive function
{
  if(timeout_in_ms < SX126X_MAX_TIMEOUT_IN_MS || timeout_in_ms == SX126X_RX_CONTINUOUS)
  {
    sx126x.receive_timeout = timeout_in_ms;
  }
  else
  {
    Serial.println(F("Invalid timeout"));
    sx126x.receive_timeout = 0;
  }
}

void set_receive_buffer_address(uint8_t address)
{
  sx126x.rx_base = address;
  sx126x.api_status = sx126x_set_buffer_base_address(sx126x.reserved, sx126x.tx_base, sx126x.rx_base);
}

// Miscellaneous Parameter setters

// If the OCP value is important to you, you must call sx126x_set_ocp_value() and reset it to your desired value
void set_power(sx126x_tx_power_t power_in_dbm) // used in init function for default 14dbm. Pa_config handled according to datasheet
{
  // Refer to section 13.1.14
  if(sx126x.power != power_in_dbm)
  {      
    sx126x_pa_cfg_params_t pa_config;
    sx126x.power = power_in_dbm;
    if(sx126x.chip == SX1262)
    {
      switch(sx126x.power)
      {
        case SX126X_TX_14DBM:
        {
          pa_config.pa_duty_cycle = 0x02; // Never set higher than 0x04
          pa_config.hp_max = 0x02; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 140mA by default, Refer to section 5.1
          pa_config.device_sel = 0x00; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 22, SX126X_RAMP_200_US);
          break;
        }
        case SX126X_TX_17DBM:
        {
          pa_config.pa_duty_cycle = 0x02; // Never set higher than 0x04
          pa_config.hp_max = 0x03; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 140mA by default, Refer to section 5.1
          pa_config.device_sel = 0x00; // SX1262 = 0x00, SX1261 = 0x01 
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 22, SX126X_RAMP_200_US);
          break;
        }
        case SX126X_TX_20DBM:
        {
          pa_config.pa_duty_cycle = 0x03; // Never set higher than 0x04
          pa_config.hp_max = 0x05; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 140mA by default, Refer to section 5.1
          pa_config.device_sel = 0x00; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 22, SX126X_RAMP_200_US);
          break;
        }
        case SX126X_TX_22DBM:
        {
          pa_config.pa_duty_cycle = 0x04; // Never set higher than 0x04
          pa_config.hp_max = 0x07; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 140mA by default, Refer to section 5.1
          pa_config.device_sel = 0x00; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 22, SX126X_RAMP_200_US);
          break;
        }
        default:
        {
          Serial.println(F("Invalid Power"));
        }
      }
    }
    else
    {
      switch(sx126x.power)
      {
        case SX126X_TX_10DBM:
        {
          pa_config.pa_duty_cycle = 0x01; // Never set higher than 0x04
          pa_config.hp_max = 0x00; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 60mA by default, Refer to section 5.1
          pa_config.device_sel = 0x01; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 13, SX126X_RAMP_200_US);
          break;
        }
        case SX126X_TX_14DBM:
        {
          pa_config.pa_duty_cycle = 0x04; // Never set higher than 0x04
          pa_config.hp_max = 0x00; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 60mA by default, Refer to section 5.1
          pa_config.device_sel = 0x01; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4
          sx126x_set_tx_params(sx126x.reserved, 14, SX126X_RAMP_200_US);
          break;
        }
        case SX126X_TX_15DBM:
        {
          // Refer to section 13.1.14.1
          if(sx126x.frequency < 400000000)
          {
            Serial.println(F("Power cannot be set above 14dbm when frequency synthesis is less than 400 Mhz "));
            break;
          }
          pa_config.pa_duty_cycle = 0x06; // Never set higher than 0x04
          pa_config.hp_max = 0x00; // 0x00 - 0x07, Higher settings will accelerate chip aging

          // OCP set to 60mA by default, Refer to section 5.1
          pa_config.device_sel = 0x01; // SX1262 = 0x00, SX1261 = 0x01
          pa_config.pa_lut = 0x01; // sx126x.reserved
          sx126x_set_pa_cfg(sx126x.reserved, &pa_config);
          // Refer to section 13.4.4  
          sx126x_set_tx_params(sx126x.reserved, 14, SX126X_RAMP_200_US);
          break;
        }
        default:
        {
          Serial.println(F("Invalid Power"));
        }
      }
    }
  }
}

void set_frequency(uint32_t frequency_in_hz)
{
  sx126x.frequency = frequency_in_hz;
  sx126x_set_rf_freq(sx126x.reserved, sx126x.frequency);
}

void gfsk_set_node_address(const void* context, const uint8_t address)
{
  sx126x.api_status = ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_NODE_ADDRESS, &address, 1 );
}

void gfsk_set_broadcast_address(const void* context, const uint8_t address)
{
  sx126x.api_status = ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_BROADCAST_ADDRESS, &address, 1 );
}

// Refer to AN1200.48 application note. These settings only apply for bandwidths 500, 125 khz, and spreadfactors 7 to 12
void set_cad_params()
{
  switch(sx126x.lora_modulation_params.bw)
  {
    case SX126X_LORA_BW_500:
    {          
      sx126x.lora_cad_params.cad_detect_min = 10;
      switch(sx126x.lora_modulation_params.sf)
      {
        case SX126X_LORA_SF7:
        {
          sx126x.lora_cad_params.cad_detect_peak = 21;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF8:
        {
          sx126x.lora_cad_params.cad_detect_peak = 22;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF9:
        {
          sx126x.lora_cad_params.cad_detect_peak = 22;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF10:
        {
          sx126x.lora_cad_params.cad_detect_peak = 23;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF11:
        {
          sx126x.lora_cad_params.cad_detect_peak = 25;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF12:
        {
          sx126x.lora_cad_params.cad_detect_peak = 29;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_08_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        default:
        {
            Serial.println(F("Spreadfactor not supported"));
        }
      }
      break;
    }
    case SX126X_LORA_BW_125:
    {
      sx126x.lora_cad_params.cad_detect_min = 10; 
      switch(sx126x.lora_modulation_params.sf)
      {
        case SX126X_LORA_SF7:
        {
          sx126x.lora_cad_params.cad_detect_peak = 22;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_02_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF8:
        {
          sx126x.lora_cad_params.cad_detect_peak = 22;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_02_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF9:
        {
          sx126x.lora_cad_params.cad_detect_peak = 23;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF10:
        {
          sx126x.lora_cad_params.cad_detect_peak = 24;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF11:
        {
          sx126x.lora_cad_params.cad_detect_peak = 25;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        case SX126X_LORA_SF12:
        {
          sx126x.lora_cad_params.cad_detect_peak = 28;
          sx126x.lora_cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
          sx126x_set_cad_params(sx126x.reserved, &sx126x.lora_cad_params);
          break;
        }
        default:
        {
            Serial.println(F("Spreadfactor not supported"));
        }
      }
      break;
    }
    default:
    {
      Serial.println(F("Bandwidth not supported"));
    }
  }
} 



// Miscellaneous Parameter getters

sx126x_irq_t get_interrupts() // for users to know what interrupts have been triggered
{
  return sx126x.interrupts;
}

void clear_interrupts()
{
  sx126x.interrupts.tx_done = false;
  sx126x.interrupts.rx_done = false;
  sx126x.interrupts.preamble_detected = false;
  sx126x.interrupts.sync_word_valid = false;
  sx126x.interrupts.header_valid = false;
  sx126x.interrupts.header_error = false;
  sx126x.interrupts.crc_error = false;
  sx126x.interrupts.cad_done = false;
  sx126x.interrupts.cad_detected = false;
  sx126x.interrupts.timeout = false;
  sx126x.interrupts.lr_fhss_hop = false;
}

uint32_t get_time_on_air()
{
  switch(sx126x.packet_type)
  {
    case SX126X_PKT_TYPE_GFSK:
    {
      return sx126x_get_gfsk_time_on_air_in_ms(&sx126x.gfsk_packet_params, &sx126x.gfsk_modulation_params);
    }
    case SX126X_PKT_TYPE_LORA:
    {
      return sx126x_get_lora_time_on_air_in_ms(&sx126x.lora_packet_params, &sx126x.lora_modulation_params);
    }
  }
  return 0;
}

sx126x_status_t get_api_status()
{
  return sx126x.api_status;
}

sx126x_chip_status_t get_chip_status()
{
  sx126x_get_status(sx126x.reserved, &sx126x.chip_status);
  return sx126x.chip_status;
}

uint8_t get_payload_length()
{
  if(sx126x.packet_type == SX126X_PKT_TYPE_LORA)
  {
    return sx126x.lora_packet_params.pld_len_in_bytes;
  }
  else
  {
    return sx126x.gfsk_packet_params.pld_len_in_bytes;
  }
}

uint16_t get_preamble_length()
{
  if(sx126x.packet_type == SX126X_PKT_TYPE_LORA)
  {
    return sx126x.lora_packet_params.preamble_len_in_symb;
  }
  else
  {
    return sx126x.gfsk_packet_params.preamble_len_in_bits;
  }
}

sx126x_pkt_type_t get_packet_type()
{
  return sx126x.packet_type;
}

uint8_t get_lora_sync_word()
{
  return sx126x.lora_sync_word;
}

void get_gfsk_sync_word(uint8_t* sync_word)
{
  memcpy(sync_word, sx126x.gfsk_sync_word, sx126x.gfsk_packet_params.sync_word_len_in_bits / 8);
}

// Victor Kalenda Addition End










void sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_SLEEP] = {
        SX126X_SET_SLEEP,
        ( uint8_t ) cfg,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_SLEEP, 0, 0 );
}

void sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_STANDBY] = {
        SX126X_SET_STANDBY,
        ( uint8_t ) cfg,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STANDBY, 0, 0 );
}

void sx126x_set_fs( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_FS] = {
        SX126X_SET_FS,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_FS, 0, 0 );
}

void sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = {
        SX126X_SET_STOP_TIMER_ON_PREAMBLE,
        ( enable == true ) ? 1 : 0,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

void sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms, const uint32_t sleep_time_in_ms )
{
    const uint32_t rx_time_in_rtc_step    = sx126x_convert_timeout_in_ms_to_rtc_step( rx_time_in_ms );
    const uint32_t sleep_time_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( sleep_time_in_ms );

    sx126x.api_status = sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_time_in_rtc_step, sleep_time_in_rtc_step );
}

void sx126x_set_cad( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD] = {
        SX126X_SET_CAD,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD, 0, 0 );
}

void sx126x_set_tx_cw( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_CONTINUOUS_WAVE] = {
        SX126X_SET_TX_CONTINUOUS_WAVE,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

void sx126x_set_tx_infinite_preamble( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_INFINITE_PREAMBLE] = {
        SX126X_SET_TX_INFINITE_PREAMBLE,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

void sx126x_cal( const void* context, const sx126x_cal_mask_t param )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE] = {
        SX126X_CALIBRATE,
        ( uint8_t ) param,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE, 0, 0 );
}

void sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    // Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
    const uint8_t freq1 = freq1_in_mhz / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    // Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
    const uint8_t freq2 =
        ( freq2_in_mhz + SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    sx126x.api_status = sx126x_cal_img( context, freq1, freq2 );
}

void sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_TX_FALLBACK_MODE] = {
        SX126X_SET_RX_TX_FALLBACK_MODE,
        ( uint8_t ) fallback_mode,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_TX_FALLBACK_MODE, 0, 0 );
}

//
// DIO and IRQ Control Functions
//

void sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask, const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO_IRQ_PARAMS] = {
        SX126X_SET_DIO_IRQ_PARAMS,     ( uint8_t )( irq_mask >> 8 ),  ( uint8_t )( irq_mask >> 0 ),
        ( uint8_t )( dio1_mask >> 8 ), ( uint8_t )( dio1_mask >> 0 ), ( uint8_t )( dio2_mask >> 8 ),
        ( uint8_t )( dio2_mask >> 0 ), ( uint8_t )( dio3_mask >> 8 ), ( uint8_t )( dio3_mask >> 0 ),
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

void sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    const uint8_t buf[SX126X_SIZE_GET_IRQ_STATUS] = {
        SX126X_GET_IRQ_STATUS,
        SX126X_NOP,
    };
    uint8_t         irq_local[sizeof( sx126x_irq_mask_t )] = { 0x00 };
    sx126x_status_t status                                 = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_IRQ_STATUS, irq_local,
                                                  sizeof( sx126x_irq_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *irq = ( ( sx126x_irq_mask_t ) irq_local[0] << 8 ) + ( ( sx126x_irq_mask_t ) irq_local[1] << 0 );
    }

    sx126x.api_status = status;
}

void sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask )
{
    const uint8_t buf[SX126X_SIZE_CLR_IRQ_STATUS] = {
        SX126X_CLR_IRQ_STATUS,
        ( uint8_t )( irq_mask >> 8 ),
        ( uint8_t )( irq_mask >> 0 ),
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

void sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    sx126x_get_irq_status( context, &sx126x_irq_mask );

    if( ( sx126x.api_status == SX126X_STATUS_OK ) && ( sx126x_irq_mask != 0 ) )
    {
        sx126x_clear_irq_status( context, sx126x_irq_mask );
    }
    if( ( sx126x.api_status == SX126X_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = sx126x_irq_mask;
    }
}

void sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL] = {
        SX126X_SET_DIO2_AS_RF_SWITCH_CTRL,
        ( enable == true ) ? 1 : 0,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL, 0, 0 );
}

void sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage, const uint32_t timeout )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL] = {
        SX126X_SET_DIO3_AS_TCXO_CTRL, ( uint8_t ) tcxo_voltage,    ( uint8_t )( timeout >> 16 ),
        ( uint8_t )( timeout >> 8 ),  ( uint8_t )( timeout >> 0 ),
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

void sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_TYPE] = {
        SX126X_GET_PKT_TYPE,
        SX126X_NOP,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_TYPE, ( uint8_t* ) pkt_type, 1 );
}

void sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD_PARAMS] = {
        SX126X_SET_CAD_PARAMS,
        ( uint8_t ) params->cad_symb_nb,
        params->cad_detect_peak,
        params->cad_detect_min,
        ( uint8_t ) params->cad_exit_mode,
        ( uint8_t )( params->cad_timeout >> 16 ),
        ( uint8_t )( params->cad_timeout >> 8 ),
        ( uint8_t )( params->cad_timeout >> 0 ),
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD_PARAMS, 0, 0 );
}

void sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         exp    = 0;
    uint8_t         mant =
        ( ( ( nb_of_symbs > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT ) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : nb_of_symbs ) +
          1 ) >>
        1;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    const uint8_t buf[SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT] = {
        SX126X_SET_LORA_SYMB_NUM_TIMEOUT,
        mant << ( 2 * exp + 1 ),
    };

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT, 0, 0 );

    if( ( status == SX126X_STATUS_OK ) && ( nb_of_symbs > 0 ) )
    {
        uint8_t reg = exp + ( mant << 3 );
        status      = sx126x_write_register( context, SX126X_REG_LR_SYNCH_TIMEOUT, &reg, 1 );
    }

    sx126x.api_status = status;
}

//
// Communication Status Information
//

void sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status )
{
    const uint8_t buf[SX126X_SIZE_GET_STATUS] = {
        SX126X_GET_STATUS,
    };
    uint8_t         status_local = 0;
    sx126x_status_t status       = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATUS, &status_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        radio_status->cmd_status =
            ( sx126x_cmd_status_t )( ( status_local & SX126X_CMD_STATUS_MASK ) >> SX126X_CMD_STATUS_POS );
        radio_status->chip_mode =
            ( sx126x_chip_modes_t )( ( status_local & SX126X_CHIP_MODES_MASK ) >> SX126X_CHIP_MODES_POS );
    }

    sx126x.api_status = status;
}

void sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status )
{
    const uint8_t buf[SX126X_SIZE_GET_RX_BUFFER_STATUS] = {
        SX126X_GET_RX_BUFFER_STATUS,
        SX126X_NOP,
    };
    uint8_t         status_local[sizeof( sx126x_rx_buffer_status_t )] = { 0x00 };
    sx126x_status_t status                                            = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RX_BUFFER_STATUS, status_local,
                                                  sizeof( sx126x_rx_buffer_status_t ) );

    if( status == SX126X_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = status_local[0];
        rx_buffer_status->buffer_start_pointer = status_local[1];
    }

    sx126x.api_status = status;
}

void sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t         pkt_status_local[3] = { 0x00 };
    sx126x_status_t status              = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local, 3 );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rx_status.pkt_sent =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.pkt_received =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.abort_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.length_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.crc_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.adrs_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ) != 0 ) ? true : false;

        pkt_status->rssi_sync = ( int8_t )( -pkt_status_local[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    sx126x.api_status = status;
}

void sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t         pkt_status_local[sizeof( sx126x_pkt_status_lora_t )] = { 0x00 };
    sx126x_status_t status                                               = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local,
                                                  sizeof( sx126x_pkt_status_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = ( int8_t )( -pkt_status_local[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) pkt_status_local[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    sx126x.api_status = status;
}

void sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    const uint8_t buf[SX126X_SIZE_GET_RSSI_INST] = {
        SX126X_GET_RSSI_INST,
        SX126X_NOP,
    };
    uint8_t         rssi_local = 0x00;
    sx126x_status_t status     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RSSI_INST, &rssi_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        *rssi_in_dbm = ( int8_t )( -rssi_local >> 1 );
    }

    sx126x.api_status = status;
}

void sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t         stats_local[sizeof( sx126x_stats_gfsk_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_gfsk_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }

    sx126x.api_status = status;
}

void sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t         stats_local[sizeof( sx126x_stats_lora_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }
    sx126x.api_status = status;
}

void sx126x_reset_stats( const void* context )
{
    const uint8_t buf[SX126X_SIZE_RESET_STATS] = {
        SX126X_RESET_STATS, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_RESET_STATS, 0, 0 );
}

//
// Miscellaneous
//

void sx126x_reset( const void* context )
{
    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_reset( context );
}

void sx126x_wakeup( const void* context )
{
    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_wakeup( context );
}

void sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    if( bw != 0 )
    {
        status = SX126X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
        {
            if( bw <= gfsk_bw[i].bw )
            {
                *param = gfsk_bw[i].param;
                status = SX126X_STATUS_OK;
                break;
            }
        }
    }

    sx126x.api_status = status;
}

uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case SX126X_LORA_BW_007:
        bw_in_hz = 7812UL;
        break;
    case SX126X_LORA_BW_010:
        bw_in_hz = 10417UL;
        break;
    case SX126X_LORA_BW_015:
        bw_in_hz = 15625UL;
        break;
    case SX126X_LORA_BW_020:
        bw_in_hz = 20833UL;
        break;
    case SX126X_LORA_BW_031:
        bw_in_hz = 31250UL;
        break;
    case SX126X_LORA_BW_041:
        bw_in_hz = 41667UL;
        break;
    case SX126X_LORA_BW_062:
        bw_in_hz = 62500UL;
        break;
    case SX126X_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case SX126X_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case SX126X_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    }

    return bw_in_hz;
}

void sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    sx126x_status_t status;

    uint8_t tmp_ana_lna   = 0x00;
    uint8_t tmp_ana_mixer = 0x00;
    uint8_t tmp           = 0x00;

    // Configure for random number generation
    status = sx126x_read_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }
    tmp    = tmp_ana_lna & ~( 1 << 0 );
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }

    status = sx126x_read_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }
    tmp    = tmp_ana_mixer & ~( 1 << 7 );
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }

    // Start RX continuous
    status = sx126x_set_rx_with_timeout_in_rtc_step( context, SX126X_RX_CONTINUOUS );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = sx126x_read_register( context, SX126X_REG_RNGBASEADDRESS, ( uint8_t* ) &numbers[i], 4 );
        if( status != SX126X_STATUS_OK )
        {
            sx126x.api_status = status;
            return;
        }
    }

    sx126x_set_standby( context, SX126X_STANDBY_CFG_RC );
    if( sx126x.api_status != SX126X_STATUS_OK )
    {
        return;
    }

    // Restore registers
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        sx126x.api_status = status;
        return;
    }
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );

    sx126x.api_status = status;
}

//
// Registers access
//

void sx126x_cfg_rx_boosted( const void* context, const bool state )
{
    if( state == true )
    {
        sx126x.api_status = sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x96 }, 1 );
    }
    else
    {
        sx126x.api_status = sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x94 }, 1 );
    }
}

void sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    // Victor Kalenda Addition Start
    memcpy(sx126x.gfsk_sync_word, sync_word, sync_word_len);
    sx126x.gfsk_packet_params.sync_word_len_in_bits = 8 * sync_word_len;
    sx126x.reset_params = true;
    // Victor Kalenda Addition End

    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buf[8] = { 0 };

    if( sync_word_len <= 8 )
    {
        memcpy( buf, sync_word, sync_word_len );
        status = sx126x_write_register( context, SX126X_REG_SYNCWORDBASEADDRESS, buf, 8 );
    }

    sx126x.api_status = status;
}

void sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    // Victor Kalenda Addition Start
    sx126x.lora_sync_word = sync_word;
    sx126x.reset_params = true;
    // Victor Kalenda Addition End

    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         buffer[2] = { 0x00 };

    status = sx126x_read_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );

    if( status == SX126X_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = sx126x_write_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );
    }

    sx126x.api_status = status;
}

void sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    uint8_t s[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    sx126x.api_status = sx126x_write_register( context, SX126X_REG_CRCSEEDBASEADDRESS, s, sizeof( s ) );
}

void sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    sx126x.api_status = sx126x_write_register( context, SX126X_REG_CRCPOLYBASEADDRESS, poly, sizeof( poly ) );
}

void sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    // The SX126X_REG_WHITSEEDBASEADDRESS @ref LSBit is used for the seed value. The 7 MSBits must not be modified.
    // Thus, we first need to read the current value and then change the LSB according to the provided seed @ref value.
    status = sx126x_read_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
    if( status == SX126X_STATUS_OK )
    {
        reg_value = ( reg_value & 0xFE ) | ( ( uint8_t )( seed >> 8 ) & 0x01 );
        status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            reg_value = ( uint8_t ) seed;
            status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS + 1, &reg_value, 1 );
        }
    }

    sx126x.api_status = status;
}

void sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
    sx126x.api_status = ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

void sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta, const uint8_t trimming_cap_xtb )
{
    uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

    sx126x.api_status = ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_XTATRIM, trimming_capacitor_values, 2 );
}

void sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr, uint8_t register_nb )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buffer[9];

    status = sx126x_read_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

    if( status == SX126X_STATUS_OK )
    {
        const uint8_t initial_nb_of_registers = buffer[0];
        uint8_t*      register_list           = &buffer[1];

        for( uint8_t index = 0; index < register_nb; index++ )
        {
            bool register_has_to_be_added = true;

            // Check if the current register is already added to the list
            for( uint8_t i = 0; i < buffer[0]; i++ )
            {
                if( register_addr[index] == ( ( uint16_t ) register_list[2 * i] << 8 ) + register_list[2 * i + 1] )
                {
                    register_has_to_be_added = false;
                    break;
                }
            }

            if( register_has_to_be_added == true )
            {
                if( buffer[0] < SX126X_MAX_NB_REG_IN_RETENTION )
                {
                    register_list[2 * buffer[0]]     = ( uint8_t )( register_addr[index] >> 8 );
                    register_list[2 * buffer[0] + 1] = ( uint8_t )( register_addr[index] >> 0 );
                    buffer[0] += 1;
                }
                else
                {
                    sx126x.api_status = SX126X_STATUS_ERROR;
                    return;
                }
            }
        }

        if( buffer[0] != initial_nb_of_registers )
        {
            status = sx126x_write_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
        }
    }

    sx126x.api_status = status;
}

void sx126x_init_retention_list( const void* context )
{
    const uint16_t list_of_registers[3] = { SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION, SX126X_REG_IQ_POLARITY };

    sx126x_add_registers_to_retention_list( context, list_of_registers,
                                                   sizeof( list_of_registers ) / sizeof( list_of_registers[0] ) );
}

void sx126x_get_lora_params_from_header( const void* context, sx126x_lora_cr_t* cr, bool* crc_is_on )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buffer_cr;
    uint8_t         buffer_crc;

    status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CR, &buffer_cr, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CRC, &buffer_crc, 1 );

        if( status == SX126X_STATUS_OK )
        {
            *cr = ( sx126x_lora_cr_t )( ( buffer_cr & SX126X_REG_LR_HEADER_CR_MASK ) >> SX126X_REG_LR_HEADER_CR_POS );
            *crc_is_on = ( ( buffer_crc & SX126X_REG_LR_HEADER_CRC_MASK ) != 0 ) ? true : false;
        }
    }

    sx126x.api_status = status;
}















/*
 * -----------------------------------------------------------------------------
 * -----------------------------------------------------------------------------
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

// Victor Kalenda Addition Start

// Refer to section 14.5
void set_lora()
{
  // Auto Low Data Rate Optimization, Refer to section 6.1.1.1 for symbol rate. The inverse of this function is the symbol time
  if(sx126x.auto_ldro)
  {
    // Find the lora bandwidth
    double bandwidth = 0;
    for(uint8_t i = 0; i < ( sizeof( lora_bw ) / sizeof( lora_bw_t )); i++ )
    {
      if(sx126x.lora_modulation_params.bw == lora_bw[i].param)
      {
        bandwidth = lora_bw[i].bw;
        break;
      }
    } 
    // Refer to section 6.1.1.4 for 16.38 ms constraint
    // This turns out to be the symbol time for sf 11 and bw 125khz, using the inverse of the symbol rate yields a more precise value of 16.384ms.
    if( pow(2, (double) sx126x.lora_modulation_params.sf) / bandwidth > 16.384 )
    {
      sx126x.lora_modulation_params.ldro = true;
    }
    else
    {
      sx126x.lora_modulation_params.ldro = false;
    }
  }
  // Set the packet type
  sx126x_set_pkt_type(sx126x.reserved, sx126x.packet_type);

  // Set the modulation parameters
  sx126x_set_lora_mod_params(sx126x.reserved, &sx126x.lora_modulation_params);

  // Set the packet parameters
  sx126x_set_lora_pkt_params(sx126x.reserved, &sx126x.lora_packet_params);
}

// Refer to section 14.5
void set_gfsk()
{
  // Set the packet type
  sx126x_set_pkt_type(sx126x.reserved, sx126x.packet_type);

  // Set the modulation parameters
  sx126x_set_gfsk_mod_params(sx126x.reserved , &sx126x.gfsk_modulation_params );

  // Set the packet parameters
  sx126x_set_gfsk_pkt_params(sx126x.reserved, &sx126x.gfsk_packet_params);
}

// function for if the user sets and invalid bitrate or frequency deviation
bool correct_bandwidth(uint32_t raw_br, uint32_t fdev)
{
  for(uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t )); i++ )
  {
    if((raw_br + (2 * fdev) + (2 * sx126x.crystal_frequency_error)) < gfsk_bw[i].bw)
    {
      sx126x.gfsk_modulation_params.bw_dsb_param = (sx126x_gfsk_bw_t)gfsk_bw[i].param;
      return true;
    }
  }
  return false;
}

void check_IRQ()
{
  sx126x_get_and_clear_irq_status(sx126x.reserved, &sx126x.IRQ);
  if(sx126x.IRQ & SX126X_IRQ_NONE)
  {
    return;
  }
  if(sx126x.IRQ & SX126X_IRQ_TX_DONE)
  {
    sx126x.interrupts.tx_done = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_RX_DONE)
  {
    sx126x.interrupts.rx_done = true;
    // Refer to section 15.3
    sx126x_stop_rtc(sx126x.reserved);
  }
  if(sx126x.IRQ & SX126X_IRQ_PREAMBLE_DETECTED)
  {
    sx126x.interrupts.preamble_detected = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_SYNC_WORD_VALID)
  {
    sx126x.interrupts.sync_word_valid = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_HEADER_VALID)
  {
    sx126x.interrupts.header_valid = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_HEADER_ERROR)
  {
    sx126x.interrupts.header_error = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_CRC_ERROR)
  {
    sx126x.interrupts.crc_error = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_CAD_DONE)
  {
    sx126x.interrupts.cad_done = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_CAD_DETECTED)
  {
    sx126x.interrupts.cad_detected = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_TIMEOUT)
  {
    sx126x.interrupts.timeout = true;
  }
  if(sx126x.IRQ & SX126X_IRQ_LR_FHSS_HOP)
  {
    sx126x.interrupts.lr_fhss_hop = true;
  }
}

void check_if_reset_is_required_now()
{
  sx126x_get_status(sx126x.reserved, &sx126x.chip_status);
  // If the chip is in receive mode, reset the chip immediately
  if(sx126x.chip_status.chip_mode <= SX126X_CHIP_MODE_RX && sx126x.reset_params)
  {
    set_packet_type(sx126x.packet_type); 
    sx126x.reset_params = false;
  }
}

// Victor Kalenda Addition End









//
// Buffer Access
//

void sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_BUFFER] = {
        SX126X_WRITE_BUFFER,
        offset,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_BUFFER, buffer, size );
}

void sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_BUFFER] = {
        SX126X_READ_BUFFER,
        offset,
        SX126X_NOP,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_BUFFER, buffer, size );
}

sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address, const uint8_t rx_base_address )
{
    const uint8_t buf[SX126X_SIZE_SET_BUFFER_BASE_ADDRESS] = {
        SX126X_SET_BUFFER_BASE_ADDRESS,
        tx_base_address,
        rx_base_address,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_TX] = {
        SX126X_SET_TX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX, 0, 0 );
}

sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX] = {
        SX126X_SET_RX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX, 0, 0 );
}

void sx126x_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS && !SX126X_RX_CONTINUOUS )
    {
        sx126x.api_status = SX126X_STATUS_UNKNOWN_VALUE;
        return;
    }

    uint32_t timeout_in_rtc_step = 0;

    if(timeout_in_ms == SX126X_RX_CONTINUOUS)
    {
      timeout_in_rtc_step = SX126X_RX_CONTINUOUS;
    }
    else
    {
      timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );
    }

    sx126x.api_status = sx126x_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

void sx126x_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        sx126x.api_status = SX126X_STATUS_UNKNOWN_VALUE;
        return;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    sx126x.api_status = sx126x_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

void sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode )
{
    const uint8_t buf[SX126X_SIZE_SET_REGULATOR_MODE] = {
        SX126X_SET_REGULATOR_MODE,
        ( uint8_t ) mode,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE_IMAGE] = {
        SX126X_CALIBRATE_IMAGE,
        freq1,
        freq2,
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context, const uint32_t rx_time_in_rtc_step, const uint32_t sleep_time_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_DUTY_CYCLE] = {
        SX126X_SET_RX_DUTY_CYCLE,
        ( uint8_t )( rx_time_in_rtc_step >> 16 ),
        ( uint8_t )( rx_time_in_rtc_step >> 8 ),
        ( uint8_t )( rx_time_in_rtc_step >> 0 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 16 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 8 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

void sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PA_CFG] = {
        SX126X_SET_PA_CFG, params->pa_duty_cycle, params->hp_max, params->device_sel, params->pa_lut,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PA_CFG, 0, 0 );
}

sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_REGISTER] = {
        SX126X_WRITE_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_REGISTER, buffer, size );
}

sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_REGISTER] = {
        SX126X_READ_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size );
}

sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    const uint8_t buf[SX126X_SIZE_SET_RF_FREQUENCY] = {
        SX126X_SET_RF_FREQUENCY,  ( uint8_t )( freq >> 24 ), ( uint8_t )( freq >> 16 ),
        ( uint8_t )( freq >> 8 ), ( uint8_t )( freq >> 0 ),
    };

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

void sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    sx126x.frequency = freq_in_hz; // Victor Kalenda Addition
    const uint32_t freq = sx126x_convert_freq_in_hz_to_pll_step( freq_in_hz );
    sx126x.api_status = sx126x_set_rf_freq_in_pll_steps( context, freq );
}

void sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_TYPE] = {
        SX126X_SET_PKT_TYPE,
        ( uint8_t ) pkt_type,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_TYPE, 0, 0 );
}

void sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm, const sx126x_ramp_time_t ramp_time )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_PARAMS] = {
        SX126X_SET_TX_PARAMS,
        ( uint8_t ) pwr_in_dbm,
        ( uint8_t ) ramp_time,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_PARAMS, 0, 0 );
}

void sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params )
{
    sx126x_status_t status  = SX126X_STATUS_ERROR;
    const uint32_t  bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );
    const uint32_t  fdev    = sx126x_convert_freq_in_hz_to_pll_step( params->fdev_in_hz );
    const uint8_t   buf[SX126X_SIZE_SET_MODULATION_PARAMS_GFSK] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ), params->bw_dsb_param,
        ( uint8_t )( fdev >> 16 ),    ( uint8_t )( fdev >> 8 ),           ( uint8_t )( fdev >> 0 ),
    };

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_GFSK, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_GFSK, ( sx126x_lora_bw_t ) 0 );
        // WORKAROUND END
    }
    sx126x.api_status = status;
}

void sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params )
{
    sx126x_status_t status                                      = SX126X_STATUS_ERROR;
    const uint8_t   buf[SX126X_SIZE_SET_MODULATION_PARAMS_LORA] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( params->sf ), ( uint8_t )( params->bw ),
        ( uint8_t )( params->cr ),    params->ldro & 0x01,
    };

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_LORA, params->bw );
        // WORKAROUND END
    }

    sx126x.api_status = status;
}

void sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_GFSK] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_bits >> 8 ),
        ( uint8_t )( params->preamble_len_in_bits >> 0 ),
        ( uint8_t )( params->preamble_detector ),
        params->sync_word_len_in_bits,
        ( uint8_t )( params->address_filtering ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->dc_free ),
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_GFSK, 0, 0 );
}

void sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_LORA] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_symb >> 8 ),
        ( uint8_t )( params->preamble_len_in_symb >> 0 ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_is_on ? 1 : 0 ),
        ( uint8_t )( params->invert_iq_is_on ? 1 : 0 ),
    };

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_LORA, 0, 0 );

    // WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 §15.4
    if( status == SX126X_STATUS_OK )
    {
        uint8_t reg_value = 0;

        status = sx126x_read_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            if( params->invert_iq_is_on == true )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 when using standard IQ polarity
            }
            status = sx126x_write_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        }
    }
    // WORKAROUND END

    sx126x.api_status = status;
}

//
// Miscellaneous
//

static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type, sx126x_lora_bw_t bw )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    status = sx126x_read_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        if( pkt_type == SX126X_PKT_TYPE_LORA )
        {
            if( bw == SX126X_LORA_BW_500 )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
            }
        }
        else
        {
            reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any (G)FSK configuration
        }

        status = sx126x_write_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );
    }
    return status;
}

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case SX126X_GFSK_CRC_OFF:
        return 0;
    case SX126X_GFSK_CRC_1_BYTE:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES:
        return 2;
    case SX126X_GFSK_CRC_1_BYTE_INV:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES_INV:
        return 2;
    }

    return 0;
}

void sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors )
{
    const uint8_t buf[SX126X_SIZE_GET_DEVICE_ERRORS] = {
        SX126X_GET_DEVICE_ERRORS,
        SX126X_NOP,
    };
    uint8_t         errors_local[sizeof( sx126x_errors_mask_t )] = { 0x00 };
    sx126x_status_t status                                       = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_DEVICE_ERRORS, errors_local,
                                                  sizeof( sx126x_errors_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *errors = ( ( sx126x_errors_mask_t ) errors_local[0] << 8 ) + ( ( sx126x_errors_mask_t ) errors_local[1] << 0 );
    }

    sx126x.api_status = status;
}

void sx126x_clear_device_errors( const void* context )
{
    const uint8_t buf[SX126X_SIZE_CLR_DEVICE_ERRORS] = {
        SX126X_CLR_DEVICE_ERRORS,
        SX126X_NOP,
        SX126X_NOP,
    };

    sx126x.api_status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_DEVICE_ERRORS, 0, 0 );
}

uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p, const sx126x_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == SX126X_LORA_PKT_IMPLICIT;
    const int32_t cr_denom         = mod_p->cr + 4;

    int32_t ceil_denominator;
    int32_t ceil_numerator =
        ( pld_len_in_bytes << 3 ) + ( pkt_p->crc_is_on ? 16 : 0 ) - ( 4 * sf ) + ( pld_is_fix ? 0 : 20 );

    if( sf <= 6 )
    {
        ceil_denominator = 4 * sf;
    }
    else
    {
        ceil_numerator += 8;

        if( mod_p->ldro )
        {
            ceil_denominator = 4 * ( sf - 2 );
        }
        else
        {
            ceil_denominator = 4 * sf;
        }
    }

    if( ceil_numerator < 0 )
    {
        ceil_numerator = 0;
    }

    // Perform integral ceil()
    int32_t intermed =
        ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator ) * cr_denom + pkt_p->preamble_len_in_symb + 12;

    if( sf <= 6 )
    {
        intermed += 2;
    }

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p, const sx126x_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = sx126x_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p )
{
    return pkt_p->preamble_len_in_bits + ( pkt_p->header_type == SX126X_GFSK_PKT_VAR_LEN ? 8 : 0 ) +
           pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes + ( pkt_p->address_filtering == SX126X_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               sx126x_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p, const sx126x_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * SX126X_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
}

uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
    return ( uint32_t )( timeout_in_ms * ( SX126X_RTC_FREQ_IN_HZ / 1000 ) );
}

void sx126x_cfg_tx_clamp( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0x00;

    status = sx126x_read_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        reg_value |= SX126X_REG_TX_CLAMP_CFG_MASK;
        status = sx126x_write_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );
    }

    sx126x.api_status = status;
}

void sx126x_stop_rtc( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    reg_value = 0;
    status    = sx126x_write_register( context, SX126X_REG_RTC_CTRL, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );

        if( status == SX126X_STATUS_OK )
        {
            reg_value |= SX126X_REG_EVT_CLR_TIMEOUT_MASK;
            status = sx126x_write_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );
        }
    }
    
    sx126x.api_status = status;
}

/* --- EOF ------------------------------------------------------------------ */
