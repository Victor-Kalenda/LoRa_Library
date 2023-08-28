/*
  Purpose:

  Allow users to set the chip to any setting imaginable using serial commands

*/
#include <LoRa_Library.h>
#include "commands.h"

#define SW 8

const void* reserved;

sx126x_pkt_status_lora_t lora_packet_status;
sx126x_rx_buffer_status_t rx_buffer_status;
sx126x_stats_lora_t lora_rx_info;
sx126x_pkt_status_gfsk_t gfsk_packet_status;
sx126x_errors_mask_t device_errors;
sx126x_stats_lora_t lora_packet_statistics;
sx126x_stats_gfsk_t gfsk_packet_statistics;

void setup() 
{
  Serial.begin(9600);

  // Refer to Dorji Applied Technologies DAD06 Development Board Datasheet for pinout
  // SW Pin on DRF1262T
  pinMode(SW, OUTPUT);
  // Activate the antenna switch for testing
  digitalWrite(SW, HIGH);

  start_params_t chip_setup;
  chip_setup.busy = 3;
  chip_setup.reset = 14;
  chip_setup.NSS = 7;
  chip_setup.dio1 = 5; // set dio1 to -1 to deactivate automatic interrupt handling
  chip_setup.chip = SX1262;
  chip_setup.crystal = TCXO;
  chip_setup.crystal_frequency_error = 0;
  chip_setup.regulator = SX126X_REG_MODE_DCDC;
  
  initialize_chip(chip_setup);
}

void loop() 
{
  uint32_t input[10];
  while(!get_input(input));
  uint32_t command = input[0];
  uint32_t value_1 = input[1];
  uint32_t value_2 = input[2];

  switch(command)
  {
    case MODEM:
    {
      if(check_input(value_1, 0, 1))
      {
        set_packet_type(sx126x_pkt_type_t(value_1));
        Serial.println(F("Modem Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case POWER:
    {
      if(check_input(value_1, 0, 5))
      {
        set_power(sx126x_tx_power_t(value_1));
        Serial.println(F("Power Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case FREQUENCY:
    {
      if(check_input(value_1, 150000000, 960000000))
      {
        set_frequency(value_1);
        Serial.print(F("Frequency Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case CALIBRATE:
    {
      if(check_input(value_1, 0, 7))
      {
        uint32_t value = pow(2, value_1);
        if(value == 128)
        {
          value--;
        }
        sx126x_cal(reserved, sx126x_cal_mask_t(value));
        Serial.println(F("Calibration Complete"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case CALIBRATE_IMAGE_IN_MHZ:
    {
      if(check_input(value_1, 150, 960) && check_input(value_2, 150, 960))
      {
        sx126x_cal_img_in_mhz(reserved, value_1, value_2);
        Serial.println(F("Image Calibration Complete"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case SET_OCP_VALUE:
    {
      if(check_input(value_1, 0, 63))
      {
        sx126x_set_ocp_value(reserved, value_1);
        Serial.print(F("OCP set to "));
        Serial.println(float(value_1) * 2.5);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case SET_TRIMMING_CAPACITORS:
    {
      if(check_input(value_1, 0, 47) && check_input(value_2, 0, 47))
      {
        // Refer to section 4.1.3
        sx126x_set_standby(reserved, SX126X_STANDBY_CFG_XOSC);
        sx126x_set_trimming_capacitor_values(reserved, value_1, value_2);
        Serial.println(F("Trimming Capacitors Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }





    case LORA_CRC:
    {
      if(check_input(value_1, 0, 1))
      {
        lora_crc_on(value_1);
        Serial.println(F("CRC Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_INVERT_IQ:
    {
      if(check_input(value_1, 0, 1))
      {
        lora_invert_iq(value_1);
        Serial.println(F("Invert IQ Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_HEADER_TYPE:
    {
      if(check_input(value_1, 0, 1))
      {
        lora_set_header_type(sx126x_lora_pkt_len_modes_t(value_1));
        Serial.println(F("Header Type Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_PLD_LEN:
    {
      if(check_input(value_1, 0, 255))
      {
        lora_set_pld_len(value_1);
        Serial.print(F("Lora Payload Length Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_PREAMBLE_LEN_IN_SYMBOLS:
    {
      if(check_input(value_1, 10, 65535))
      {
        lora_set_pre_len(value_1);
        Serial.print(F("Lora Preamble Length Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_SPREADFACTOR:
    {
      if(check_input(value_1, 5, 12))
      {
        lora_set_sf(sx126x_lora_sf_t(value_1));
        Serial.print(F("Lora Spreadfactor Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_BANDWIDTH:
    {
      if(check_input(value_1, 0, 10) && value_1 != 7)
      {
        lora_set_bw(sx126x_lora_bw_t(value_1));
        Serial.println(F("Lora Bandwidth Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_CODERATE:
    {
      if(check_input(value_1, 1, 4))
      {
        lora_set_cr(sx126x_lora_cr_t(value_1));
        Serial.print(F("Lora Code Rate Set to 4 / "));
        Serial.println(value_1 + 4);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_LDRO:
    {
      if(check_input(value_1, 0, 1))
      {
        lora_set_ldro(value_1);
        Serial.println(F("Lora LDRO Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case LORA_AUTO_LDRO:
    {
      if(check_input(value_1, 0, 1))
      {
        lora_set_auto_ldro(value_1);
        Serial.println(F("Lora Auto LDRO Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }




    case GFSK_PREAMBLE_LEN_IN_BITS:
    {
      if(check_input(value_1, 8, 65535))
      {
        gfsk_set_pre_len(value_1);
        Serial.print(F("GFSK Preamble Length Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_PREAMBLE_DETECT_LEN:
    {
      if(check_input(value_1, 4, 7) || value_1 == 0)
      {
        gfsk_set_pre_detector(sx126x_gfsk_preamble_detector_t(value_1));
        Serial.println(F("GFSK Preamble Detector Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_NODE_ADDRESS:
    {
      if(check_input(value_1, 0, 255))
      {
        gfsk_set_node_address(reserved, value_1);
        Serial.println(F("GFSK Node Address Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_BROADCAST_ADDRESS:
    {
      if(check_input(value_1, 0, 255))
      {
        gfsk_set_broadcast_address(reserved, value_1);
        Serial.println(F("GFSK Broadcast Address Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_ADDRESS_FILTERING_ON:
    {
      if(check_input(value_1, 0, 2))
      {
        gfsk_addr_filter(sx126x_gfsk_address_filtering_t(value_1));
        Serial.println(F("GFSK Address Filtering Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_HEADER:
    {
      if(check_input(value_1, 0, 1))
      {
        gfsk_set_header_type(sx126x_gfsk_pkt_len_modes_t(value_1));
        Serial.println(F("GFSK Header Type Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_PLD_LEN:
    {
      if(check_input(value_1, 0, 255))
      {
        gfsk_set_pld_len(value_1);
        Serial.print(F("GFSK Payload Length Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_CRC:
    {
      if(check_input(value_1, 0, 2) || value_1 == 4 || value_1 == 6)
      {
        gfsk_crc(sx126x_gfsk_crc_types_t(value_1));
        Serial.println(F("GFSK CRC Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_DC_FREE:
    {
      if(check_input(value_1, 0, 1))
      {
        gfsk_whitening_on(sx126x_gfsk_dc_free_t(value_1));
        Serial.println(F("GFSK Whitening Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_BR_IN_BPS:
    {
      if(check_input(value_1, 600, 300000))
      {
        gfsk_set_br(value_1);
        Serial.print(F("GFSK Bit Rate Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_FDEV_IN_HZ:
    {
      if(check_input(value_1, 600, 200000))
      {
        gfsk_set_fdev(value_1);
        Serial.print(F("GFSK Frequency Deviation Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_PULSE_SHAPE:
    {
      if(check_input(value_1, 8, 11) || value_1 == 0)
      {
        gfsk_set_ps(sx126x_gfsk_pulse_shape_t(value_1));
        Serial.println(F("GFSK Pulse Shape Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_BW_DSB_PARAM:
    {
      if(check_input(value_1, 9, 31) && value_1 != 16 && value_1 != 24)
      {
        gfsk_set_bw(sx126x_gfsk_bw_t(value_1));
        Serial.println(F("GFSK Bandwidth Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }



    case GFSK_SYNC_WORD:
    {
      if(check_input(input[9], 0, 8))
      {
        uint8_t sync_word[8];
        for(int i = 1; i <= 8; i++)
        {
          sync_word[i-1] = input[i];
          if(!check_input(sync_word[i-1], 0, 255))
          {
            Serial.println(F("Invalid GFSK Sync Word"));
            break;
          }
        }
        sx126x_set_gfsk_sync_word(reserved, sync_word, input[9]);
        Serial.println(F("GFSK Sync Word Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_CRC_SEED:
    {
      if(check_input(value_1, 0, 65535))
      {
        sx126x_set_gfsk_crc_seed(reserved, value_1);
        Serial.println(F("GFSK CRC Seed Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_CRC_POLYNOMIAL:
    {
      if(check_input(value_1, 0, 65535))
      {
        sx126x_set_gfsk_crc_polynomial(reserved, value_1);
        Serial.println(F("GFSK CRC Polynomial Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case GFSK_WHITENING_SEED:
    {
      if(check_input(value_1, 0, 65535))
      {
        sx126x_set_gfsk_whitening_seed(reserved, value_1);
        Serial.println(F("GFSK Whitening Seed Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }




    case SET_TRANSMIT_TIMEOUT:
    {
      if(check_input(value_1, 0, 262142))
      {
        set_transmit_timeout(value_1);
        Serial.print(F("Transmit Timeout Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case READ_TRANSMIT_BUFFER:
    {
      uint8_t outgoing[9];
      read_transmit_buffer(outgoing);
      for(int i = 0; i < get_payload_length() - 1; i++)
      {
        Serial.print(outgoing[i]);
        Serial.print(F(" - "));
      }
      Serial.println(outgoing[get_payload_length() - 1]);
      break;
    }
    case SET_TRANSMIT_BUFFER_ADDRESS:
    {
      if(check_input(value_1, 0, 255))
      {
        set_transmit_buffer_address(value_1);
        Serial.print(F("Transmit Buffer Base Address Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case TRANSMIT:
    {
      uint8_t outgoing[9];
      for(int i = 1; i <= 9; i++)
      {
        outgoing[i-1] = input[i];
        if(!check_input(outgoing[i-1], 0, 255))
        {
          Serial.println(F("Invalid Transmit Buffer Input"));
          break;
        }
      }
      transmit(outgoing);
      break;
    }
    


    case SET_LORA_SYMBOL_TIMEOUT:
    {
      if(check_input(value_1, 0, 248))
      {
        sx126x_set_lora_symb_nb_timeout(reserved, value_1);
        Serial.println(F("Lora Symbol Timeout Set to "));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case SET_RECEIVE_TIMEOUT:
    {
      if(check_input(value_1, 0, 262142) || value_1 == SX126X_RX_CONTINUOUS)
      {
        set_receive_timeout(value_1);
        Serial.print(F("Receive Timeout Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case READ_RECEIVE_BUFFER:
    {
      uint8_t incoming[9];
      read_receive_buffer(incoming);
      for(int i = 0; i < get_payload_length() - 1; i++)
      {
        Serial.print(incoming[i]);
        Serial.print(F(" - "));
      }
      Serial.println(incoming[get_payload_length() - 1]);
      break;
    }
    case SET_RECEIVE_BUFFER_ADDRESS:
    {
      if(check_input(value_1, 0, 255))
      {
        set_receive_buffer_address(value_1);
        Serial.print(F("Receive Buffer Base Address Set to "));
        Serial.println(value_1);
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case RECEIVE_MODE:
    {
      receive_mode();
      Serial.println(F("Receive Mode Set"));
      break;
    }
    case RECEIVE_MODE_DUTY_CYCLING:
    {
      receive_mode_duty_cycle(value_1, value_2);
      Serial.println(F("Receive Mode with Duty Cycling Set"));
      break;
    }
    case STANDBY_MODE:
    {
      if(check_input(value_1, 0, 1))
      {
        sx126x_set_standby(reserved, sx126x_standby_cfg_t(value_1));
        Serial.println(F("Standby Mode Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case FALLBACK_MODE:
    {
      if(value_1 == 32 || value_1 == 48 || value_1 == 64)
      {
        sx126x_set_rx_tx_fallback_mode(reserved, sx126x_fallback_modes_t(value_1));
        Serial.println(F("Fallback Mode Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case SLEEP:
    {
      if(value_1 == 0 || value_1 == 4)
      {
        sx126x_set_sleep(reserved, sx126x_sleep_cfgs_t(value_1));
        Serial.println(F("Sleep Mode Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case WAKEUP:
    {
      sx126x_wakeup(reserved);
      Serial.println(F("Wakeup Initiated"));
      break;
    }
    case SET_CAD_PARAMS:
    {
      if(check_input(input[1], 0, 4) && check_input(input[2], 0, 255) && check_input(input[3], 0, 255)
          && (check_input(input[4], 0, 1) || input[4] == 16))
      {
        sx126x_cad_params_t params = {sx126x_cad_symbs_t(input[1]), uint8_t(input[2]), uint8_t(input[3]), sx126x_cad_exit_modes_t(input[4]), input[5]};
        sx126x_set_cad_params(reserved, &params);
        Serial.println(F("Personalized CAD Parameters Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case CAD:
    {
      if(check_input(value_1, 0, 1))
      {
        if(value_1)
        {
          set_cad_params();
          Serial.println(F("Best Known CAD Parameters Set"));
        }
        sx126x_set_cad(reserved);
        Serial.println(F("CAD Mode Set"));
      }
      else
      {
        Serial.println(F("Invalid Input"));
      }
      break;
    }
    case FREQUENCY_SYNTHESIS_MODE:
    {
      sx126x_set_fs(reserved);
      Serial.println(F("Frequency Synthesis Mode Set"));
      break;
    }
    case TX_CONTINUOUS_WAVE:
    {
      sx126x_set_tx_cw(reserved);
      Serial.println(F("TX Continuous Wave Mode Set"));
      break;
    }
    case TX_INFINITE_PREAMBLE:
    {
      sx126x_set_tx_infinite_preamble(reserved);
      Serial.println(F("TX Infinite Preamble Mode Set"));
      break;
    }
  




    case GET_INTERRUPTS:
    {
      check_interrupts();
      break;
    }
    case CLEAR_INTERRUPTS:
    {
      clear_interrupts();
      Serial.println(F("Interrupts Cleared"));
      break;
    }
    case GET_TIME_ON_AIR:
    {
      Serial.print(F("Time on air = "));
      Serial.println(get_time_on_air());
      break;
    }
    case GET_API_STATUS:
    {
      check_api_status();
      break;
    }
    case GET_CHIP_STATUS:
    {
      check_chip_status();
      break;
    }
    case GET_DEVICE_ERRRORS:
    {
      check_device_errors();
      break;
    }
    case CHECK_RECEIVE_BUFFER:
    {
      check_rx_buffer();
      break;
    }
    case CHECK_LORA_PACKET:
    {
      check_lora_packet();
      break;
    }
    case CHECK_GFSK_PACKET:
    {
      check_gfsk_packet();
      break;
    }
    case CHECK_PLD_LEN:
    {
      Serial.print(F("Payload Length (bytes) = "));
      Serial.println(get_payload_length());
      break;
    }
    case CHECK_PREAMBLE_LEN:
    {
      Serial.print(F("Preamble Length (symbols/bits) = "));
      Serial.println(get_preamble_length());
      break;
    }
    case GET_LORA_PARAMS_FROM_HEADER:
    {
      sx126x_lora_cr_t coderate;
      bool crc_is_on;
      sx126x_get_lora_params_from_header(reserved, &coderate, &crc_is_on);
      Serial.println(F("Received Packet"));
      Serial.print(F("- Coderate = 4 / "));
      Serial.println(coderate + 4);
      if(crc_is_on)
      {
        Serial.println(F("- CRC = ON"));
      }
      else
      {
        Serial.println(F("- CRC = OFF"));
      }
      break;
    }
    case GET_PACKET_TYPE:
    {
      Serial.print(F("Packet Type = "));
      switch(get_packet_type())
      {
        case SX126X_PKT_TYPE_GFSK:
        {
          Serial.println("GFSK");
          break;
        }
        case SX126X_PKT_TYPE_LORA:
        {
          Serial.println("LoRa");
          break;
        }
      }
      break;
    }


    case GET_LORA_PACKET_STATS:
    {
      check_lora_packet_stats();
      break;
    }
    case GET_GFSK_PACKET_STATS:
    {
      check_gfsk_packet_stats();
      break;
    }
    case RESET_PACKET_STATS:
    {
      sx126x_reset_stats(reserved);
      break;
    }
    case GET_LORA_SYNC_WORD:
    {
      Serial.print(F("Lora Sync Word = "));
      Serial.println(get_lora_sync_word());
      break;
    }
    case GET_GFSK_SYNC_WORD:
    {
      Serial.print(F("GFSK Sync Word = "));
      uint8_t sync_word[8];
      get_gfsk_sync_word(sync_word);
      for(int i = 0; i < 7; i++)
      {
        Serial.print(sync_word[i]);
        Serial.print(F(" - "));
      }
      Serial.println(sync_word[7]);
      break;
    }
    default:
    {
      Serial.println(F("Invalid Input"));
    }
  }
}

bool get_input(uint32_t *command)
{
  static int b = 0;
  static char ch[11];
  static int i = -1;
  if(Serial.available() > 0)
  {
    i++;
    ch[i] = (char)Serial.read();

    if(ch[i] == '\r' || ch[i] == '\n')
    {
      command[b] = atol(ch);
      Serial.println(command[b]);
      for(int j = 0; j < 10; j++)
      {
        ch[j] = '\0';
      }      
      i = -1;
      b = 0;
      return true;
    }
    if(isSpace(ch[i]))
    {
      command[b] = atol(ch);
      Serial.print(command[b]);
      Serial.print(F(" "));
      for(int j = 0; j < 10; j++)
      {
        ch[j] = '\0';
      }
      i = -1;
      b++;
    }
  }
  return false;
}

bool check_input(const uint32_t value, const uint32_t lower, const uint32_t upper)
{
  return (value >= lower && value <= upper);
}

bool config_lora(bool crc_is_on, bool invert_iq_is_on, sx126x_lora_pkt_len_modes_t header_type, uint8_t payload_length_in_bytes,
                uint16_t preamble_length_in_symbols, sx126x_lora_sf_t spreadfactor, sx126x_lora_bw_t bandwidth, 
                sx126x_lora_cr_t coderate, bool ldro_is_on)
{
  if(!lora_set_pre_len(preamble_length_in_symbols))
  {
    Serial.print(F("Preamble Length is Invalid"));
    return false;
  }
  lora_crc_on(crc_is_on);
  lora_invert_iq(invert_iq_is_on);
  lora_set_header_type(header_type);
  lora_set_pld_len(payload_length_in_bytes);
  lora_set_sf(spreadfactor);
  lora_set_bw(bandwidth);
  lora_set_cr(coderate);
  lora_set_ldro(ldro_is_on);
  set_packet_type(SX126X_PKT_TYPE_LORA);
}

bool config_gfsk(uint16_t preamble_len_in_bits, sx126x_gfsk_preamble_detector_t preamble_detection_length, sx126x_gfsk_address_filtering_t address_filtering,
                sx126x_gfsk_pkt_len_modes_t header_type, uint8_t pld_len_in_bytes, sx126x_gfsk_crc_types_t crc_type, sx126x_gfsk_dc_free_t dc_free,
                uint32_t br_in_bps, uint32_t fdev_in_hz, sx126x_gfsk_pulse_shape_t pulse_shape, sx126x_gfsk_bw_t bw_dsb_param)
{
  if(!gfsk_set_pre_len(preamble_len_in_bits))
  {
    Serial.println(F("Preamble Length is Invalid"));
    return false;
  }
  // set the bandwidth before frequency deviation and bitrate so that bandwidth can be modified if the inputs are invalid
  if(!gfsk_set_bw(bw_dsb_param))
  {
    Serial.println(F("Bandwidth Parameter Invalid"));
    return false;
  }
  if(!gfsk_set_br(br_in_bps))
  {
    Serial.println(F("Bitrate Parameter Invalid"));
    return false;
  }
  if(!gfsk_set_fdev(fdev_in_hz))
  {
    Serial.println(F("Frequency Deviation Parameter Invalid"));
    return false;
  }
  gfsk_set_pre_detector(preamble_detection_length);
  gfsk_addr_filter(address_filtering);
  gfsk_set_header_type(header_type);
  gfsk_set_pld_len(pld_len_in_bytes);
  gfsk_crc(crc_type);
  gfsk_whitening_on(dc_free);
  gfsk_set_ps(pulse_shape);
  gfsk_set_bw(bw_dsb_param);
}

// Refer to section 13.5.3
void check_lora_packet()
{
  sx126x_get_lora_pkt_status(reserved, &lora_packet_status);
  Serial.print(F("LoRa RSSI Average = "));
  Serial.println(lora_packet_status.rssi_pkt_in_dbm);
  Serial.print(F("LoRa RSSI Estimate Previous Packet = "));
  Serial.println(lora_packet_status.signal_rssi_pkt_in_dbm);
  Serial.print(F("LoRa SNR Estimate = "));
  Serial.println(lora_packet_status.snr_pkt_in_db);
  sx126x_get_lora_stats(reserved, &lora_rx_info);
  Serial.print(F("Number of packets received = "));
  Serial.println(lora_rx_info.nb_pkt_received);
  Serial.print(F("Number of CRC errors = "));
  Serial.println(lora_rx_info.nb_pkt_crc_error);
  Serial.print(F("Number of header errors = "));
  Serial.println(lora_rx_info.nb_pkt_header_error);
}

// Refer to section 13.5.3
// Sync Error and Preamble Error is not implemented from the datasheet
void check_gfsk_packet()
{
  sx126x_get_gfsk_pkt_status(reserved, &gfsk_packet_status);
  Serial.print(F("GFSK Buffer Status = "));
  Serial.print(gfsk_packet_status.rx_status.pkt_sent);
  Serial.print(gfsk_packet_status.rx_status.pkt_received);
  Serial.print(gfsk_packet_status.rx_status.abort_error);
  Serial.print(gfsk_packet_status.rx_status.length_error);
  Serial.print(gfsk_packet_status.rx_status.crc_error);
  Serial.println(gfsk_packet_status.rx_status.adrs_error);
  Serial.print(F("GFSK RSSI Previous Packet = "));
  Serial.println(gfsk_packet_status.rssi_sync);
  Serial.print(F("GFSK RSSI Average = "));
  Serial.println(gfsk_packet_status.rssi_avg);
}

void check_lora_packet_stats()
{
  sx126x_get_lora_stats(reserved, &lora_packet_statistics);
  Serial.print(F("Number of CRC Errors = "));
  Serial.println(lora_packet_statistics.nb_pkt_crc_error);
  Serial.print(F("Number of Header Errors = "));
  Serial.println(lora_packet_statistics.nb_pkt_header_error);
  Serial.print(F("Number of packets received = "));
  Serial.println(lora_packet_statistics.nb_pkt_received);
}

void check_gfsk_packet_stats()
{
  sx126x_get_gfsk_stats(reserved, &gfsk_packet_statistics);
  Serial.print(F("Number of CRC Errors = "));
  Serial.println(gfsk_packet_statistics.nb_pkt_crc_error);
  Serial.print(F("Number of Packet Length Errors = "));
  Serial.println(gfsk_packet_statistics.nb_pkt_len_error);
  Serial.print(F("Number of packets received = "));
  Serial.println(gfsk_packet_statistics.nb_pkt_received);
}

// Refer to section 13.5.1
void check_chip_status()
{
  Serial.print(F("Chip Mode = "));
  Serial.println(get_chip_status().chip_mode);
  Serial.print(F("Command Status = "));
  Serial.println(get_chip_status().cmd_status);
}

// Refer to section 13.6.1
void check_device_errors()
{
  Serial.print(F("Device Errors = "));
  sx126x_get_device_errors(reserved, &device_errors);
  Serial.println(device_errors, BIN);
  if(device_errors != 0)
  {
    Serial.print(F("Clearing Device Errors = "));
    sx126x_clear_device_errors(reserved);
    Serial.println(device_errors, BIN);
  }
}

void check_api_status()
{
  Serial.print("API Status = ");
  Serial.println(get_api_status());
}

void check_rx_buffer()
{
  sx126x_get_rx_buffer_status(reserved, &rx_buffer_status);
  Serial.print("Start Pointer = ");
  Serial.println(rx_buffer_status.buffer_start_pointer);
  Serial.print("Payload Length = ");
  Serial.println(rx_buffer_status.pld_len_in_bytes);
}

void check_interrupts()
{
  sx126x_irq_t interrupts = get_interrupts();
  if(interrupts.tx_done)
  {
    Serial.println(F("TX Done"));
  }
  if(interrupts.rx_done)
  {
    Serial.println(F("RX Done"));
  }
  if(interrupts.preamble_detected)
  {
    Serial.println(F("Preamble Detected"));
  }
  if(interrupts.sync_word_valid)
  {
    Serial.println(F("Sync Word Valid"));
  }
  if(interrupts.header_valid)
  {
    Serial.println(F("Header Valid"));
  }
  if(interrupts.header_error)
  {
    Serial.println(F("Header Error"));
  }
  if(interrupts.crc_error)
  {
    Serial.println(F("CRC Error"));
  }
  if(interrupts.cad_done)
  {
    Serial.println(F("CAD Done"));
  }
  if(interrupts.cad_detected)
  {
    Serial.println(F("CAD Detected"));
  }
  if(interrupts.timeout)
  {
    Serial.println(F("IRQ Timeout"));
  }
  if(interrupts.lr_fhss_hop)
  {
    Serial.println(F("Frequency Hop"));
  }
}
