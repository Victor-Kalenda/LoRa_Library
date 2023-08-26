#include "sx126x_hal.h"
#include <SPI.h>

uint8_t busy = 0;
uint8_t reset = 0;
uint8_t nss = 0;
bool sleeping = false;

void sx126x_hal_init( uint8_t cs_pin, uint8_t reset_pin, uint8_t busy_pin, int8_t dio1)
{
  nss = cs_pin;
  reset = reset_pin;
  busy = busy_pin;

  pinMode (cs_pin, OUTPUT);
  pinMode (reset_pin, OUTPUT);
  
  if(dio1 != -1)
  {
    pinMode (dio1, INPUT);
  }
  
  pinMode (busy_pin, INPUT);
  digitalWrite(cs_pin,1);
  digitalWrite(reset_pin,1);  
  SPI.begin();
  Serial.println(F("HAL Initialized"));
}

static uint8_t SPI_tx_rx( uint8_t data )
{
  uint8_t rx_data = 0;
  rx_data = SPI.transfer(data);

  return rx_data;
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length )
{
  // Refer to section 8.3.1
  while(digitalRead(busy) == 1);

  digitalWrite(nss,0);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  for( uint16_t i = 0; i < command_length; i++ )
  {
    SPI_tx_rx( command[i] );
  }

  for( uint16_t i = 0; i < data_length; i++ )
  {
    SPI_tx_rx( data[i] );
  }
  SPI.endTransaction();
  digitalWrite(nss,1);

  // SX126x_SET_SLEEP = 0x84 in sx126x.cpp
  if( command[0] == 0x84)
  {
    // Refer to section 8.3.1
    sleeping = true;
  }
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length, uint8_t* data, const uint16_t data_length )
{
  // Refer to section 8.3.1
  if(sleeping)
  {
    return SX126X_HAL_STATUS_ERROR;
  }
  while(digitalRead(busy) == 1);

  digitalWrite(nss,0);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // Send the command
  for(uint16_t i = 0; i < command_length; i++)
  {
    SPI_tx_rx( command[i] );
  }

  // Read the response and store it in the data buffer
  for( uint16_t i = 0; i < data_length; i++ )
  {
    data[i] = SPI_tx_rx( SX126X_NOP );
  }
  SPI.endTransaction();
  digitalWrite(nss,1);

  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
  delayMicroseconds( 10000 );
  digitalWrite(reset,0);
  delayMicroseconds( 20000 );
  digitalWrite(reset,1);
  delayMicroseconds( 10000 );

  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
  noInterrupts();

  digitalWrite(nss,0);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // SX126x_GET_STATUS = 0xC0 in sx126x.cpp
  
  SPI_tx_rx( 0xC0 );
  uint8_t status_local = SPI_tx_rx( SX126X_NOP );

  SPI.endTransaction();
  digitalWrite(nss,1);

  // Wait for chip to be ready.
  while(digitalRead(busy) == 1);
  interrupts();

  return SX126X_HAL_STATUS_OK;
}