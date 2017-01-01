// NRF24.cpp
//
// Copyright (C) 2012 Mike McCauley
// $Id: NRF24.cpp,v 1.2 2014/05/20 06:00:55 mikem Exp mikem $

#include "NRF24_low_latency.h"
#include <SPI.h>

// Uses the DigitalIO library from https://github.com/greiman/DigitalIO/
#include "DigitalPin.h"

DigitalPin<SS> chipSelectPin;

NRF24::NRF24(uint8_t chipEnablePin)
{
    _configuration = NRF24_EN_CRC; // Default: 1 byte CRC enabled
    _chipEnablePin = chipEnablePin;
    _address_width = 0; // Invalid address width.
    _broadcast_address[0] = 0;
}

boolean NRF24::init()
{
    // Initialise the slave select pin
    pinMode(_chipEnablePin, OUTPUT);
    digitalWrite(_chipEnablePin, LOW);
    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);
  
    // Added code to initilize the SPI interface and wait 100 ms
    // to allow NRF24 device to "settle".  100 ms may be overkill.
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    // Wait for NRF24 POR (up to 100msec)
    delay(100);

    // start the SPI library:
    // Note the NRF24 wants mode 0, MSB first and default to 1 Mbps
    // NRF24 supports max SPI data rate of 10 Mbps.
    SPI.begin();
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    // Clear interrupts
    if (!spiWriteRegister(NRF24_REG_07_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT))
	return false; // Could not write to device. Not connected?

    // Use DPL mode by default.
    setDynamicPayloadMode();
    
    // Make sure we are powered down
    powerDown();

    // Flush FIFOs
    flushTx();
    flushRx();

    return true;
}

// Low level commands for interfacing with the device
uint8_t NRF24::spiCommand(uint8_t command)
{
    chipSelectPin.low();
    uint8_t status = SPI.transfer(command);
    chipSelectPin.high();
    return status;
}

// Read and write commands
uint8_t NRF24::spiRead(uint8_t command)
{
    chipSelectPin.low();
    SPI.transfer(command); // Send the address, discard status
    uint8_t val = SPI.transfer(0); // The MOSI value is ignored, value is read
    chipSelectPin.high();
    return val;
}

uint8_t NRF24::spiWrite(uint8_t command, uint8_t val)
{
    chipSelectPin.low();
    uint8_t status = SPI.transfer(command);
    SPI.transfer(val); // New register value follows
    chipSelectPin.high();
    return status;
}

void NRF24::spiBurstRead(uint8_t command, uint8_t* dest, uint8_t len)
{
    chipSelectPin.low();
    SPI.transfer(command); // Send the start address, discard status
    while (len--)
	*dest++ = SPI.transfer(0); // The MOSI value is ignored, value is read
    chipSelectPin.high();
    // 300 microsecs for 32 octet payload
}

uint8_t NRF24::spiBurstWrite(uint8_t command, uint8_t* src, uint8_t len)
{
    chipSelectPin.low();
    uint8_t status = SPI.transfer(command);
    while (len--)
	SPI.transfer(*src++);
    chipSelectPin.high();
    return status;
}

// Use the register commands to read and write the registers
uint8_t NRF24::spiReadRegister(uint8_t reg)
{
    return spiRead((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_R_REGISTER);
}

uint8_t NRF24::spiWriteRegister(uint8_t reg, uint8_t val)
{
    return spiWrite((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER, val);
}

void NRF24::spiBurstReadRegister(uint8_t reg, uint8_t* dest, uint8_t len)
{
    return spiBurstRead((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_R_REGISTER, dest, len);
}

uint8_t NRF24::spiBurstWriteRegister(uint8_t reg, uint8_t* src, uint8_t len)
{
    return spiBurstWrite((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER, src, len);
}

uint8_t NRF24::statusRead()
{
    return spiReadRegister(NRF24_REG_07_STATUS);
}

uint8_t NRF24::flushTx()
{
    return spiCommand(NRF24_COMMAND_FLUSH_TX);
}

uint8_t NRF24::flushRx()
{
    return spiCommand(NRF24_COMMAND_FLUSH_RX);
}

boolean NRF24::setChannel(uint8_t channel)
{
    spiWriteRegister(NRF24_REG_05_RF_CH, channel & NRF24_RF_CH);
    return true;
}
boolean NRF24::setConfiguration(uint8_t configuration)
{
    _configuration = configuration;
}

boolean NRF24::setRetry(uint8_t delay, uint8_t count)
{
    spiWriteRegister(NRF24_REG_04_SETUP_RETR, ((delay << 4) & NRF24_ARD) | (count & NRF24_ARC));
    return true;
}

bool NRF24::setPipeAddress(uint8_t pipe, uint8_t* address, uint8_t len)
{
  if (len < 3 || len > 5)
    return false;

  if (_address_width!=0 && _address_width!=len)
    return false;

  _address_width = len;
  spiWriteRegister(NRF24_REG_03_SETUP_AW, len-2);
  spiBurstWriteRegister(NRF24_REG_0A_RX_ADDR_P0 + pipe, address, len);
  return true;
}

bool NRF24::setThisAddress(uint8_t* address, uint8_t len)
{
  // Set pipe 1 for this address
  // RX_ADDR_P2 is set to RX_ADDR_P1 with the LSbyte set to 0xff, for use as a broadcast address
  return setPipeAddress(1, address, len); 
}

bool NRF24::setBroadcastAddress(uint8_t *address, uint8_t len) {
  if (len < 3 || len > 5)
    return false;

  if (_address_width!=0 && _address_width!=len)
    return false;

  // P0 is used to recv from broadcast_address. But in TX mode, P0 needs to be set as Transmit
  // address for auto-acking packets. So P0 needs to be used interchangably. When switching to RX
  // mode P0 is set as broadcast_address in powerUpRx(). When switching to TX mode, P0 is set as
  // Transmit address in setTransmitAddress().
  _address_width = len;
  memcpy(_broadcast_address, address, _address_width);

  return true;
}

bool NRF24::setTransmitAddress(uint8_t* address, uint8_t len)
{
  if (_address_width!=len)
    return false;

  // Set both TX_ADDR and RX_ADDR_P0 for auto-ack with Enhanced shockwave.
  spiBurstWriteRegister(NRF24_REG_0A_RX_ADDR_P0, address, len);
  spiBurstWriteRegister(NRF24_REG_10_TX_ADDR, address, len);
  return true;
}

bool NRF24::setStaticPayloadMode(uint8_t size)
{
  // Disable DPL
  spiWriteRegister(NRF24_REG_01_EN_AA, 0x0);
  spiWriteRegister(NRF24_REG_1C_DYNPD, 0x0);
  spiWriteRegister(NRF24_REG_1D_FEATURE, NRF24_EN_DYN_ACK);

  spiWriteRegister(NRF24_REG_11_RX_PW_P0, size);
  spiWriteRegister(NRF24_REG_12_RX_PW_P1, size);
  return true;
}

bool NRF24::setDynamicPayloadMode() {
  // DPL mode requires auto-ack.
  spiWriteRegister(NRF24_REG_01_EN_AA, 0x3F/*NRF24_ENAA_ALL*/);
  
  // Enable dynamic payload length on all pipes
  spiWriteRegister(NRF24_REG_1C_DYNPD, 0x3F /*NRF24_DPL_ALL*/);
  
  // Enable dynamic payload length, disable payload-with-ack, enable noack
  spiWriteRegister(NRF24_REG_1D_FEATURE, NRF24_EN_DPL | NRF24_EN_DYN_ACK);  
  return true;
}

boolean NRF24::setRF(uint8_t data_rate, uint8_t power)
{    
    uint8_t value = (power << 1) & NRF24_PWR;
    // Ugly mapping of data rates to noncontiguous 2 bits:
    if (data_rate == NRF24DataRate250kbps)
	value |= NRF24_RF_DR_LOW;
    else if (data_rate == NRF24DataRate2Mbps)
	value |= NRF24_RF_DR_HIGH;
    // else NRF24DataRate1Mbps, 00
    spiWriteRegister(NRF24_REG_06_RF_SETUP, value);

    if (data_rate == NRF24DataRate250kbps)
	spiWriteRegister(NRF24_REG_04_SETUP_RETR, 0x43); // 1250usecs, 3 retries
    else
	spiWriteRegister(NRF24_REG_04_SETUP_RETR, 0x03); // 250us, 3 retries
	
    return true;
}

boolean NRF24::powerDown()
{
  if (_mode != POWER_DOWN) {
    spiWriteRegister(NRF24_REG_00_CONFIG, _configuration);
    digitalWrite(_chipEnablePin, LOW);
    _mode = POWER_DOWN;
  }
  return true;
}

boolean NRF24::powerUpRx()
{
  if (_mode != RX) {
    boolean status = spiWriteRegister(NRF24_REG_00_CONFIG, _configuration | NRF24_PWR_UP | NRF24_PRIM_RX);

    // When switching to RX mode, P0 is set as broadcast_address.
    if(_broadcast_address[0])
      spiBurstWriteRegister(NRF24_REG_0A_RX_ADDR_P0, _broadcast_address, _address_width);

    digitalWrite(_chipEnablePin, HIGH);
    _mode = RX;
    return status;
  }
  return true;
}

boolean NRF24::powerUpTx()
{
  if (_mode != TX) {
    // Its the pulse high that puts us into TX mode
    digitalWrite(_chipEnablePin, LOW);
    boolean status = spiWriteRegister(NRF24_REG_00_CONFIG, _configuration | NRF24_PWR_UP);
    digitalWrite(_chipEnablePin, HIGH);
    _mode = TX;
    return status;
  }
  return true;
}

bool NRF24::sendBlocking(uint8_t* data, uint8_t len) {
  powerUpTx();
  spiBurstWrite(NRF24_COMMAND_W_TX_PAYLOAD, data, len);
  // Radio will return to Standby II mode after transmission is complete
  return waitPacketSent();
}

bool NRF24::sendNoAck(uint8_t* data, uint8_t len)
{
    uint8_t status;
    // To achieve low latency, this mode does not do additional checking
    // powerUpTx() should be called before calling this.
    //powerUpTx();
    spiBurstWrite(NRF24_COMMAND_W_TX_PAYLOAD_NOACK, data, len);
    // Radio will return to Standby II mode after transmission is complete
    return true;
}

boolean NRF24::waitPacketSent()
{
    // If we are currently in receive mode, then there is no packet to wait for
    if (_mode != TX)
      return false;

    // Wait for either the Data Sent or Max ReTries flag, signalling the 
    // end of transmission
    uint8_t status;
    uint32_t start = millis();
    while (!((status = statusRead()) & (NRF24_TX_DS | NRF24_MAX_RT)) && millis()-start < 4)
	;

    // Must clear NRF24_MAX_RT if it is set, else no further comm
    spiWriteRegister(NRF24_REG_07_STATUS, NRF24_TX_DS | NRF24_MAX_RT);
    if (status & NRF24_MAX_RT)
	flushTx();
    // Return true if data sent, false if MAX_RT
    return status & NRF24_TX_DS;
}

boolean NRF24::isSending()
{
    return (_mode == TX) &&
      !(spiReadRegister(NRF24_REG_00_CONFIG) & NRF24_PRIM_RX) && !(statusRead() & (NRF24_TX_DS | NRF24_MAX_RT));
}

boolean NRF24::printRegisters()
{
    uint8_t registers[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0d, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1c, 0x1d};

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
	Serial.print(i, HEX);
	Serial.print(": ");
	Serial.println(spiReadRegister(i), HEX);
    }
    return true;
}

uint8_t NRF24::available()
{
    if (_mode != RX)
      return 0;

    if (spiReadRegister(NRF24_REG_17_FIFO_STATUS) & NRF24_RX_EMPTY)
	return 0;
    // Manual says that messages > 32 octets should be discarded
    uint8_t len = spiRead(NRF24_COMMAND_R_RX_PL_WID);
    if (len > 32) {
	flushRx();
	return 0;
    }
    return len;
}

void NRF24::waitAvailable()
{
    powerUpRx();
    while (!available())
	;
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool NRF24::waitAvailableTimeout(uint16_t timeout)
{
    powerUpRx();
    uint32_t starttime = millis();
    while ((millis() - starttime) < timeout) {
        if (available())
           return true;
    }
    // Clear the possible interrupt flags.
    spiWriteRegister(NRF24_REG_07_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
    return false;
}

boolean NRF24::recv(uint8_t* buf, uint8_t* len)
{
    // Clear read interrupt
    spiWriteRegister(NRF24_REG_07_STATUS, NRF24_RX_DR);

    // 0 microsecs @ 8MHz SPI clock
    if (!available())
	return false;
    // 32 microsecs (if immediately available)
    *len = spiRead(NRF24_COMMAND_R_RX_PL_WID);
    // 44 microsecs
    spiBurstRead(NRF24_COMMAND_R_RX_PAYLOAD, buf, *len);
    // 140 microsecs (32 octet payload)

    return true;
}

uint8_t NRF24::waitAndRecv(uint8_t* buf, uint16_t timeout) {
  // To achieve low latency, this mode does not do additional checking
  // powerUpRx() should be called before calling this.
  //powerUpRx();

  timeout = timeout<<6;
  while (--timeout) {
    uint8_t len = available();

    if (len) {
      // Clear read interrupt
      //spiWriteRegister(NRF24_REG_07_STATUS, NRF24_RX_DR);
      spiBurstRead(NRF24_COMMAND_R_RX_PAYLOAD, buf, len);
      return len;
    }
  }
  return 0;
}
