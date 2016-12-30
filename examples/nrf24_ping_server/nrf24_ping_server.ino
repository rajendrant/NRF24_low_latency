#include "NRF24_low_latency.h"

NRF24 nrf24;

void setup() 
{
  Serial.begin(115200);
  if (!nrf24.init())
    Serial.println("NRF24 init failed");
 if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setThisAddress((uint8_t*)"s-1", 3))
    Serial.println("setThisAddress failed");
  if (!nrf24.setTransmitAddress((uint8_t*)"s-1", 3))
   Serial.println("setTransmitAddress failed");
  if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPowerm18dBm))
    Serial.println("setRF failed");

  // Dynamic payload mode.
  if (!nrf24.setDynamicPayloadMode())
    Serial.println("setDynamicPayloadMode failed");

  Serial.println("initialised");
}

void loop()
{
  uint32_t data;
  uint8_t len;

  for(int i=0; i<1000; i++) {
    nrf24.powerUpRx();
    len = nrf24.waitAndRecv((uint8_t*)&data, 600);
    if (!len)
      continue;

    // At this time, the other side is in TX mode, the below delay
    // ensures the other side could switch to RX mode, before this
    // board starts sending packet.
    delayMicroseconds(120);

    // Send the same data back
    if (!nrf24.sendBlocking((uint8_t*)&data, sizeof(data)))
       ;//Serial.println("send failed");
  }
}

