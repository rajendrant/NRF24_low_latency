#include "NRF24_low_latency.h"

NRF24 nrf24;

void setup() 
{
  Serial.begin(115200);
  if (!nrf24.init())
    Serial.println("NRF24 init failed");
  if (!nrf24.setChannel(76))
    Serial.println("setChannel failed");
  if (!nrf24.setThisAddress((uint8_t*)"s-1", 3))
    Serial.println("setThisAddress failed");
  if (!nrf24.setTransmitAddress((uint8_t*)"s-1", 3))
   Serial.println("setTransmitAddress failed");
  if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPowerm18dBm))
    Serial.println("setRF failed");
  
  Serial.println("initialised");
}

void loop()
{
  uint8_t data;
  uint8_t len;
  uint16_t recv_count = 0;

  // Static payload mode is needed to send packets with NO ACK.
  nrf24.setStaticPayloadMode(1);

  // To achieve low latency, powerup RX should be called separately.
  nrf24.powerUpRx();

  while((len = nrf24.waitAndRecv(&data, 300))) {
    recv_count++;
    //Serial.println(data);
  }

  if (recv_count) {
    Serial.print("Received packets: ");
    Serial.println(recv_count);

    // Dynamic payload mode for sending the result in ACK mode.
    nrf24.setDynamicPayloadMode();

    if (!nrf24.sendBlocking((uint8_t*)&recv_count, 2))
       Serial.println("send failed");
  }
}

