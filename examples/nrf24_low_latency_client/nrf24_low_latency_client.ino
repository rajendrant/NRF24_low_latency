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

/**
 * This example could send 20000 NO-ACK packets per second, and the
 * other side could receive 16000 of those packets.
 */
void loop()
{
  uint16_t sent_count = 0;
  uint16_t recv_count = 0;

  // Static payload mode is needed to send packets with NO ACK.
  nrf24.setStaticPayloadMode(1);
  
  // To achieve low latency, powerup TX should be called separately.
  nrf24.powerUpTx();

  /* Loop for one second */
  for(unsigned long start_millis=millis();
      millis()-start_millis < 1000; ) {
    for(uint8_t i=0; ++i < 6;) {
      if (!nrf24.sendNoAck(&i, 1)) {
        Serial.println("send failed");
        nrf24.flushTx();
        nrf24.flushRx();
      }
      sent_count++;
      delayMicroseconds(35);
    }
    if (!nrf24.waitPacketSent())
      Serial.println("waitPacketSent failed");
  }

  // Dynamic payload mode for receiving the result in ACK mode.
  nrf24.setDynamicPayloadMode();
  nrf24.powerUpRx();
  if (!nrf24.waitAndRecv((uint8_t*)&recv_count, 400))
      Serial.println("fail");

  Serial.println("Result");
  Serial.print("Sent packets: ");
  Serial.println(sent_count);
  Serial.print("Packets received by the other side: ");
  Serial.println(recv_count);
  Serial.println();
  delay(2000);
}
