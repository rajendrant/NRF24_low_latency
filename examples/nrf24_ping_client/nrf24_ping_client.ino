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

/**
 * This example could send 1020-1040 packets per second, with an RTT
 * of 840 microseconds, in Arduino nano running at 16MHz.
 */
void loop()
{
  uint32_t pass=0, fail=0, rtt_avg=0;
  uint8_t data[32];
  uint8_t len;

  /* Loop for one second */
  for(unsigned long start_millis=millis();
      millis()-start_millis <= 1000; ) {
    unsigned long start = micros();
    if (!nrf24.sendBlocking((uint8_t*)&start, 4)) {
      //Serial.println("send failed");
      nrf24.flushTx();
      nrf24.flushRx();
      fail++;
      continue;
    }
    if (!nrf24.waitAvailableTimeout(2) && nrf24.recv(data, &len)) {
      Serial.println("recv failed");
      nrf24.flushRx();
      fail++;
      continue;
    }
    pass++;
    rtt_avg += micros() - start;

    // At this time, the other side is in TX mode, the below delay
    // ensures the other side could switch to RX mode, before this
    // board starts sending packet.
    delayMicroseconds(120);
  }
  Serial.println("Result");
  Serial.print("Pass: ");
  Serial.print(pass);
  Serial.println(" packets-per-second");
  Serial.print("Fail: ");
  Serial.print(fail);
  Serial.println(" packets-per-second");
  Serial.print("RTT: ");
  Serial.print((double)rtt_avg/pass);
  Serial.println(" microseconds-per-packet");
  Serial.println();
  delay(2000);
}

