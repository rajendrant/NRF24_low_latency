# nRF24L01+
nRF24L01+ is an inexpensive 2.4GHz transceiver. There are lot of Arduino libraries that support nRF24L01+ chips. The goal of this library is to optimize for low latency sending and receiving.

## Connections
Connecting with Arduino is similar to how its done for other nRF24L01+ libraries.

             Arduino Nano      nRF24L01+ brakout
                  3V3----------VCC   (3.3V - do not connect with 5V)
              pin D8-----------CE    (chip enable in)
           SS pin D10----------CSN   (chip select in)
          SCK pin D13----------SCK   (SPI clock in)
         MOSI pin D11----------SDI   (SPI Data in)
         MISO pin D12----------SDO   (SPI data out)
                               IRQ   (Interrupt output, not connected)
                  GND----------GND   (ground in)

## Examples
### nrf24_ping
This is a straightforward ping example, where the client board sends ACKed payloads, and the server board forwards the ACKed payloads back. This example could achieve 1000+ packets-per-second in Arduino Nano running at 8Mhz.

### nrf24_low_latency
This example shows the optimized low latency packet transmission. The client sends non-acked packets. The server just receives the packets. These packets are small and could be treated as beacon broadcast packets. This client could send 20000+ packets per seconds, out of which 14000+ packets are received by the server.

## Credits
This code is inspired from the following libraries. 
 * http://www.airspayce.com/mikem/arduino/RadioHead/
 * http://www.airspayce.com/mikem/arduino/NRF24/
 * https://github.com/TMRh20/RF24/

This code uses the following library.
 * Fast Digital I/O https://github.com/greiman/DigitalIO
