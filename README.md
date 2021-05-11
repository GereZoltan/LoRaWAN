# LoRaWAN
Possibly complete LoRaWAN implementation in micropython with SX1262

## Introduction
Aim of the project is to deliver a complete solution for connecting an IoT device to LoRaWAN network.
Tested on Raspberry Pi Pico with RF-LAMBDA62, but it should work on other microcontrollers (after adjusting the GPIO numbering).
The server is [ChirpStack open-source LoRaWAN](https://www.chirpstack.io)

This project is built upon these github projects:
- [micropySX126X](https://github.com/ehong-tl/micropySX126X) for the SX126X driver
- [LoRaWAN](https://github.com/jeroennijhof/LoRaWAN) for the LoRaWAN packet handling
- [micropython-aes](https://github.com/piaca/micropython-aes) for the AES computation



