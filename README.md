# CTC (Cloud Type CirrusLogic)
- AVS (Amazon Voice Service) and audio pre-processing DSP with sensory voice trigger.
- GVA (Google Voice Assistant) and audio pre-processing DSP with sensory voice trigger.
- Based the ESP-Voice-Assistant SDK.

## Overview
- Espressif's AVS & GVA app. project based on [esp-va-sdk](https://github.com/espressif/esp-va-sdk)
- Merge H/W audio pre-processor firmware (Cirrus Logic CS48L32 with Sensory trigger).
- Follow the [Getting Started Guide](https://github.com/luvinland/ctc-esp-va-sdk/blob/master/README-Getting-Started.md) to clone the required repositories and to compile and flash the firmware.

## Block diagram
![Block diagram](https://user-images.githubusercontent.com/26864945/72865558-aa5f3900-3d1b-11ea-8b92-afdc9a0a47fd.png)


## Hardware
* WiFi & BT SoC : [Espressif ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)
* Audio pre-processing DSP : [Cirrus Logic CS48L32](https://www.cirrus.com/products/cs48l32/)
