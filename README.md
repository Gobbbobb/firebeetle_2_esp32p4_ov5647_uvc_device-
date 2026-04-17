# firebeetle_2_esp32p4_ov5647_uvc_device
Firebeetle 2 esp32p4 with ov5647 mipi csi camera uvc device with PPA

### Software Required

* `potplay` APP for PC on Windows OS
  
# Overview

This project captures RGB565 frames from the camera, processes them with PPA (scale + rotate), encodes to JPEG using hardware encoder, and streams over USB as a UVC device.

Pipeline:

Camera (1280x960 RGB565)
    ↓
PPA (scale 0.5× + rotate 90°CW → 480x640)
    ↓
JPEG encoder (quality = 85)
    ↓
USB UVC (MJPEG)

# Key Specs
Input: 1280x960 RGB565
Output: 480x640 RGB565 (rotated)
Format: MJPEG (UVC)
JPEG quality: 85
Buffers: DMA-capable, 64-byte aligned (PSRAM)

# Important Details
Cache handling is mandatory
Invalidate after ISP DMA
Flush after PPA DMA
Uses V4L2 USERPTR buffers
Double-buffered JPEG output
Blocking PPA operation for simplicity
Runs pipeline on Core 1

# Notes
Designed for ESP32-P4 with PSRAM
Relies on:
PPA (SRM mode)
HW JPEG encoder
USB UVC stack

# License
Apache 2.0 license
