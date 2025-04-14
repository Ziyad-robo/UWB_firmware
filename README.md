# ESP32 UWB Indoor Localization Project

This project uses ESP32 and DW1000 UWB modules for indoor positioning/localization. It includes multiple firmware options for different purposes.

## Hardware Requirements

- ESP32 development boards
- DW1000 UWB modules
- USB cable for programming

## Setup

1. Make sure PlatformIO is installed
2. Connect your ESP32 to your computer
3. The correct USB port is set in `platformio.ini` (default: `/dev/ttyUSB1`)

## Available Firmware Options

### 1. Anchor Calibration

Used to calibrate an ESP32_UWB module intended for use as a fixed anchor point. Uses binary search to find anchor antenna delay to calibrate against a known distance.

**To upload:**
```
pio run -e anchor_calibrate -t upload
```

### 2. Tag 2D Website

Sets up an ESP32 as a localization tag that works with a 2D positioning system and displays data on a website.

**To upload:**
```
pio run -e tag_2d_website -t upload
```

### 3. Setup Tag

Basic setup for an ESP32 as a UWB tag.

**To upload:**
```
pio run -e setup_tag -t upload
```

### 4. Setup Anchor

Basic setup for an ESP32 as a UWB anchor.

**To upload:**
```
pio run -e setup_anchor -t upload
```

## Troubleshooting

If you encounter upload issues:

1. Check that your ESP32 is properly connected
2. Verify the correct port in platformio.ini
3. Try pressing the RESET button on the ESP32 before uploading
4. On some boards, you may need to hold the BOOT button while starting the upload
5. If needed, change the upload_port in platformio.ini to the correct port

## Serial Monitor

To open the serial monitor after uploading:
```
pio device monitor -p /dev/ttyUSB1 -b 115200
``` 