# Justfile for ESP32 UWB Projects
# Run commands from the root project directory.

# Default port and baud rate (can be overridden if needed)
default_port := "/dev/ttyUSB1"
default_baud := "115200"

# --- Anchor Calibrate --- 

anchor-cal-upload:
    #!/usr/bin/env bash
    echo "Uploading Anchor Calibrate firmware..."
    cd src/anchor_calibrate && pio run -t upload

anchor-cal-monitor:
    #!/usr/bin/env bash
    echo "Opening Serial Monitor for Anchor Calibrate..."
    cd src/anchor_calibrate && pio device monitor -p {{default_port}} -b {{default_baud}}

# --- Setup Anchor --- 

setup-anchor-upload:
    #!/usr/bin/env bash
    echo "Uploading Setup Anchor firmware..."
    cd src/setup_anchor && pio run -t upload

setup-anchor-monitor:
    #!/usr/bin/env bash
    echo "Opening Serial Monitor for Setup Anchor..."
    cd src/setup_anchor && pio device monitor -p {{default_port}} -b {{default_baud}}

# --- Setup Tag --- 

setup-tag-upload:
    #!/usr/bin/env bash
    echo "Uploading Setup Tag firmware..."
    cd src/setup_tag && pio run -t upload

setup-tag-monitor:
    #!/usr/bin/env bash
    echo "Opening Serial Monitor for Setup Tag..."
    cd src/setup_tag && pio device monitor -p {{default_port}} -b {{default_baud}}

# --- Tag 2D Website --- 

tag-website-upload:
    #!/usr/bin/env bash
    echo "Uploading Tag 2D Website firmware..."
    cd src/tag_2d_website && pio run -t upload

tag-website-monitor:
    #!/usr/bin/env bash
    echo "Opening Serial Monitor for Tag 2D Website..."
    cd src/tag_2d_website && pio device monitor -p {{default_port}} -b {{default_baud}}

# --- General Commands --- 

list:
    @echo "Available commands:"
    @echo "  just anchor-cal-upload    - Upload Anchor Calibrate firmware"
    @echo "  just anchor-cal-monitor   - Monitor Anchor Calibrate serial output"
    @echo "  just setup-anchor-upload  - Upload Setup Anchor firmware"
    @echo "  just setup-anchor-monitor - Monitor Setup Anchor serial output"
    @echo "  just setup-tag-upload     - Upload Setup Tag firmware"
    @echo "  just setup-tag-monitor    - Monitor Setup Tag serial output"
    @echo "  just tag-website-upload   - Upload Tag 2D Website firmware"
    @echo "  just tag-website-monitor  - Monitor Tag 2D Website serial output"
    @echo "  just list                 - Show this help message" 