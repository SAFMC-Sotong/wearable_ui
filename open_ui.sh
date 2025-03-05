#!/bin/bash
./open_voice_control.sh &
node server.js &
chromium-browser --kiosk --start-fullscreen --window-size=320,240 --force-device-scale-factor=1.0 127.0.0.1:8080 &
wait