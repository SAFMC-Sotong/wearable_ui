# Wearable Interface
GUI Interface for Wearable 

Designed for a TFT screen of 320x240 running on a Raspberry Pi 

![UI on TFT Screen](images/ui_screen.jpg)

## Setup
### TFT Screen Setup

### Kiosk Mode with Chromium
```chromium-browser --kiosk --start-fullscreen --window-size=320,240 --force-device-scale-factor=1.0 127.0.0.1:8080```

## To run:
For full running system:

```./open_ui.sh```

For testing of voice control system:

```./open_voice_control.sh```

## Things to Note:

1. **Change IP address of drone where appropriate**
2. 

## TODO
1. Enable camera streaming from Jetson to UI (look into webrtc)
2. Add actual data (battery, mode etc) from drone to UI (possibly through subscription of topic in mavros on drone to UDP to UI)
3. Tune colour and resolution of screen 