# Wearable Interface
GUI Interface for Wearable 

Designed for a TFT screen of 320x240 running on a Raspberry Pi 

![UI on TFT Screen](images/ui_screen.jpg)

## Setup
### TFT Screen Setup

### Kiosk Mode with Chromium
```chromium-browser --kiosk --start-fullscreen --window-size=320,240 --force-device-scale-factor=1.0 127.0.0.1:8080```

Uses MJPEG streaming with HTTP endpoint for web interface

## To run:
For full running system:

```./open_ui.sh```

For testing of voice control system:

```./open_voice_control.sh```

## To start the CSI camera stream on Jetson:
```
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=320, height=240, format=NV12, framerate=30/1' ! nvvidconv flip-method=2 ! 'video/x-raw, format=I420' ! jpegenc quality=60 ! udpsink host=10.42.0.104 port=5000 sync=false
```
**Quality set to 100% will cause each packet to exceed the UDP packet size limit. The limit has not been tested but 60% works. Potentially TCP works as well.**

## For testing only:
### For testing the camera with a loopback IP on a Rasp Pi (open on UI):
```
rpicam-vid -t 0 -n --width 320 --height 240 --vflip --sharpness 2.0 --codec mjpeg --quality 100 --inline -o - |   gst-launch-1.0 fdsrc fd=0 !   jpegparse !   udpsink host=127.0.0.1 port=5000
```
With another terminal:
```
gst-launch-1.0 udpsrc port=5000 ! h264parse ! avdec_h264 ! videoconvert ! jpegenc ! multipartmux ! tcpserversink host=0.0.0.0 port=8090
```

### For testing the Jetson CSI camera stream:
With [jetson-inference](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md) library: ```video-viewer csi://0 ```

With [Nvidia Gstreamer](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/SD/Multimedia/GstreamerBasedCameraCapture.html) : ```nvgstcapture-1.0``` 

## Things to Note:

1. **Change IP address of drone where appropriate**
2. **If camera on Jetson can't stream, check the CSI cable connection. If it still doesn't work run:**
   ``` sudo service nvargus-daemon restart```
3. s

## TODO
1. ~~Enable camera streaming from Jetson to UI (look into webrtc)~~ 
2. Add actual data (battery, mode etc) from drone to UI (possibly through subscription of topic in mavros on drone to UDP to UI)
3. Tune colour and resolution of screen 
