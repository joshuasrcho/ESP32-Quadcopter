ESP32-based flight controller!\
https://hackaday.io/project/172293-esp32-quadcopter

### ESP-Quadcopter_main
- Must install ESP32 Board in Arduino IDE before uploading
- create a `secrets.h` file that specifies `WIFI_SSID`, `WIFI_PASSWORD`, and `SERVER_NAME`
- Install `ArduinoWebsockets` by Gil Maimon from Arduino Library Manager   
- install `MPU6050` by Electronic Cats from Arduino Library Manager
- install `ServoESP32` from Arduino Library Manager
- install `ArduinoJson` from Arduino Library Manager


### WebsocketCamera
- Arduino code that runs on AI Thinker ESP32-CAM
- Takes pictures and streams them to `\camera` websocket
- Must install ESP32 Board in Arduino IDE before uploading
- create a `secrets.h` file that specifies `WIFI_SSID`, `WIFI_PASSWORD`, and `SERVER_NAME`
- Install `ArduinoWebsockets` by Gil Maimon from Arduino Library Manager   


### QuadcopterWebServer
- server.py: Main web server script that runs inside an Amazon EC2 instance (Ubuntu 20.04)
```
pip install tornado
pip install numpy
pip install opencv-contrib-python
pip install imutils
