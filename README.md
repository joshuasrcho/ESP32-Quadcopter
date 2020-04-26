# ESP-Quadcopter
install ESP32 from board manager


### mqttCamera
- Old attempt at streaming images to an AWS IoT core via MQTT. Doesn't work.
- From library manager, install MQTT by Joel Gaehwiler
- Also install Arduino Json by Benoit Blanchon
- Follow https://aws.amazon.com/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/

### ESP32_MPU6050_test
- test IMU connection with ESP32
- From library manager, install MPU6050 by Electronic Cats

### MotorTest
- From library manager, install ServoESP32
- ESCs take 50Hz PWM signal. Lowest pulse width is 1ms, highest is 2ms.

### httpCamera
- HTTP client that sends captured images to the web server
- install HTTPClient.h
- create and include secrets.h that contains WiFi information and server IP address.

### QuadcopterWebServer
- Main web server on an Amazon EC2 instance
- Uses Tornado Python framework
