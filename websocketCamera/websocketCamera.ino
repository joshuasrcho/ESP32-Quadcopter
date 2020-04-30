// Wifi and websocket stuff
#include <WiFi.h>
#include <ArduinoWebsockets.h>

// Camera stuff
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "camera_pins.h"

#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

// sensitive network info
#include "secrets.h"

camera_fb_t * fb = NULL; // camera capture data
bool wsConnected;

using namespace websockets;

WebsocketsClient client;

void onMessageCallback(WebsocketsMessage message) {
    ;
    //Serial.print("Got Message: ");
    //Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connnection Opened");
        wsConnected = true;
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connnection Closed");
        wsConnected = false;
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.println("Got a Pong!");
    }
}

void connectWS(){
  // Connect to server
  client.connect(SERVER_NAME);

  Serial.println("Connected to Wifi, Connecting to server.");
  // try to connect to Websockets server
  bool connected = client.connect(SERVER_NAME);
  if(connected) {
      Serial.println("Connected!");
  } else {
      Serial.println("Not Connected!");
  }

  // Setup Callbacks
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);
}

void connectWifi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
}

void initCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void captureImage(){
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
}

void postImage(){
  // Publish picture
  uint8_t * jpg_buf = fb->buf;
  size_t length = fb->len;
  Serial.println("buffer is " + String(length) + " bytes");
  char * part_buf[64]; 
  
  client.sendBinary((const char*)jpg_buf,length);

  if(fb){
    esp_camera_fb_return(fb);
    fb = NULL;
    jpg_buf = NULL;
  } 
  else if(jpg_buf){
    free(jpg_buf);
    jpg_buf = NULL;
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  connectWifi();
  connectWS();
  initCamera();
  }

void loop() {
  if(WiFi.status()== WL_CONNECTED){
    if (wsConnected){
      captureImage();
      postImage(); 
    }
    else{
      connectWS();
    }
  }
  else {
    Serial.println("WiFi Disconnected");
    connectWifi();
    connectWS();
  }
  client.poll();
}
