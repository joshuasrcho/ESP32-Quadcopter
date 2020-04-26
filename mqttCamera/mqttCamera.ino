#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "soc/timer_group_struct.h" // Disable watchdog timer
#include "soc/timer_group_reg.h"    // Disable watchdog timer
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

#include "WiFi.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>

#include "secrets.h" // contains sensitive information
#include "camera_pins.h"

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

//  StaticJsonDocument<200> doc;
//  deserializeJson(doc, payload);
//  const char* message = doc["message"];
}

void publishMessage()
{
  // Take Picture with Camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Publish picture
  const char* pic_buf = (const char*)(fb->buf);
  size_t length = fb->len;
  size_t olen;

  Serial.println("buffer is " + String(length) + " bytes");
  int packet_size = 200;
  String packets = "";
  float pack = float(length) / packet_size;
  if ( float(pack / int(pack)) > 1){
    packets = String(int(pack) + 1);
  }
  else{
    packets = String(int(pack));
  }
  Serial.println("Total number of packets is: " + packets);
  
  String subs;
  int i = 0;
  sendimage:
    subs = "";
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
  if (i < packets.toInt() - 1){
    for (int j = 0; j < packet_size; j++){
      subs += pic_buf[j + i * packet_size];
    }
  }
  else{
    for (int j = 0; j < length - i * packet_size; j++){
      subs += pic_buf[j + i * packet_size];
    }
  }
  Serial.println("Packet size: " + String(subs.length()) + " of total size: " + String(length));
  
  uint16_t packetIdPubTemp = client.publish( AWS_IOT_PUBLISH_TOPIC, subs.c_str(), subs.length());

  if ( !packetIdPubTemp  ){
    Serial.println( "Sending Failed! err: " + String( packetIdPubTemp ) );
  }
  else{
    Serial.println("Packet " + String(i + 1) + " of total " + packets + " published.");
  }
  delay(5);
  i++;
  if (i < packets.toInt()){
    goto sendimage;
  }
  if ( packetIdPubTemp  ){
    Serial.println("MQTT Publish succesful");
  }

  // No delay result in no message sent.
  delay(200);
  esp_camera_fb_return(fb);
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IoT\n");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!\n");
}



void setup() {
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println();

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

  connectAWS();
  publishMessage();
}

void loop() {
  client.loop();
  delay(10000);
}
