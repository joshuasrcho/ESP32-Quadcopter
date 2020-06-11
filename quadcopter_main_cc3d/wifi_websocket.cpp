#include "wifi_websocket.h"
#include "secrets.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

WebsocketsClient client;
bool wsConnected;
int controlReference[5]; // throttle, yaw, pitch, roll

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

void onMessageCallback(WebsocketsMessage message) {
    DynamicJsonDocument doc(150);
    
    // msg is a JSON object that goes like "{\"throttle\":150,\"yaw\":20,\"pitch\":25,\"roll\":25}"
    const char* msg = (message.data()).c_str();
    deserializeJson(doc,msg);
    
    controlReference[0] = doc["throttle"];
    controlReference[1] = doc["yaw"];
    controlReference[2] = doc["pitch"];
    controlReference[3] = doc["roll"];
    controlReference[4] = doc["active"];
    controlReference[5] = doc["follow"];
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

void connectWebsocket(){
  // Connect to server
  client.connect(SERVER_NAME);

  Serial.println("Connected to Wifi, Connecting to server.");
  // try to connect to Websockets server
  bool connected = client.connect(SERVER_NAME);
  if(connected) {
      Serial.println("Connected!");
      for(int i=0;i<5;i++){
        controlReference[i] = 0;
      }
  } else {
      Serial.println("Not Connected!");
  }

  // Setup Callbacks
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);
}

bool pollWebsocket(){
  if (client.available()){
    client.poll(); // Get ready to receive next message
    return true;  
  }
  else{
    return false;
  }
}

int* getControlReference(){
  return controlReference;
}
