#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi连接信息
const char* ssid = "ccddyy";
const char* password = "11111111";

// 阿里云MQTT服务器连接信息
const char* mqttHostUrl = "iot-06z00entpsexhou.mqtt.iothub.aliyuncs.com";
const int port = 1883; // 阿里云服务器提供的端口
const char* clientId = "hb8pVz05ghI.ESP32_AAA|securemode=2,signmethod=hmacsha256,timestamp=1754147148895|";
const char* username = "ESP32_AAA&hb8pVz05ghI";
const char* passwd = "f76ad9df5ec8aedfac6ecdc87aa7715a237d1521d96ec206d7e1af5fb48e09bd";

// MQTT主题
const char* subscribeTopic = "/a1WxHxxxxxx/device1/user/get";
const char* publishTopic = "/a1WxHxxxxxx/device1/user/update";
const char* humilityTopic = "/a1WxHxxxxxx/device1/Humility"; // 新增Humility主题

// 创建WiFi客户端和MQTT客户端
WiFiClient espClient;
PubSubClient client(espClient);

// 连接WiFi
void connectWiFi() {
  Serial.print("连接WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("WiFi连接成功! IP地址: " + WiFi.localIP().toString());
}

// MQTT重连函数
void reconnect() {
  while (!client.connected()) {
    Serial.print("尝试连接阿里云MQTT服务器...");
    
    // 设置MQTT连接参数
    client.setKeepAlive(60);
    
    if (client.connect(clientId, username, passwd)) {
      Serial.println("连接成功！");
      client.subscribe(subscribeTopic);
      Serial.println("已订阅主题: " + String(subscribeTopic));
      
      // 发布连接成功消息
      StaticJsonDocument<200> doc;
      doc["connected"] = "ESP32连接成功";
      char jsonBuffer[512];
      serializeJson(doc, jsonBuffer);
      client.publish(publishTopic, jsonBuffer);
    } else {
      Serial.print("连接失败，错误码: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// MQTT消息回调函数
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("收到消息: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  // 简单解析JSON消息
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (!error && doc.containsKey("command")) {
    Serial.println("收到命令: " + String(doc["command"].as<const char*>()));
    // 这里可以添加命令处理逻辑
  }
}

void setup() {
  Serial.begin(115200);
  connectWiFi();
  
  client.setServer(mqttHostUrl, port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // 每10秒发布一次Humility主题消息
  static unsigned long lastHumilityPublish = 0;
  if (millis() - lastHumilityPublish > 10000) { // 10000毫秒 = 10秒
    lastHumilityPublish = millis();
    
    // 发布消息"30"到Humility主题
    client.publish(humilityTopic, "30");
    Serial.println("已发送消息到Humility主题: 30");
  }
}













