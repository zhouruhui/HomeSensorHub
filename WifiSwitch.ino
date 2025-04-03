/***************************************************************
 *  ESP32 + MQTT + Home Assistant Discovery + WS281x + OTA 示例（低功耗优化版）
 ***************************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include "driver/rtc_io.h"

//--------------------- 用户需根据实际情况修改的宏定义 ---------------------//
#define MQTT_MAX_PACKET_SIZE 512

#define OTA_HOST        "SS_Device"
#define OTA_PASS        "summer"

#define WIFI_SSID       "HUAWEI-G100EU"       // WiFi SSID
#define WIFI_PASSWORD   "Pp@123456"           // WiFi 密码

#define MQTT_HOST       "192.168.3.134"       // MQTT 代理地址
#define MQTT_PORT       1883                  // MQTT 代理端口
#define MQTT_USER       "pimqtt"              // MQTT 用户名
#define MQTT_PASSWORD   "Pp@12345678*"        // MQTT 密码

#define BATTERY_CALIBRATION_FACTOR 3.1f      // 电池电压校准系数

//--------------------- 时间常量配置 ---------------------//
const unsigned long 
  SWITCH_INDICATOR_DURATION = 1000,   // 开关指示灯持续时间（毫秒）
  WIFI_ERROR_INTERVAL = 5000,         // WiFi错误指示灯间隔（毫秒）
  MQTT_ERROR_INTERVAL = 5000,         // MQTT错误指示灯间隔（毫秒）
  BATTERY_READ_INTERVAL = 300000;     // 电池电压读取间隔（毫秒）

// GPIO配置
const int SWITCH_PINS[6] = { 27, 26, 21, 19, 18, 25 };
#define BDLED_PIN      15
#define LED_PIN        16  
#define NUM_LEDS       7
#define BATTERY_PIN    34  

//--------------------- 全局变量与对象 ---------------------//
String switchUniqueIDs[6];
String lightUniqueIDs[6];
String batterySensorUniqueID;
String batterySensorStateTopic;

WiFiClient   espClient;
PubSubClient mqttClient(espClient);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

bool switchStates[6] = {false};
bool lightStates[6]  = {false};
float currentBatteryVoltage = 0.0;

// 低功耗相关变量
volatile bool switchCheckNeeded = false;
unsigned long lastBatteryReadTime = 0;

// 指示灯控制
bool indicatorActive = false;
unsigned long indicatorStartTime = 0;

// 错误指示灯
bool wifiErrorActive = false;
unsigned long wifiPreviousMillis = 0;
bool wifiLedState = false;
bool mqttErrorActive = false;
unsigned long mqttPreviousMillis = 0;
bool mqttLedState = false;

//--------------------- 函数声明 ---------------------//
void setupWiFi();
bool reconnectMQTT();
void publishConfigToHomeAssistant();
void callback(char* topic, byte* payload, unsigned int length);
float readBatteryVoltage();
void setLightState(int index, bool state, bool fromSwitch);
String getDeviceID();
String getClientID();
void publishInitialStates();
void handleIndicator();
void handleErrors();
void IRAM_ATTR handleSwitchInterrupt();

//--------------------------------------------------//
//                    setup()
//--------------------------------------------------//
void setup() {
  btStop();  // 关闭蓝牙
  setCpuFrequencyMhz(80);  // 设置CPU频率为80MHz
  
  // 配置GPIO唤醒源
  for(int i = 0; i < 6; i++){
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SWITCH_PINS[i]), handleSwitchInterrupt, CHANGE);
  }

  // 初始化外设
  Serial.begin(115200);
  strip.begin();
  strip.show();
  pinMode(BDLED_PIN, OUTPUT);
  digitalWrite(BDLED_PIN, LOW);

  // 电源管理配置
  esp_sleep_enable_wifi_wakeup();
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);

  // 网络连接
  setupWiFi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE);

  // 生成设备ID
  String deviceID = getDeviceID();
  for(int i=0; i<6; i++){
    switchUniqueIDs[i] = deviceID + "_switch_" + String(i+1);
    lightUniqueIDs[i] = deviceID + "_light_" + String(i+1);
  }
  batterySensorUniqueID = deviceID + "_battery_voltage";
  batterySensorStateTopic = "homeassistant/sensor/" + batterySensorUniqueID + "/state";

  // OTA配置
  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.begin();
}

//--------------------------------------------------//
//                    loop()
//--------------------------------------------------//
void loop() {
  ArduinoOTA.handle();
  
  // 网络维护
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  // 开关状态检测
  if (switchCheckNeeded) {
    switchCheckNeeded = false;
    for(int i = 0; i < 6; i++){
      bool currentState = (digitalRead(SWITCH_PINS[i]) == HIGH);
      if (currentState != switchStates[i]) {
        switchStates[i] = currentState;
        String stateTopic = "homeassistant/binary_sensor/" + switchUniqueIDs[i] + "/state";
        mqttClient.publish(stateTopic.c_str(), switchStates[i] ? "ON" : "OFF", true);
        updateBatteryIndicator();
      }
    }
  }

  // 指示灯管理
  handleIndicator();
  handleErrors();

  // 动态调整唤醒时间
  unsigned long currentTime = millis();
  unsigned long idleTime = currentTime - lastBatteryReadTime;

  if (idleTime >= BATTERY_READ_INTERVAL) {
    // 无操作时，进入深度睡眠，延长唤醒时间
    esp_sleep_enable_timer_wakeup(10000000); // 10秒唤醒
  } else {
    // 有操作时，快速检测，缩短唤醒时间
    esp_sleep_enable_timer_wakeup(1000000); // 1秒唤醒
  }

  // 进入低功耗模式
  esp_light_sleep_start();
  
  // 唤醒后检查连接
  if(WiFi.status() != WL_CONNECTED) setupWiFi();
}

//--------------------------------------------------//
//                中断处理函数
//--------------------------------------------------//
void IRAM_ATTR handleSwitchInterrupt() {
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last > 50000) { // 50ms
    switchCheckNeeded = true;
  }
  last = now;
}

//--------------------------------------------------//
//                电池电压处理
//--------------------------------------------------//
void updateBatteryIndicator() {
  if (millis() - lastBatteryReadTime >= BATTERY_READ_INTERVAL) {
    currentBatteryVoltage = readBatteryVoltage();
    String payload = String(currentBatteryVoltage, 2);
    mqttClient.publish(batterySensorStateTopic.c_str(), payload.c_str(), true);
    lastBatteryReadTime = millis();
  }

  // 指示灯逻辑
  uint32_t color = strip.Color(255, 0, 0); // 默认红色
  if(currentBatteryVoltage > 8.4) color = strip.Color(0, 255, 0);
  else if(currentBatteryVoltage >= 7.6) color = strip.Color(0, 0, 255);
  
  strip.setPixelColor(6, color);
  strip.setBrightness(5); // 5%亮度
  strip.show();
  indicatorStartTime = millis();
  indicatorActive = true;
}

// --------------------------------------------------//
//               WiFi 连接函数
// --------------------------------------------------//
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("正在连接到: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startTime = millis(); // 记录开始时间
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startTime >= 60000 || retryCount >= 3) { // 超过1分钟或重试3次
      Serial.println("\n连接WiFi超时，正在重启...");
      ESP.restart(); // 重启设备
    }
    retryCount++;
  }
  Serial.println("");
  Serial.print("WiFi 已连接，IP地址: ");
  Serial.println(WiFi.localIP());
}

// --------------------------------------------------//
//          MQTT 重连函数 (非阻塞)
// --------------------------------------------------//
bool reconnectMQTT() {
  // 如果已经连接，直接返回
  if (mqttClient.connected()) {
    return true;
  }

  // 生成唯一的 MQTT 客户端 ID
  String clientID = getClientID();

  // 尝试连接
  Serial.print("连接到 MQTT...");
  if (mqttClient.connect(clientID.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("已连接");
    // 订阅6个灯的命令主题
    for(int i=0; i<6; i++){
      String cmdTopic = "homeassistant/light/" + lightUniqueIDs[i] + "/set";
      mqttClient.subscribe(cmdTopic.c_str());
      Serial.printf("已订阅主题: %s\n", cmdTopic.c_str());
    }
    // 发布初始状态
    publishInitialStates();
    // 发布配置消息
    publishConfigToHomeAssistant(); // 确保每次重连后重新发布配置
    // 更新错误状态
    wifiErrorActive = false;
    mqttErrorActive = false;
    return true;
  } else {
    Serial.print("连接失败, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" 尝试稍后重连...");
    // 根据MQTT连接状态，设置相应的设备状态
    if(WiFi.status() == WL_CONNECTED){
      mqttErrorActive = true; // 如果WiFi连接正常，但MQTT连接失败
    } else {
      wifiErrorActive = true; // 如果WiFi连接也有问题
      setupWiFi(); // WiFi重连
    }
    return false;
  }
}

// --------------------------------------------------//
//        发布 Home Assistant 自发现配置
// --------------------------------------------------//
void publishConfigToHomeAssistant() {
  String deviceID = getDeviceID();
  // 构建 device 信息 JSON 字符串
  String deviceInfo = "{\"identifiers\": [\"" + deviceID + "\"], \"manufacturer\": \"Custom\", \"model\": \"ESP32\", \"name\": \"" + deviceID + " Device\"}";

  // 配置 binary_sensor (开关 S1~S6)
  for (int i = 0; i < 6; i++) {
    String configTopic = "homeassistant/binary_sensor/" + switchUniqueIDs[i] + "/config";

    // 将JSON消息写成一行
    String simplePayload = "{\"name\": \"SS Switch " + String(i + 1) + "\", \"state_topic\": \"homeassistant/binary_sensor/" + switchUniqueIDs[i] + "/state\", \"payload_on\": \"ON\", \"payload_off\": \"OFF\", \"unique_id\": \"" + switchUniqueIDs[i] + "\", \"device\": " + deviceInfo + "}";

    Serial.printf("准备发送 Binary Sensor 配置到主题: %s\n", configTopic.c_str());
    Serial.printf("JSON 内容: %s\n", simplePayload.c_str());

    bool result = mqttClient.publish(configTopic.c_str(), simplePayload.c_str(), true);
    if (result) {
      Serial.printf("发送成功: %s\n", configTopic.c_str());
    } else {
      Serial.printf("发布主题 %s 失败\n", configTopic.c_str());
    }
  }

  // 配置 light (L1~L6)
  for (int i = 0; i < 6; i++) {
    String configTopic = "homeassistant/light/" + lightUniqueIDs[i] + "/config";

    // 将JSON消息写成一行
    String simplePayload = "{\"name\": \"SS Light " + String(i + 1) + "\", \"command_topic\": \"homeassistant/light/" + lightUniqueIDs[i] + "/set\", \"state_topic\": \"homeassistant/light/" + lightUniqueIDs[i] + "/state\", \"payload_on\": \"ON\", \"payload_off\": \"OFF\", \"unique_id\": \"" + lightUniqueIDs[i] + "\", \"device\": " + deviceInfo + "}";

    Serial.printf("准备发送 Light 配置到主题: %s\n", configTopic.c_str());
    Serial.printf("JSON 内容: %s\n", simplePayload.c_str());

    bool result = mqttClient.publish(configTopic.c_str(), simplePayload.c_str(), true);
    if (result) {
      Serial.printf("发送成功: %s\n", configTopic.c_str());
    } else {
      Serial.printf("发布主题 %s 失败\n", configTopic.c_str());
    }
  }

  // 配置 sensor (电池电压传感器)
  String batteryConfigTopic = "homeassistant/sensor/" + batterySensorUniqueID + "/config";

  // 将JSON消息写成一行
  String batteryPayload = "{\"name\": \"SS Battery Voltage\", \"state_topic\": \"" + batterySensorStateTopic + "\", \"unit_of_measurement\": \"V\", \"device_class\": \"voltage\", \"unique_id\": \"" + batterySensorUniqueID + "\", \"device\": " + deviceInfo + "}";

  Serial.printf("准备发送 Battery Sensor 配置到主题: %s\n", batteryConfigTopic.c_str());
  Serial.printf("JSON 内容: %s\n", batteryPayload.c_str());

  bool result = mqttClient.publish(batteryConfigTopic.c_str(), batteryPayload.c_str(), true);
  if (result) {
    Serial.printf("发送成功: %s\n", batteryConfigTopic.c_str());
  } else {
    Serial.printf("发布主题 %s 失败\n", batteryConfigTopic.c_str());
  }
}

// --------------------------------------------------//
//        发布初始状态消息
// --------------------------------------------------//
void publishInitialStates(){
  // 发布开关的初始状态
  for(int i=0; i<6; i++){
    String stateTopic = "homeassistant/binary_sensor/" + switchUniqueIDs[i] + "/state";
    String payload = switchStates[i] ? "ON" : "OFF";
    mqttClient.publish(stateTopic.c_str(), payload.c_str(), true);
    Serial.printf("已发布初始 Binary Sensor 状态: %s -> %s\n", stateTopic.c_str(), payload.c_str());
  }

  // 发布电池电压的初始状态
  currentBatteryVoltage = readBatteryVoltage();
  String batteryPayloadStr = String(currentBatteryVoltage, 2); // 保留2位小数
  mqttClient.publish(batterySensorStateTopic.c_str(), batteryPayloadStr.c_str(), true);
  Serial.printf("已发布初始电池电压状态: %s -> %.2f V\n", batterySensorStateTopic.c_str(), currentBatteryVoltage);
}

// --------------------------------------------------//
//   MQTT 回调函数：接收灯命令，控制灯带(L1~L6)
// --------------------------------------------------//
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  Serial.printf("接收到消息，主题: %s，内容: %s\n", topic, msg.c_str());

  // 找到是哪个灯的命令主题
  for(int i=0; i<6; i++){
    String cmdTopic = "homeassistant/light/" + lightUniqueIDs[i] + "/set";
    if(String(topic) == cmdTopic){
      bool newState = false;
      if(msg == "ON"){
        newState = true;
      } else if(msg == "OFF"){
        newState = false;
      } else {
        // 处理其他可能的命令，例如调光或颜色变化（如果需要）
        Serial.printf("[WARNING] 未知命令: %s\n", msg.c_str());
        break;
      }
      setLightState(i, newState, false); // false 表示来自 MQTT 命令
      break;
    }
  }
}

// --------------------------------------------------//
//   读取电池电压 (根据实际电路自行调整)
// --------------------------------------------------//
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  // ESP32 ADC 默认分辨率为 12bit (0-4095)
  float voltage = (3.3f * raw) / 4095.0f * BATTERY_CALIBRATION_FACTOR;
  return voltage;
}

// --------------------------------------------------//
//   设置指定LED的状态，并同步发布状态
// --------------------------------------------------//
void setLightState(int index, bool state, bool fromSwitch) {
  if(lightStates[index] != state){
    lightStates[index] = state;
    if(state){
      // 白色亮灯
      strip.setPixelColor(index, strip.Color(255, 255, 255));
      strip.setBrightness(5);  // 5%亮度
    } else {
      // 灭
      strip.setPixelColor(index, 0);
    }
    strip.show();

    // 同步发布 state
    String stateTopic = "homeassistant/light/" + lightUniqueIDs[index] + "/state";
    String payload = state ? "ON" : "OFF";
    bool success = mqttClient.publish(stateTopic.c_str(), payload.c_str(), true);
    if (success) {
      Serial.printf("[LIGHT %d] %s命令: %s\n", index+1, fromSwitch ? "来自开关" : "来自MQTT", payload.c_str());
    } else {
      Serial.printf("[ERROR] 发布主题 %s 失败\n", stateTopic.c_str());
    }
  }
}

// --------------------------------------------------//
//   生成唯一的设备ID（基于MAC地址）
// --------------------------------------------------//
String getDeviceID(){
  String mac = WiFi.macAddress();
  String mac_no_colon = "";
  for(int i=0; i<mac.length(); i++){
    if(mac[i] != ':') mac_no_colon += mac[i];
  }
  // 使用MAC地址的后4位作为设备ID的一部分
  return "SS_" + mac_no_colon.substring(mac_no_colon.length() - 4);
}

// --------------------------------------------------//
//   生成唯一的 MQTT 客户端 ID
// --------------------------------------------------//
String getClientID(){
  String deviceID = getDeviceID();
  return deviceID + "_Client";
}

// --------------------------------------------------//
//   处理指示灯逻辑（开关状态变化时显示电池指示）
// --------------------------------------------------//
void handleIndicator(){
  if(indicatorActive){
    if(millis() - indicatorStartTime >= SWITCH_INDICATOR_DURATION){
      // 关闭L7
      strip.setPixelColor(6, 0);
      strip.show();
      indicatorActive = false;
    }
  }
}

// --------------------------------------------------//
//   处理错误状态下的指示灯闪烁
// --------------------------------------------------//
void handleErrors(){
  unsigned long currentMillis = millis();

  // 优先检查指示灯是否处于活动状态
  if(indicatorActive){
    // 如果指示灯正在活动，不处理错误指示灯
    return;
  }

  if(wifiErrorActive){
    // WiFi错误，LED闪烁红色，每2秒切换一次
    if(currentMillis - wifiPreviousMillis >= WIFI_ERROR_INTERVAL){
      wifiPreviousMillis = currentMillis;
      wifiLedState = !wifiLedState;
      if(wifiLedState){
        strip.setPixelColor(6, strip.Color(255, 0, 0)); // 红色
        strip.setBrightness(50);  // 50%亮度
      }
      else{
        strip.setPixelColor(6, 0); // 熄灭
      }
      strip.show();
    }
  }
  else if(mqttErrorActive){
    // MQTT错误，LED闪烁蓝色，每2秒切换一次
    if(currentMillis - mqttPreviousMillis >= MQTT_ERROR_INTERVAL){
      mqttPreviousMillis = currentMillis;
      mqttLedState = !mqttLedState;
      if(mqttLedState){
        strip.setPixelColor(6, strip.Color(0, 0, 255)); // 蓝色
        strip.setBrightness(50);  // 50%亮度
      }
      else{
        strip.setPixelColor(6, 0); // 熄灭
      }
      strip.show();
    }
  }
  else{
    // 如果没有错误，确保LED关闭
    strip.setPixelColor(6, 0);
    strip.show();
  }
}