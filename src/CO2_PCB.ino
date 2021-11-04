
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager for WiFi and Configurations
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <SPIFFS.h>
#include <PubSubClient.h> //for mqtt
#include <max6675.h>
#include <Adafruit_GFX.h> //for oled
#include <Adafruit_SSD1306.h> //for oled
#include <CRCx.h> //https://github.com/hideakitai/CRCx
#include <ezButton.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//------------------------------------------ pin --------------------------------------------------------------
//buzzer
int BUZZER_PIN = 25;     //蜂鳴器控制腳位

//Max6675 pin
const int8_t thermoSO1 = 23; //SO1
const int8_t thermoCS1 = 22; //CS1
const int8_t thermoCLK1 = 21; //SCK1

//For Reset WiFi Button
const int btnWiFiResetPin = 32;  
//For CO2 Calibration Button
const int btnCO2CalPin = 33;  
//Senseair Sensor UART pins
const int RXD2 = 16;
const int TXD2 = 17;

const int ONBOARD_LED = 2;

//------------------------------------------ global variables --------------------------------------------------

char DeviceId[32] = "CO2_BOX";  // Device ID, SSID
char APPassword[32] = "1qaz2wsx";  // Wifi AP Password

//global config with default value
int   HeaterPWMfreq=120;
float kp=2.5;
float ki=0.06;
float kd=0.8;

//MQTT
char mqttServer[40] = "";  // MQTT伺服器位址
char mqttPort[6]  = "1883";
char mqttUserName[32] = "";  // 使用者名稱
char mqttPwd[32] = "";  // MQTT密碼

static int taskCore = 0;

//------------------------------------------ Second Configurations ----------------------------------
class Config {
    public:
        Config(){} 
        void load();
        void save();
        bool wifi_enable;
};
void Config::load() {
  if (SPIFFS.begin(true)) {
    if (SPIFFS.exists("/config2.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config2.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
     
        deserializeJson(doc, buf.get(), DeserializationOption::NestingLimit(20));
        serializeJsonPretty(doc, Serial);

        if (!doc.isNull() && doc.containsKey("wifi_enable")) {
          Serial.println("\nparsed json");
          wifi_enable = doc["wifi_enable"];
        } else {
          Serial.println("failed to load json config");
          wifi_enable = false;
          save();
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}

void Config::save() {
    DynamicJsonDocument doc(1024);
    doc["wifi_enable"]   = wifi_enable;
    File configFile = SPIFFS.open("/config2.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
}
Config config;

//------------------------------------------ NTP ----------------------------------
class NTP {
    public:
        NTP(){} 
        void init();
        void loop();
        String getTime();
    private:
        WiFiUDP ntpUDP;
        NTPClient *timeClient;
};
void NTP::init() {
  timeClient = new NTPClient(ntpUDP);
  timeClient->begin();
  timeClient->setTimeOffset(28800);//+8 hr
  timeClient->update();
}

void NTP::loop() {
}

String NTP::getTime(){
  return timeClient->getFormattedTime();
}

NTP ntp;

//------------------------------------------ OLED --------------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

class SCREEN {
    public:
        SCREEN() { 
          display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
        } 
        void init();
        void drawAPmode();
        void drawError(String text);
        void drawText(String text);
        void drawCO2(int co2);
        void drawBitmap();
        void drawCalmode();
    private:
        Adafruit_SSD1306 display;
};

void SCREEN::init() {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, OLED_RESET, true)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
}

void SCREEN::drawAPmode(){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("AP Mode"));
  display.setTextSize(1);
  char text[64];
  sprintf(&text[0], "\nSSID:%s\nPassword:%s", DeviceId, APPassword);
  display.println(text);
  display.display();
}


void SCREEN::drawError(String text){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  display.display();
}


void SCREEN::drawText(String text){
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(text);
  display.display();
}

void SCREEN::drawCO2(int co2){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  String CO2s = "CO2: " + String(co2);
  display.println(CO2s);
  if(config.wifi_enable){
    String now = ntp.getTime();
    display.println("\n");
    display.println(now);
    //display.println("WiFi Enable");
  }else{
    display.setTextSize(1);
    display.println("\n\n\n");
    display.println("WiFi Disabled");
  }
  display.display();
}

void SCREEN::drawCalmode(){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Cal CO2\n\n"));
  display.setTextSize(1);
  display.println(F("Reset to 400"));
  display.display();
}


//https://javl.github.io/image2cpp/
#define imageWidth 64
#define imageHeight 64
const unsigned char bitmap_co2_logo [] PROGMEM = {
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x0e, 0x01, 0xff, 0xff, 0xf0, 0x00, 
  0x00, 0x00, 0xff, 0xc3, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x01, 0xff, 0xf7, 0xff, 0xff, 0xf8, 0x00, 
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7e, 0x03, 0xf8, 0x0f, 0xe0, 0x3f, 0x00, 
  0x00, 0x7c, 0x01, 0xf0, 0x07, 0xc0, 0x1f, 0x00, 0x03, 0xf8, 0x00, 0xe0, 0x03, 0xc0, 0x1f, 0xc0, 
  0x07, 0xf8, 0xf0, 0xe1, 0xe1, 0x87, 0x0f, 0xf0, 0x1f, 0xf0, 0xf8, 0xc3, 0xe1, 0x87, 0x0f, 0xf8, 
  0x3f, 0xf0, 0xf8, 0xc3, 0xe1, 0xff, 0x1f, 0xfc, 0x3f, 0xf0, 0xff, 0xc3, 0xf1, 0xfe, 0x1f, 0xfc, 
  0x7f, 0xf0, 0xff, 0xc3, 0xf1, 0xfc, 0x3f, 0xfe, 0x7f, 0xf0, 0xff, 0xc3, 0xf1, 0xf8, 0x3f, 0xfe, 
  0xff, 0xf0, 0xff, 0xc3, 0xe1, 0xf0, 0x7f, 0xff, 0xff, 0xf0, 0xf8, 0xc3, 0xe1, 0xf0, 0xff, 0xff, 
  0xff, 0xf0, 0xf0, 0xe1, 0xe1, 0xe1, 0xff, 0xff, 0xff, 0xf8, 0x00, 0xe0, 0x03, 0xc0, 0x0f, 0xff, 
  0xff, 0xfc, 0x01, 0xf0, 0x07, 0x80, 0x0f, 0xff, 0xff, 0xfe, 0x03, 0xf8, 0x0f, 0x80, 0x0f, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void SCREEN::drawBitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - imageWidth ) / 2,
    (display.height() - imageHeight) / 2,
    bitmap_co2_logo, imageWidth, imageHeight, 1);
    
  display.display();
  //delay(1000);
}

SCREEN screen;

//------------------------------------------ WiFi and Configurations ----------------------------------

bool shouldSaveConfig;
void saveConfigCallback () {
  shouldSaveConfig = true;
}

class WIFI {
    public:
        WIFI(char *AP_SSID, char* AP_PASS){ 
            ap_ssid = AP_SSID;
            ap_pass = AP_PASS;
        } 
        void init();
        void reset();
        void check();
    private:
        void readConfig();
        char *ap_ssid;
        char* ap_pass;
};


void WIFI::init() {
  readConfig();
  
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  shouldSaveConfig = false;
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqttServer("mqttServer", "mqtt server", mqttServer, 40);
  WiFiManagerParameter custom_mqttPort("mqttPort", "mqtt port", mqttPort, 6);
  WiFiManagerParameter custom_mqttUserName("mqttUserName", "mqtt user name", mqttUserName, 32);
  WiFiManagerParameter custom_mqttPwd("mqttPwd", "mqtt password", mqttPwd, 32);
  WiFiManagerParameter custom_DeviceId("DeviceId", "Device ID", DeviceId, 32);
  
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&custom_mqttPort);
  wm.addParameter(&custom_mqttUserName);
  wm.addParameter(&custom_mqttPwd);
  wm.addParameter(&custom_DeviceId);
  

  bool res;
  screen.drawAPmode();
  wm.setConfigPortalTimeout(180);//seconds
  res = wm.autoConnect(ap_ssid,ap_pass); // password protected ap
  if(!res) {
      Serial.println("Failed to connect, restart");
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
      screen.drawText("WiFi connected");
  }

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DynamicJsonDocument doc(1024);
    
    doc["mqttServer"]   = custom_mqttServer.getValue();
    doc["mqttPort"]     = custom_mqttPort.getValue();
    doc["mqttUserName"] = custom_mqttUserName.getValue();
    doc["mqttPwd"]      = custom_mqttPwd.getValue();
    doc["DeviceId"]     = custom_DeviceId.getValue();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    
    serializeJsonPretty(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
    readConfig();
  }  
}

void WIFI::readConfig(){
  //clean FS, for testing
  // SPIFFS.format();
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
     
        deserializeJson(doc, buf.get(), DeserializationOption::NestingLimit(20));
        serializeJsonPretty(doc, Serial);

        if (!doc.isNull()) {
          Serial.println("\nparsed json");

          if (doc.containsKey("HeaterPWMfreq")){
            HeaterPWMfreq = atoi(doc["HeaterPWMfreq"]);
          }
          if (doc.containsKey("kp")){
            kp = atof(doc["kp"]);
          }
          if (doc.containsKey("ki")){
            ki = atof(doc["ki"]);
          }
          if (doc.containsKey("kd")){
            kd = atof(doc["kd"]);
          }
          if (doc.containsKey("mqttServer")){
            strcpy(mqttServer, doc["mqttServer"]);  
          }
          if (doc.containsKey("mqttPort")){
            strcpy(mqttPort, doc["mqttPort"]);
          }
          if (doc.containsKey("mqttUserName")){
            strcpy(mqttUserName, doc["mqttUserName"]);
          }
          if (doc.containsKey("mqttPwd")){
            strcpy(mqttPwd, doc["mqttPwd"]);
          }
          if (doc.containsKey("DeviceId")){
            strcpy(DeviceId, doc["DeviceId"]);
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}


void WIFI::check()
{
  static unsigned long samplingTime = millis();
  static int countdown=5;
  
  if(millis()-samplingTime > 5000)
  {
      samplingTime=millis();
      
      if(WiFi.status()!= WL_CONNECTED) {
        countdown--;
        if(countdown==0){
            Serial.println("Failed to reconnect");
            ESP.restart();
        }
      }else{
        countdown=5;
      }
  }
}



//------------------------------------------ MQTT --------------------------------------------------------------
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char CommandTopic[64];
char DataTopic[64];

void mqtt_init() {
    mqttClient.setServer(mqttServer, atoi(mqttPort)); 
    sprintf(&CommandTopic[0],"cmd/%s/#", DeviceId);  //"cmd/DeviceId/#";
    sprintf(&DataTopic[0],"data/%s/", DeviceId);  //"data/DeviceId/";
}

void mqtt_loop(){
  if(WiFi.status()!= WL_CONNECTED) {
    return;
  }
  if (!mqttClient.connected()) {
      int countdown=5;
      while (!mqttClient.connected()) {
          if (mqttClient.connect(DeviceId, mqttUserName, mqttPwd)) {
            Serial.println("MQTT connected");
            screen.drawText("MQTT connected");
            mqttClient.subscribe(CommandTopic);
            mqttClient.setCallback(mqttCallback);
          } else {
            screen.drawError("Failed to connect MQTT");
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);  // 等5秒之後再重試'
            //設置 timeout, 過了 25 秒仍無法連線, 就重啟 EPS32
            countdown--;
            if(countdown==0){
              Serial.println("Failed to reconnect");
              ESP.restart();
            }
          }
    }
  }
  mqttClient.loop();
}

void mqtt_publish(const char* topic, String str){
    // 宣告字元陣列
    byte arrSize = str.length() + 1;
    char msg[arrSize];
    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" message: ");
    Serial.print(str);
    Serial.print(" arrSize: ");
    Serial.println(arrSize);
    str.toCharArray(msg, arrSize); // 把String字串轉換成字元陣列格式
    if (!mqttClient.publish(topic, msg)){
      Serial.println("Faliure to publish, maybe you should check the message size: MQTT_MAX_PACKET_SIZE 128");       // 發布MQTT主題與訊息
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
}

//------------------------------------------ Push Buttons ------------------------------------------------------
#include <functional>
#define LONG_PRESS_CALLBACK_SIGNATURE std::function<void(void)> long_pressed_callback
#define SHORT_PRESS_CALLBACK_SIGNATURE std::function<void(void)> short_pressed_callback
const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds
class BUTTON {
    public:
        BUTTON(int PIN)  {pin=PIN;} 
        void init();
        void loop();
        void setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE);
        void setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE);
    private:
        ezButton *button;
        int pin;
        unsigned long pressedTime  = 0;
        unsigned long releasedTime = 0;
        bool isPressing = false;
        bool isLongDetected = false;
        LONG_PRESS_CALLBACK_SIGNATURE;
        SHORT_PRESS_CALLBACK_SIGNATURE;
};

void BUTTON::init() {
    button = new ezButton(pin);
    button->setDebounceTime(50);
}

void BUTTON::setLongPressedCallback(LONG_PRESS_CALLBACK_SIGNATURE) {
    this->long_pressed_callback = long_pressed_callback;
}

void BUTTON::setShortPressedCallback(SHORT_PRESS_CALLBACK_SIGNATURE) {
    this->short_pressed_callback = short_pressed_callback;
}

void BUTTON::loop(){
   button->loop(); // MUST call the loop() function first

  if(button->isPressed()){
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  }

  if(button->isReleased()) {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME ){
      Serial.println("A short press is detected");
      if(short_pressed_callback){
        short_pressed_callback();
      }
    }
      
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
      isLongDetected = true;
      if(long_pressed_callback){
        long_pressed_callback();
      }
    }
  }
}

void wifi_btn_short_pressed(){
   Serial.println("WiFi short press is detected");
   config.wifi_enable=!config.wifi_enable;
   config.save();
   ESP.restart();
}

void wifi_btn_long_pressed(){
   Serial.println("WiFi long press is detected");
   screen.drawText("Reset Wifi");
   delay(2000); // wait for a second
   Serial.println("To reset wifi");
   WiFiManager wm;
   wm.resetSettings();
   ESP.restart();
}

BUTTON wifi_btn(btnWiFiResetPin);
BUTTON co2_btn(btnCO2CalPin);

//------------------------------------------ co2 sensor ------------------------------------------------------
const byte MB_PKT_7 = 7;   //MODBUS Packet Size
const byte MB_PKT_8 = 8;   //MODBUS Packet Size
// SenseAir S8 MODBUS commands
const byte cmdReSA[MB_PKT_8] = {0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5}; // SenseAir Read CO2
const byte cmdOFFs[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03}; // SenseAir Close ABC
const byte cmdCal1[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5}; // SenseAir Calibration 1
const byte cmdCal2[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7}; // SenseAir Calibration 2
const byte cmdCal3[MB_PKT_8] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05}; // SenseAir Calibration 3
const byte cmdCalR[MB_PKT_7] = {0xFE, 0x03, 0x02, 0x00, 0x20, 0xAD, 0x88};       // SenseAir Calibration Response
static byte response[MB_PKT_8] = {0};

HardwareSerial &co2sensor = Serial1;
byte ConnRetry = 0;
int CO2 = 0;
int CO2value;
float CO2cor;
unsigned int crc_cmd;
int sensor_state=0;

//// Initialice SenseAir S8
void CO2iniSenseAir()
{
  //screen.drawText("Init CO2 Sensor");
  screen.drawBitmap();
  co2sensor.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //Deactivate Automatic Self-Calibration
  co2sensor.write(cmdOFFs, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);
  Serial.print(F("Deactivate Automatic Self-Calibration SenseAir S8: "));
  CheckResponse(cmdOFFs, response, MB_PKT_8);
  delay(2000);

  while (co2SenseAir() == 0 && (ConnRetry < 5))
  {
    BadConn();
  }

  if (ConnRetry == 5)
    softwareReset();

  Serial.println(F(" Air sensor detected AirSense S8 Modbus"));

  delay(10);
  Serial.println(F("SenseAir read OK"));
  delay(4000);
}

//Done or failed revision routine
void CheckResponse(const byte *a, const byte *b, uint8_t len_array_cmp)
{
  bool check_match = false;
  for (int n = 0; n < len_array_cmp; n++)
  {
    if (a[n] != b[n])
    {
      check_match = false;
      break;
    }
    else
      check_match = true;
  }

  if (check_match)
  {
    Serial.println(F("CheckResponse done"));
  }
  else
  {
    Serial.println(F("CheckResponse failed"));
  }
}

// CO2 lecture SenseAir S8
int co2SenseAir()
{
  int co2=0;
  static byte responseSA[MB_PKT_7] = {0};
  memset(responseSA, 0, MB_PKT_7);
  co2sensor.write(cmdReSA, MB_PKT_8);
  co2sensor.readBytes(responseSA, MB_PKT_7);
  co2 = (256 * responseSA[3]) + responseSA[4];

  Serial.print(F("Read SenseAir S8: "));

  if (co2 != 0)
  {
    crc_cmd = crcx::crc16(responseSA, 5);
    if (responseSA[5] == lowByte(crc_cmd) && responseSA[6] == highByte(crc_cmd))
    {
      Serial.println(F("OK DATA"));
      return co2;
    }
    else
    {
      while (co2sensor.available() > 0){
        char t = co2sensor.read(); // Clear serial input buffer;
      }

      Serial.println(F("FAIL CRC_CO2"));
      return 0;
    }
  } else{
    Serial.println(F("FAILED"));
    return 0; 
  }
}

// Calibration routine SenseAir S8

void CalibrationSenseAir()
{
  sensor_state=1;
  byte responseSA[MB_PKT_7] = {0};
  delay(100);

  //Step 1 Calibration
  co2sensor.write(cmdCal1, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);

  Serial.print(F("Calibration Step1: "));
  CheckResponse(cmdCal1, response, MB_PKT_8);
  delay(1000);

  //Step 2 Calibration
  co2sensor.write(cmdCal2, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);

  Serial.print(F("Calibration Step2: "));
  CheckResponse(cmdCal2, response, MB_PKT_8);
  delay(4000);

  //Step 3 Calibration
  co2sensor.write(cmdCal3, MB_PKT_8);
  co2sensor.readBytes(responseSA, MB_PKT_7);

  Serial.print(F("Resetting forced calibration factor to 400: "));
  CheckResponse(cmdCalR, responseSA, MB_PKT_7);
  delay(1000);
  sensor_state=0;
}

void BadConn()
{
  Serial.println("Air sensor not detected. Please check wiring... Try# " + String(ConnRetry));

  if (ConnRetry > 1)
  {
    delay(20);
  }
  delay(2500);
  ConnRetry++;
}


//Software RESET

void softwareReset()
{
  delay(2000);
  Serial.println(F("RESET..."));
  ESP.restart();
}



void display_data(){
    static unsigned long samplingTime = millis();
    unsigned long samplingInterval = 500; //ms, 10sec
    
    if(millis()-samplingTime > samplingInterval)
    {
      if(sensor_state==0){
        screen.drawCO2(CO2);
      }else if(sensor_state==1){
        screen.drawCalmode();
      }
      samplingTime=millis();
    }
}


void report_data(){
    static unsigned long samplingTime = millis();
    unsigned long samplingInterval  = 10000; //ms, 10sec

    if(sensor_state!=0){
      return;
    }
    
    if(millis()-samplingTime > samplingInterval)
    {
      // 組合MQTT訊息；
      String msgStr = "";      // 暫存MQTT訊息字串
      msgStr=msgStr+"{ \"co2\":"+CO2+ "}";
      mqtt_publish(DataTopic, msgStr);
      msgStr = "";
      samplingTime=millis();
    }
}

void TaskSerial1( void *pvParameters ){
  while(1){
      static unsigned long samplingTime = millis();
      unsigned long samplingInterval  = 1000; //ms, 1sec
      if(millis()-samplingTime > samplingInterval)
      {
        int CO2_Read = co2SenseAir();  
        if(CO2_Read != 0){
            CO2 = CO2_Read;
        }
        samplingTime=millis();
      }

      wifi_btn.loop();
      co2_btn.loop();
      delay(1); 
  }     
}

void setup_task() {
    xTaskCreatePinnedToCore(
       TaskSerial1,
       "TaskSerial1",   // A name just for humans
       10000,  // This stack size can be checked & adjusted by reading the Stack Highwater
       NULL,
       0,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
       NULL, 
       taskCore);
}

//=========================================================================================================

WIFI wifi(DeviceId, APPassword);

void setup()
{
  Serial.begin(115200);
  wifi_btn.init();
  wifi_btn.setLongPressedCallback(wifi_btn_long_pressed);
  wifi_btn.setShortPressedCallback(wifi_btn_short_pressed);
  co2_btn.init();
  co2_btn.setLongPressedCallback(CalibrationSenseAir);
  config.load();
  screen.init();
  CO2iniSenseAir();
  setup_task();
  pinMode(ONBOARD_LED,OUTPUT);
  if(config.wifi_enable){
    wifi.init();  
    mqtt_init();
    ntp.init();
    digitalWrite(ONBOARD_LED,HIGH);
  }else{
    digitalWrite(ONBOARD_LED,LOW);
  }
}
void loop()
{
  if(config.wifi_enable){
    wifi.check();
    mqtt_loop();
    report_data();
    ntp.loop();
  }
  display_data();
  
}
