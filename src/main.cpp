#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <Arduino.h>
#include <SPI.h>
#include "FastLED.h"
#include <CircularBuffer.h>
#include <WiFi.h>
#include <Preferences.h>
#include "time.h"
#include "exixe.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP32TimerInterrupt.h>
#include <iostream>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include "secrets.h"

// #define SECRET_WIFI_SSID "SSID"
// #define SECRET_WIFI_PASS "password"
// #define SECRET_WIFI_SSID2 "SSID"
// #define SECRET_WIFI_PASS2 "password"
// #define SECRET_SAFECAST_APIKEY "deviceid"
// #define SECRET_SAFECAST_DEVICEID 123
// #define SECRET_LAT "39.0000"
// #define SECRET_LON "-76.0000"
// #define SECRET_MQTT_PASS "password"
// #define HA_DEVCICE_ID "deviceid"

#define stringify( name ) #name
#define MAX_SSID_LEN      32
#define WIFI_DELAY        500
#define MAX_CONNECT_TIME  30000							 

// GM tube pulse input interrupt pin
#define PULSE_PIN  36 

// custom two led display for nixie reading cpm/uSv
#define DISPLAYMODEUSVH GPIO_NUM_19 // Low uSvH
#define DISPLAYMODECPM GPIO_NUM_18  // Low CPM
#define MODECPMCH 10  // LED PWM Channel
#define MODEUSVHCH 11 // LED PWM Channel
uint8_t bright=255; // led pwm brightness

// precision for LEDC timer
#define LEDC_TIMER_BIT  8
// use 1000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ  8000

#define PULSE_OUT GPIO_NUM_5 // note gpio5 puts out short pwm output on boot

// fastLED 								   						
#define NUM_LEDS 21
#define CLOCK_PIN 36  // not used
#define LED_DATA_PIN 21

// Nixie
#define NX1_CS_PIN 32
#define NX2_CS_PIN 33
#define NX3_CS_PIN 25
#define NX4_CS_PIN 26
#define MOSI 14
#define MISO 12
#define SCLK 13
 
 // Safecast
bool SENDDATA=true;
String serverResponse;
#define SC_APIKEY SECRET_SAFECAST_APIKEY
#define SC_DEVICEID SECRET_SAFECAST_DEVICEID
#define SC_HOST "api.safecast.org"
#define SC_API_MEASUREMENTS_ENDPOINT  "/measurements.json"
#define SC_TEST 0

exixe tube1 = exixe(NX1_CS_PIN);
exixe tube2 = exixe(NX2_CS_PIN);
exixe tube3 = exixe(NX3_CS_PIN);
exixe tube4 = exixe(NX4_CS_PIN);
uint8_t NixieBright=127;
bool AutoBright=true;
enum DisplayType{ cpm, uSvH,mSvY};
DisplayType display = DisplayType::cpm;

#define LDR_PIN GPIO_NUM_39   // analog voltage from ORP90 tube 

int FailedConnect=0;

bool BOOT=true;

float usvh_ratio = 0.001482;  // SBM-19 rough factor for CPM > uSv/h
                              // 80CPM for a background radiation level of 0.12uSv/h = 0.00148 
                              //https://www.pocketmagic.net/tube-sbm-19-%D1%81%D0%B1%D0%BC-19/
  // SBM-19 
  // Operating Voltage Range (volts): 350 – 475V
  // Initial voltage (volts): 260 – 320V
  // Recommended Operating Voltage (volts) : 400V
  // Minimum Dead Time (at U=400V, micro sec): 250us
  // Plateau Inclination: 0.1%/V
  // Working temperature: -60 to + 70 C
  // Counting speed: max. 2000 imp/s
  // Inherent counter background (cps) 1.83 Pulses/s
  // Interelectrode Capacitance 10pF
  // Load Resistance 5 – 10 MOhms
  // Sensitivity to gamma radiation:
  // MED – 3.0 mR ∙ s -1;
  // 247.5 mR -1 ± 26mkR -1
  // Length: 195mm
  // Diameter: 18mm

char ssid[MAX_SSID_LEN] = "";
char password[32] = "";
uint8_t netConnectFailCount=0;
String hostname = "geiger";

//MQTT
#define MQTT_PASS SECRET_MQTT_PASS
IPAddress mqtt_server(172, 16, 0, 92);
WiFiClient espClient;
PubSubClient client(espClient);
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

//RTC
time_t now; // this is the epoch
tm ti; // the structure tm holds time information in a more convient way

// http server
WebServer server(80);

long ctr = 0;


// Define the array of leds
CRGB leds[NUM_LEDS];
uint8_t intensity = 127;   // Default value for the LED intensity

// loop timing
uint32_t delt_t = 0;  // 1000 ms loop
uint32_t elapsed = 0;
uint16_t slowloop=240; // 5 minute counter


//tick pulse out duration
#define TIMER0_INTERVAL_US  200
ESP32Timer ITimer0(0);

// pulse out clear
bool IRAM_ATTR TimerHandler0(void * timerNo)
{ 
  REG_WRITE(GPIO_OUT_W1TC_REG, BIT5);
  return true;
}

//tick pulse in from Geiger tube
unsigned int interruptPin = PULSE_PIN;

//dummy ticks
unsigned long threshold = 3; //test data 3=~30 CPM
bool TestTick = false; // can be enabled via mqtt

uint32_t T = 0;

// variables shared between main code and interrupt code
volatile DRAM_ATTR bool event=false;
volatile DRAM_ATTR uint16_t eventCount = 0;

// Tick in interrupt routine declaration
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR Tick(){
  portENTER_CRITICAL_ISR(&mux);
  REG_WRITE(GPIO_OUT_W1TS_REG, BIT5);
  digitalWrite(PULSE_OUT,HIGH);  //tick out
  eventCount++; 
  event=true;
  portEXIT_CRITICAL_ISR(&mux);
}
							 
CircularBuffer<uint16_t,60> cpsbuff;  // 60 second fifo for CPM
Preferences preferences; // eeprom storage

bool ota = true;  // ota enabled - can be disabled via mqtt
bool OTAINPROGRESS = false;  //block tic events during ota update

double uSvH_val;
double mSvY_val;
uint16_t CPM  = 0; // Counts Per Minute
uint16_t cps = 0; // Counts Per Second
uint16_t currentCount; // event count since last loop
uint8_t onesCount = 0;  // LED ones
uint8_t tenCount = 9; // LED tens
uint16_t cpscount = 0; // cps accumulator
uint8_t S10Count = 0; // 10 second counter

 String err="";

void IRAM_ATTR SetLED(uint8_t led, CRGB color){
   leds[led]=color;
   FastLED.show();
}

// LED display cpm/uSvH
void showDisplayMode(uint8_t bright){
  uint8_t _bright = map(bright,0,255,220,0);
  if(display==DisplayType::cpm){
    ledcWrite(MODECPMCH,_bright);
    ledcWrite(MODEUSVHCH, 255);
  }
  else if(display==DisplayType::uSvH){
    ledcWrite(MODECPMCH,255);
    ledcWrite(MODEUSVHCH, _bright);
  }
  else{
    ledcWrite(MODECPMCH,255);
    ledcWrite(MODEUSVHCH, 255);
  }
}

void displayTest(){
  //on - off - on  - off
  ledcWrite(MODECPMCH,0);   ledcWrite(MODEUSVHCH, 0);
  delay(100);
  ledcWrite(MODECPMCH,255);   ledcWrite(MODEUSVHCH, 255);
  delay(100);
  ledcWrite(MODECPMCH,0);   ledcWrite(MODEUSVHCH, 0);
  delay(100);
  ledcWrite(MODECPMCH,255);   ledcWrite(MODEUSVHCH, 255);
  delay(100);
  // fade up cpm
  for(int i = 255;i>=0;i--){
    ledcWrite(MODECPMCH,i);
    delay(5);
  }
  delay(100);
  // off
  ledcWrite(MODECPMCH,255);   ledcWrite(MODEUSVHCH, 255);
   // fade up cpm
  for(int i = 255;i>=0;i--){
    ledcWrite(MODEUSVHCH,i);
    delay(5);
  }
  ledcWrite(MODECPMCH,255);   ledcWrite(MODEUSVHCH, 255);
}

void NixiTest(){
  int speed=80;
  tube1.show_digit(0, NixieBright, 0); 
  tube2.show_digit(0, NixieBright, 0); 
  tube3.show_digit(0, NixieBright, 0); 
  tube4.show_digit(0, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(1, NixieBright, 0); 
  tube2.show_digit(1, NixieBright, 0); 
  tube3.show_digit(1, NixieBright, 0); 
  tube4.show_digit(1, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(2, NixieBright, 0); 
  tube2.show_digit(2, NixieBright, 0); 
  tube3.show_digit(2, NixieBright, 0); 
  tube4.show_digit(2, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(3, NixieBright, 0); 
  tube2.show_digit(3, NixieBright, 0); 
  tube3.show_digit(3, NixieBright, 0); 
  tube4.show_digit(3, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(4, NixieBright, 0); 
  tube2.show_digit(4, NixieBright, 0); 
  tube3.show_digit(4, NixieBright, 0); 
  tube4.show_digit(4, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(5, NixieBright, 0); 
  tube2.show_digit(5, NixieBright, 0); 
  tube3.show_digit(5, NixieBright, 0); 
  tube4.show_digit(5, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(6, NixieBright, 0); 
  tube2.show_digit(6, NixieBright, 0); 
  tube3.show_digit(6, NixieBright, 0); 
  tube4.show_digit(6, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(7, NixieBright, 0); 
  tube2.show_digit(7, NixieBright, 0); 
  tube3.show_digit(7, NixieBright, 0); 
  tube4.show_digit(7, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(8, NixieBright, 0); 
  tube2.show_digit(8, NixieBright, 0); 
  tube3.show_digit(8, NixieBright, 0); 
  tube4.show_digit(8, NixieBright, 0); 
  delay(speed);
  tube1.show_digit(9, NixieBright, 0); 
  tube2.show_digit(9, NixieBright, 0); 
  tube3.show_digit(9, NixieBright, 0); 
  tube4.show_digit(9, NixieBright, 0); 
  delay(speed);
  tube1.clear();
  tube2.clear();
  tube3.clear();
  tube4.clear();
  tube1.show_digit(0, NixieBright, 0); 
  tube2.show_digit(0, NixieBright, 0); 
  tube3.show_digit(0, NixieBright, 0); 
  tube4.show_digit(0, NixieBright, 0); 
}

void NixieInit(){
  tube1.spi_init(SCLK,MISO,MOSI);
  tube1.clear();
  tube2.clear();
  tube3.clear();
  tube4.clear();
  tube1.set_led(127, 64,0); 
  tube2.set_led(127, 64,0);
  tube3.set_led(127, 64,0); 
  tube4.set_led(127, 64,0); 
}

void NixieUpdate(){
  //Set Nixie
    if(display == DisplayType::cpm) {
      tube4.show_digit(CPM%10, NixieBright, 0);
      tube3.show_digit(CPM/10%10, NixieBright, 0);
      tube3.set_dots( 0,0);
      tube2.show_digit(CPM/100%10, NixieBright, 0);
      tube1.show_digit(CPM/1000%10, NixieBright, 0);
    }
    if(display == DisplayType::uSvH) {
      if(uSvH_val>1){
        tube4.show_digit(int(uSvH_val*100)%10, NixieBright, 0);
        tube3.show_digit(int(uSvH_val*10)%10, NixieBright, 0);
        tube3.set_dots( NixieBright,0);
        tube2.show_digit(int(uSvH_val)%10, NixieBright, 0);
        tube1.show_digit(int(uSvH_val)%100, NixieBright, 0);
      }
      else{
        tube4.show_digit(int(uSvH_val*1000)%10, NixieBright, 0);  
        tube3.show_digit(int(uSvH_val*100)%10, NixieBright, 0);   
        tube2.show_digit(int(uSvH_val*10)%10, NixieBright, 0);
        tube2.set_dots( NixieBright,0);  // Type B required
        tube1.show_digit(int(uSvH_val)%10, NixieBright, 0);
      }
    }
    if(display == DisplayType::mSvY) {
      tube4.show_digit(int(mSvY_val*100)%10, NixieBright, 0);
      tube3.show_digit(int(mSvY_val*10)%10, NixieBright, 0);
      tube3.set_dots( NixieBright,0);
      tube2.show_digit(int(mSvY_val)%10, NixieBright, 0);
      tube1.show_digit(int(mSvY_val)%100, NixieBright, 0);
    }
}

/* Scan available networks and sort them in order to their signal strength. */
bool GetPreferredNetwork() {
  Serial.println("Begin scan.");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks available.");
    if(BOOT){leds[2]=CRGB(80,0,0);FastLED.show();}
  } 
  else {
    Serial.print(n);
    Serial.println(" networks discovered.");
    if(BOOT){leds[2]=CRGB(80,80,0);FastLED.show();}
    for (int i = 0; i < n; ++i) {
      Serial.print(i); Serial.print(" ");
      Serial.print(WiFi.SSID(i)); Serial.print(" ");
      Serial.print(WiFi.RSSI(i)); Serial.print(" ");
      Serial.print(WiFi.encryptionType(i));
      Serial.println();   
    }
  }
  Serial.println("Scan complete!");
  if (n == 0) {
    Serial.println("No networks available.");
  } 
  else 
  {
    Serial.print(n);
    Serial.println(" networks discovered.");
    // look for preferred networks
    String _ssid;
    memset(password, 0, 32);
    memset(ssid, 0, MAX_SSID_LEN);
    for (int i = 0; i < n; i++) {   
      _ssid = WiFi.SSID(i);   
      if(_ssid==SECRET_WIFI_SSID){       
        strncpy(ssid, WiFi.SSID(i).c_str(), MAX_SSID_LEN);     
        strncpy(password, String(SECRET_WIFI_PASS2).c_str(), MAX_SSID_LEN);
         Serial.println(_ssid + " selected.");
        return true;
      }
      else if(_ssid==SECRET_WIFI_SSID2){ 
        strncpy(ssid, WiFi.SSID(i).c_str(), MAX_SSID_LEN);
        strncpy(password, String(SECRET_WIFI_PASS2).c_str(), MAX_SSID_LEN);
        Serial.println(_ssid + " selected.");
        return true;
      }
    }
  }
  return false;
}

bool StartNetwork(){
  BOOT=true;
  if (!espClient.connected() || (WiFi.status() != WL_CONNECTED)){
    SetLED(0,CRGB(0,0,80));
    if(BOOT) if(netConnectFailCount>3) return false;
    SetLED(0,CRGB(0,80,0));
      /* Clear previous modes. */
    WiFi.softAPdisconnect();
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname.c_str()); //define hostname
    if(BOOT) SetLED(1,CRGB(0,0,80));
    delay(500);
    if(BOOT) SetLED(1,CRGB(0,80,0));
    memset(ssid, 0, MAX_SSID_LEN);
    Serial.println("GetPreferredNetwork..");
    if(GetPreferredNetwork()){
      Serial.print(" Connecting to "); Serial.print(ssid); Serial.print(" "); Serial.println(password);
      if(ssid[0] != '\0'){
        WiFi.begin(ssid,password);
        int loop=0;
        while (WiFi.status() != WL_CONNECTED && loop < 30) {
          delay(250);
          Serial.println(" Connecting to WiFi..");        
          tube4.show_digit((uint8_t)loop%10,127,0);
          tube3.show_digit((uint8_t)loop/10%10,127,0);
          loop++;
        }
        Serial.println("Ready");
        Serial.print("IP:"); Serial.println( WiFi.localIP());
        if(BOOT){leds[3]=CRGB(80,80,0);FastLED.show();}
        if(ota) ArduinoOTA.begin();
        return true;
      }
    }
    netConnectFailCount++;
    return false;
  }
  return true;
}

bool ConnectMQTT(){
  int loop=0;
  while (!client.connected() && loop < 5){
    SetLED(19,CRGB(0,0,80));
    //espClient.clearWriteError();
    client.setServer(mqtt_server, 1883);
    if (client.connect("GeigerCounter","geiger",MQTT_PASS)) {
      Serial.println("mqtt connected");
      // home assistant config
      static String pre PROGMEM = "{\"state_topic\":\"homeassistant/sensor/geiger/state\",\"avty_t\":\"homeassistant/sensor/geiger/status\",\"stat_cla\":\"measurement\",";
      static String dev PROGMEM = "\"device\":{\"ids\":[\""+ String(HA_DEVCICE_ID)  +"\"],\"name\":\"geiger\",\"sw\":\"/Projects/Geiger\"}}";
      
      static String param1 PROGMEM = "\"name\":\"Ionizing Radiation CPM\",\"unit_of_meas\":\"CPM\",\"uniq_id\":\"mbgeigercpm\",\"value_template\":\"{{value_json.cpm}}\",";      
      client.beginPublish("homeassistant/sensor/geiger/cpm/config", pre.length()+param1.length()+dev.length(), true);
      client.print(pre);
      client.print(param1); 
      client.print(dev);
      client.endPublish();
      
      static String param2 PROGMEM  = "\"name\":\"Ionizing Radiation uSv per Hour\",\"unit_of_meas\":\"uSvH\",\"uniq_id\":\"mbgeigerusvh\",\"value_template\": \"{{value_json.uSvH}}\",";
      client.beginPublish("homeassistant/sensor/geiger/uSvH/config", pre.length()+param2.length()+dev.length(), true);
      client.print(pre);
      client.print(param2); 
      client.print(dev);
      client.endPublish();

      static String param3 PROGMEM  = "\"name\":\"Ionizing Radiation mSv per Year\",\"unit_of_meas\":\"mSvY\",\"uniq_id\":\"mbgeigermsvy\",\"value_template\": \"{{value_json.mSvY}}\",";
      client.beginPublish("homeassistant/sensor/geiger/mSvY/config", pre.length()+param3.length()+dev.length(), true);
      client.print(pre);
      client.print(param3); 
      client.print(dev);
      client.endPublish();
 
      client.publish("homeassistant/sensor/geiger/status","online");
      // Subscribe       
      client.subscribe("homeassistant/sensor/geiger/output");
      SetLED(19, CRGB(0,0,0));
    } 
    else {
      Serial.print("mqtt connect failed, rc=");
      Serial.println(client.state());
      SetLED(19, CRGB(0,0,80)); 
    }
    delay(200);
    loop++;
  }
  // force network
  if(!client.connected()){
    FailedConnect++;
    if(FailedConnect>3){
      WiFi.softAPdisconnect();
      WiFi.disconnect();
      WiFi.mode(WIFI_STA);
      StartNetwork();
      FailedConnect=0;
    }
  }

  if(client.connected())
    return true;
  else
    return false;
}

void IRAM_ATTR UpdateLED(uint8_t ones,uint8_t tens){
  FastLED.clearData();
  leds[ones] = CHSV(2,255,intensity);
  leds[tens+10 ] = CHSV(2,255,intensity); 
  //blue led on wifi issue
  if(!WiFi.isConnected())
    leds[14]=CRGB(0,0,70);
  //green led on wifi issue
  if(!client.connected())
    leds[15]=CRGB(0,70,0);
  FastLED.show();
}

void LEDTest(){
 	FastLED.clearData();
  for(int i=0;i<10;i++){
    leds[i]=CHSV(2,255,intensity);
    leds[10+i]=CHSV(110,255,intensity);
    FastLED.show(); 
    delay(160);
  }
  // FastLED.clearData();
  // for(int i=0;i<20;i++){
  //   leds[i]=CHSV(255,255,255);
  // }
  //FastLED.show(); 
  //delay(100);
  FastLED.clearData(); 
  FastLED.show();
}

void publishState(){
  snprintf (msg, MSG_BUFFER_SIZE, "{\"usvh_ratio\": %f}", usvh_ratio);
  client.publish("homeassistant/sensor/geiger/state/usvh_ratio", msg); 
  snprintf (msg, MSG_BUFFER_SIZE, "{\"LEDBright\": %d}", intensity);
  client.publish("homeassistant/sensor/geiger/state/lbri", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"test\": %s}", TestTick? "true" : "false");
  client.publish("homeassistant/sensor/geiger/state/test", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"threshold\": %lu}", threshold);
  client.publish("homeassistant/sensor/geiger/state/threshold", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"ota\": %d}", ota);
  client.publish("homeassistant/sensor/geiger/state/ota", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"display\": %s}", stringify(display));
  client.publish("homeassistant/sensor/geiger/state/display", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"NixieBright\": %d}", NixieBright);
  client.publish("homeassistant/sensor/geiger/state/nbri", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"AutoBright\": %d}", AutoBright);
  client.publish("homeassistant/sensor/geiger/state/senddata", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"SendData\": %d}", SENDDATA);
  client.publish("homeassistant/sensor/geiger/state/abri", msg,true);
  snprintf (msg, MSG_BUFFER_SIZE, "{\"Bright\": %d}", bright);
  client.publish("homeassistant/sensor/geiger/state/bright", msg,true);
}

void SavePrefs(){
  preferences.putBool("ota", ota);
  preferences.putBool("TestTick", TestTick);
  preferences.putULong("threshold", threshold);
  preferences.putUShort("intensity", intensity);
  preferences.putFloat("usvh_ratio",usvh_ratio);
  preferences.putUShort("NixieBright", NixieBright);
  preferences.putBool("AutoBright", AutoBright);
  preferences.putBool("SendData", SENDDATA);
  preferences.putString("display", stringify(display));
}

// send commands to the Geiger counter
void mqttcallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(); 

  StaticJsonDocument<200> doc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, message);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("mqtt message deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  
  if(doc["test"])  //turn on dummy ticks for testing
  {
    TestTick = doc["test"]=="1"?true:false; 
    if(doc["threshold"]) threshold = doc["threshold"];
    Serial.print("threshold:");
    Serial.println(threshold);
    cpsbuff.clear();
  }
  if(doc["usvh_ratio"])  // override the SBM-19 usvh_ratio
    usvh_ratio=doc["usvh_ratio"];
  
  if(doc["lbri"])
    intensity=doc["intensity"]; // dekatron led brightness

  if(doc["nbri"])
    NixieBright=doc["nbri"]; 

  if(doc["abri"])
    AutoBright=doc["abri"]=="1"?true:false; 

  if(doc["send"])
    SENDDATA=doc["send"]=="1"?true:false; 

  if(doc["ota"]=="1"){
    Serial.print("start OTA");
    ArduinoOTA.begin();
  }

  if(doc["ota"]=="0"){
    Serial.print("stop OTA");
    ArduinoOTA.end();
  }

  if(doc["display"])
  {
    //clear the period
    if(doc["display"]=="cpm"){
      tube1.clear();tube2.clear();tube3.clear();tube4.clear();
      display = DisplayType::cpm;
      digitalWrite(DISPLAYMODEUSVH,1);digitalWrite(DISPLAYMODECPM,0);
    }
    if(doc["display"]=="mSvY"){
      display = DisplayType::mSvY;
      digitalWrite(DISPLAYMODEUSVH,1);digitalWrite(DISPLAYMODECPM,1);
    }
    if(doc["display"]=="uSvH"){
      display = DisplayType::uSvH;
      digitalWrite(DISPLAYMODEUSVH,0);digitalWrite(DISPLAYMODECPM,1);
    }
    showDisplayMode(0);
  }

  SavePrefs();
  publishState();
}

// ISO Date string
String getLocalTimeStampString() {
  time(&now);                       // read the current time
  gmtime_r(&now, &ti);           // update the structure tm with the current time                     
  uint16_t year = ti.tm_year + 1900;
  String yearStr = String(year);
  uint8_t month = ti.tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);
  uint8_t day = ti.tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);
  uint8_t hours = ti.tm_hour;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);
  uint8_t minutes = ti.tm_min;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);
  uint8_t seconds = ti.tm_sec;
  String secondsStr = seconds < 10 ? "0" + String(seconds) : String(seconds);
  return yearStr + "-" + monthStr + "-" + dayStr + "T" +  hoursStr + ":" + minuteStr +  ":" + secondsStr + "Z";
}

void SafeCast(){
  if(WiFi.isConnected()){
    WiFiClientSecure SCclient;
    DynamicJsonDocument jsondoc(300);
    String ISODate=getLocalTimeStampString(); 
    char post[100];
    serverResponse="";
    sprintf(post,
            "POST %s?api_key=%s&%s HTTP/1.1",
            SC_API_MEASUREMENTS_ENDPOINT,
            SC_APIKEY,
            SC_TEST ? "test=true" : "");
    jsondoc["captured_at"]=ISODate; 
    jsondoc["unit"]="usv";
    jsondoc["device_id"]=SC_DEVICEID;
    jsondoc["value"]= (int)(uSvH_val * 1000 + 0.5) / 1000.0; //round 3 decimal places
    jsondoc["latitude"]=SECRET_LAT;
    jsondoc["longitude"]=SECRET_LON;
    serializeJson(jsondoc,serverResponse);
    serverResponse += "<br/>";
    SCclient.setInsecure();
    if(ISODate.startsWith("20") && jsondoc["value"]>0 && SENDDATA && SCclient.connect(SC_HOST,443)){
       SCclient.println(post);
       SCclient.println("Connection: close");
       SCclient.print("Content-Length: ");
       SCclient.println(measureJson(jsondoc));
       SCclient.println("Content-Type: application/json");
       SCclient.print("Host: ");
       SCclient.println(SC_HOST);
       // Terminate headers with a blank line
       SCclient.println();
       serializeJson(jsondoc, SCclient);
       // Get Client Response
       unsigned long timeout = millis();
       while (SCclient.available() == 0){
         if (millis() - timeout > 3000){
           SCclient.stop();
           serverResponse += "Error Safecast client timeout<br/>";
           break;
         }
       }
       while(SCclient.available()){
         serverResponse += SCclient.readStringUntil('\r'); //.replace("\r","<br/>"
       } 
       SCclient.stop();
       serverResponse += "<br/>";

     }
     else
     {
        if(SENDDATA)
          serverResponse += "Error Cannot connect to the Safecast host<br/>";
        else
          serverResponse += "Safecast DISABLED<br/>";   
     }
  }
}

void RadMon(){
  // https://radmon.org/radmon.php?function=submit&user=youruser&password=yourpassword&value=20&unit=CPM
  if(WiFi.isConnected()){
    WiFiClientSecure RMclient;
    RMclient.setInsecure();
    if(CPM>0 && SENDDATA && RMclient.connect("radmon.org",443)){
      // Send a GET request to a web page hosted by the server.
      RMclient.print("GET radmon.php?function=submit&user=youruser&password=yourpassword&value=");
      RMclient.print(CPM);
      RMclient.print("&unit=CPM HTTP/1.1\r\nHost: radmon.org\r\nConnection: close\r\n\r\n");
    }
  }
}

//Web server
String SendHTML(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\"><meta http-equiv=\"refresh\" content=\"5 url=/\">\n";
  ptr +="<title>Mr Bubble Geiger Counter</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>Geiger Counter</h1>\n";
  char buf[10];
  sprintf(buf, "%d", CPM);
  ptr +="<h1>CPM: ";
  ptr +=  buf;
  ptr += "</h1>\n";

  sprintf(buf, "%.3f",  CPM*usvh_ratio);
  ptr +="<h1>uSv/H: ";
  ptr +=  buf;
  ptr += "</h1>\n";

//simulate nixie
//   ptr +="<h1>";
// if(uSvH_val>1){
//   sprintf(buf, "%d", int(uSvH_val)%100);
//   ptr +=  buf;
//   sprintf(buf, "%d", int(uSvH_val)%10);
//   ptr +=  buf;
//   ptr +=".";
//   sprintf(buf, "%d", int(uSvH_val*10)%10);
//   ptr +=  buf;
//   sprintf(buf, "%d", int(uSvH_val*100)%10);
//   ptr +=  buf;
// }
// else{
//   sprintf(buf, "%d", int(uSvH_val)%10);
//   ptr +=  buf;
//   ptr +=".";
//   sprintf(buf, "%d", int(uSvH_val*10)%10);
//   ptr +=  buf;
//   sprintf(buf, "%d", int(uSvH_val*100)%10);
//   ptr +=  buf;
//   sprintf(buf, "%d", int(uSvH_val*1000)%10);
//   ptr +=  buf;
// }
// ptr += "</h1>\n";

  //ptr += getTimeStampString();
  //ptr += "<br>\n";

  ptr += getLocalTimeStampString() ; 

  ptr += "<p>SafeCast in ";
  ptr += 300-slowloop;
  ptr += "<br>\n";
  ptr += serverResponse;
  ptr += "</p>\n";

   char output[76];
//   snprintf(output, sizeof(output), "{\"FreeHeap\":%d,\"FreePsram\":%d,\"cycles\":%u\"ctr\":%d,\"rssi\":%d}", ESP.getFreeHeap(), ESP.getFreePsram(),ESP.getCycleCount(), ctr,WiFi.RSSI());
   snprintf(output, sizeof(output), "{\"Failed\":%d,\"mqtt\":%s,\"state\":%d,\"rssi\":%d}", FailedConnect, client.connected()?"Y":"N", client.state(), WiFi.RSSI());
   ptr += output;
   ptr += "\n";
  if(display==DisplayType::cpm)  {ptr += "<p><a class=\"button button-on\" href=\"/display\">uSV</a></p>\n";}
  if(display==DisplayType::uSvH)  {ptr += "<p><a class=\"button button-off\" href=\"/display\">CPM</a></p>\n";}

  if(SENDDATA)  {ptr += "<p><a class=\"button button-on\" href=\"/senddata\">STOP</a></p>\n";}
  if(!SENDDATA)  {ptr += "<p><a class=\"button button-off\" href=\"/senddata\">SEND</a></p>\n";}

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

void handle_OnSendData(){
    SENDDATA=!SENDDATA;
    preferences.putBool("SendData", SENDDATA);
    server.send(200, "text/html", SendHTML()); 
}

//toggle display mode
void handle_OnDisplay(){
    if(display == DisplayType::cpm){
      display = DisplayType::uSvH;
      digitalWrite(DISPLAYMODEUSVH,0);digitalWrite(DISPLAYMODECPM,1);
    }
    else{
      display = DisplayType::cpm;
      digitalWrite(DISPLAYMODEUSVH,1);digitalWrite(DISPLAYMODECPM,0);
    }
    showDisplayMode(bright);
    server.send(200, "text/html", SendHTML()); 
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML()); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

// idle counter
void IRAM_ATTR id(void*z)
{
	while (1)
	{
		ctr++;
		delay(10);
	}
}

void setup() { 
	Serial.begin(115200);
	Serial.println("resetting");
	pinMode(DISPLAYMODEUSVH,OUTPUT);
  pinMode(DISPLAYMODECPM,OUTPUT);
  ledcSetup(MODECPMCH, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(MODEUSVHCH, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(DISPLAYMODECPM,MODECPMCH);
  ledcAttachPin(DISPLAYMODEUSVH,MODEUSVHCH);
  displayTest();
  display = DisplayType::cpm;
  // get settings from flash
	if(preferences.begin("geigercounter", false)){
    TestTick = preferences.getBool("TestTick", false);
    threshold = preferences.getULong("threshold", threshold);
    intensity = preferences.getUShort("intensity", intensity);
    usvh_ratio = preferences.getFloat("usvh_ratio",usvh_ratio);
    NixieBright = preferences.getUShort("NixieBright", NixieBright);
    AutoBright = preferences.getBool("AutoBright", AutoBright);
    SENDDATA = preferences.getBool("SendData", SENDDATA);
    if(preferences.getString("display", stringify(display))=="mSvY") display = DisplayType::mSvY;
    if(preferences.getString("display", stringify(display))=="uSvH") {display = DisplayType::uSvH;digitalWrite(DISPLAYMODEUSVH,0);digitalWrite(DISPLAYMODECPM,1);}
  }
  else {
    preferences.clear();
    SavePrefs();
  }

  //200us Pulse out
  pinMode(PULSE_OUT,OUTPUT);
  digitalWrite(PULSE_OUT,LOW);
  ITimer0.attachInterruptInterval(TIMER0_INTERVAL_US, TimerHandler0);

	// attach interrupt routine to TIC interface from the geiger counter module
	pinMode(interruptPin, INPUT);   // Set pin for capturing Tube events
	attachInterrupt(interruptPin, Tick, FALLING);

  //dekatron
	FastLED.addLeds<WS2811,LED_DATA_PIN,RGB>(leds,NUM_LEDS);
	FastLED.setBrightness(84);
	FastLED.setTemperature(Typical8mmPixel);
	LEDTest();
  // initial LED's at 0 0
  UpdateLED(9,9);

  //debug
  //printstate();

  NixieInit();	
  NixiTest();
  showDisplayMode(0);

  //wifi
  if(StartNetwork()){ 
    UpdateLED(0,9);
    // init RTC to NTP
    configTime(0, 0, "pool.ntp.org");
    //mqtt
    ConnectMQTT();
  }
  
  currentCount=9;
  eventCount=0;
  BOOT=false;

  ArduinoOTA.onStart([]() {
    OTAINPROGRESS=true;
  });

  server.on("/", handle_OnConnect);
  server.on("/senddata", handle_OnSendData);
  server.on("/display", handle_OnDisplay);
  server.onNotFound(handle_NotFound);
  server.begin();

  // testing cpu load
  //xTaskCreatePinnedToCore(id, "id", 4096, NULL, 0, NULL, 0);
}

void IRAM_ATTR loop() { 
  if(event && !OTAINPROGRESS){
    noInterrupts();
    currentCount = eventCount;
    eventCount=0;
    event=false;
    interrupts();
    cpscount += currentCount;
    onesCount += currentCount;
    // led counters
    if(onesCount >= 10) {
      tenCount++;
      onesCount=0;
    } 
    if(tenCount >= 10){
      tenCount=0;
    }
    UpdateLED(onesCount,tenCount);  
  }
  currentCount=0;
  //simulate count
  if(TestTick){
    unsigned long r = random(50000);
    if (r < threshold) Tick(); 
  }

  // update once per second independent of read rate
  delt_t = millis() - elapsed;
  if (delt_t > 1000) {
	  elapsed = millis(); 
    cps = cpscount;
    //calculate CPM;  cpm is sum of cps
    cpsbuff.push(cps);
    cpscount=0;
    CPM=0; 
    for(uint8_t i = 0;i < cpsbuff.size();i++){
      CPM += cpsbuff[i];
    }
    uSvH_val = CPM*usvh_ratio;
    mSvY_val = CPM*usvh_ratio*24.*365.*0.001;
   
    if(AutoBright) {
      int adc = analogRead(LDR_PIN);   // read the analog voltage from ORP90 tube 
      // map LDR log response
      bright = 255-log(adc+1)/log(4096)*255;
      // Nixie range 8 - 127
      NixieBright =  map(bright,0,255,8,127);
      // LED range 80 - 220
      intensity = map(bright,0,255,76,220); 
    }
    else{
      bright=255;
      NixieBright=127;
      intensity=200;
    }

    NixieUpdate();
    showDisplayMode(bright);

    S10Count++;
    if(S10Count>=10){  //send mqtt every 10 seconds  
      if(client.connected()){
        char output[76];
        snprintf(output, sizeof(output), "{\"cpm\":%d,\"uSvH\":%.3f,\"mSvY\":%.3f,\"bright\":%d,\"rssi\":%d}", CPM,uSvH_val,mSvY_val,bright, WiFi.RSSI());
        client.publish("homeassistant/sensor/geiger/state", output);
      } 
      S10Count=0;
    }

    // 10 min loop retry wif/mqtt logon, send data
    slowloop++;
    if(slowloop>300){  
      // check wifi
      if(!OTAINPROGRESS && StartNetwork()){
        //ntp
        //timeClient.update();
        // mqtt connected
        SafeCast();      
        if(ConnectMQTT()){            
            publishState();
        }
      }
      slowloop=0;
    }
    ctr=0; //cpu load
  }

  //mqtt
  if(WiFi.isConnected()){
    client.loop();
  }

  //ota
  if(WiFi.isConnected() && ota)
    ArduinoOTA.handle();
  
  // http
  server.handleClient();
}


