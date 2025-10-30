#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "driver/i2s.h"

/* ========= Wi-Fi & Gemini Configuration ========= */
const char* ssid = "XuLiu";
const char* password = "liuxu123";
const String GEMINI_API_KEY = "AIzaSyA0Luy21a7bEPeq9NZA4n0EA_DwxLNp328";

/* ========= Hardware Pin Assignments ========= */
const int BUZZER_PIN = 25;
const int LED_PIN    = 26;
const int LDR_PIN    = 36;
const int STRIP_PIN  = 17;
const int ORANGE_LED_PIN = 13;

#define NUM_LEDS 30
Adafruit_NeoPixel strip(NUM_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

/* ========= RGB LED (PWM Control) ========= */
const int PIN_R = 21, PIN_G = 22, PIN_B = 23;
const int CH_R = 1, CH_G = 2, CH_B = 3;
inline void setRGB(uint8_t r,uint8_t g,uint8_t b){
  ledcWrite(CH_R,r);
  ledcWrite(CH_G,g);
  ledcWrite(CH_B,b);
}

/* ========= I2S Microphone Settings ========= */
#define I2S_WS 14
#define I2S_SCK 16
#define I2S_SD 2
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000
#define READ_LEN 512
static const size_t g_recSamples=(size_t)(SAMPLE_RATE*2.0);
static size_t g_recBytes=g_recSamples*sizeof(int16_t);
const int RMS_TRIGGER_THRESHOLD=3000;           // Audio RMS threshold for voice activation
const unsigned long VOICE_COOLDOWN_MS=5000;     // Cooldown time between voice detections

/* ========= BLE Configuration ========= */
const char* TARGET_NAME="TREASURE_05";
const int TH_SILENT=-70,TH_SLOW=-65,TH_MED=-60,TH_FAST=-55,TH_VFAST=-50,TH_FOUND=-40;
const int QL_LOW=-52,QL_HIGH=-40;
const uint8_t QL_HITS=3;
const uint16_t QL_WINDOW_MS=350;
int LDR_ON_TH=1600,LDR_OFF_TH=2700;
const uint16_t STALE_MS=600;
volatile int latestRSSI=-127;
volatile unsigned long lastSeenMs=0;
enum Mode{IDLE,SCAN};
Mode mode=IDLE;

/* ========= State Control Flags ========= */
bool micEnabled=true;
bool allowMicAutoResume=true;
unsigned long micResumeTime=0;
unsigned long scanStartTime=0;
double lastRMS=0.0;

/* ========= Orange LED Task (Status Indicator) ========= */
enum OrangeMode { ORANGE_OFF, ORANGE_SOLID, ORANGE_BLINK };
volatile OrangeMode orangeMode = ORANGE_OFF;
TaskHandle_t orangeBlinkTaskHandle=nullptr;

/**
 * Handles the orange status LED behavior:
 *  - BLINK: during Gemini analysis
 *  - SOLID: during recording
 *  - OFF: idle
 */
void OrangeBlinkTask(void*){
  pinMode(ORANGE_LED_PIN,OUTPUT);
  for(;;){
    switch(orangeMode){
      case ORANGE_BLINK:
        digitalWrite(ORANGE_LED_PIN,!digitalRead(ORANGE_LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(400));
        break;
      case ORANGE_SOLID:
        digitalWrite(ORANGE_LED_PIN,HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        break;
      default:
        digitalWrite(ORANGE_LED_PIN,LOW);
        vTaskDelay(pdMS_TO_TICKS(80));
        break;
    }
  }
}

/* ========= Base64 Audio Encoder ========= */
namespace Base64{
  const char PROGMEM tbl[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encode(const uint8_t*data,size_t len){
    String out;out.reserve(((len+2)/3)*4);
    for(size_t i=0;i<len;i+=3){
      uint32_t v=0;
      v|=(uint32_t)data[i]<<16;
      if(i+1<len)v|=(uint32_t)data[i+1]<<8;
      if(i+2<len)v|=(uint32_t)data[i+2];
      out+=pgm_read_byte(&tbl[(v>>18)&0x3F]);
      out+=pgm_read_byte(&tbl[(v>>12)&0x3F]);
      out+=(i+1<len)?pgm_read_byte(&tbl[(v>>6)&0x3F]):'=';
      out+=(i+2<len)?pgm_read_byte(&tbl[v&0x3F]):'=';
    }
    return out;
  }
}

/* ========= Median Filter for RSSI Stabilization ========= */
int median5_initFill=-127;
int median5(int x){
  static int buf[5]={-127,-127,-127,-127,-127};
  static uint8_t n=0;
  buf[n=(n+1)%5]=x;
  int a[5];
  for(int i=0;i<5;i++)a[i]=buf[i];
  for(int i=1;i<5;i++){
    int k=a[i],j=i-1;
    while(j>=0&&a[j]>k){a[j+1]=a[j];j--;}
    a[j+1]=k;
  }
  return a[2];
}

/* ========= BLE Advertisement Callback ========= */
class AdvCB:public NimBLEAdvertisedDeviceCallbacks{
  void onResult(NimBLEAdvertisedDevice*adv)override{
    if(!adv->haveName())return;
    if(adv->getName()==TARGET_NAME){
      latestRSSI=adv->getRSSI();
      lastSeenMs=millis();
    }
  }
};

NimBLEScan* pScan=nullptr;

/* ========= Utility Functions ========= */
inline void beep(int ms){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN,LOW);
}

/** Plays a short tone and RGB blink pattern when treasure is found */
void playFoundTone(){
  for(int repeat=0;repeat<3;repeat++){
    tone(BUZZER_PIN,1000,180);delay(200);
    tone(BUZZER_PIN,1800,220);delay(240);
    tone(BUZZER_PIN,2500,260);delay(280);
    noTone(BUZZER_PIN);
    for(int i=0;i<6;i++){
      setRGB(255,0,0);
      delay(100);
      setRGB(0,0,0);
      delay(100);
    }
    delay(200);
  }
}

/* ========= I2S Microphone Setup ========= */
void setupI2SMic(){
  i2s_config_t cfg={
    .mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX),
    .sample_rate=SAMPLE_RATE,
    .bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format=I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format=(i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count=8,
    .dma_buf_len=64,
    .use_apll=false
  };
  i2s_pin_config_t pin={I2S_PIN_NO_CHANGE,I2S_SCK,I2S_WS,I2S_PIN_NO_CHANGE,I2S_SD};
  i2s_driver_install(I2S_PORT,&cfg,0,NULL);
  i2s_set_pin(I2S_PORT,&pin);
  Serial.println("✅ I2S Microphone Initialized");
}

/* ========= Compute RMS (Root Mean Square) of Audio Samples ========= */
double getMicRMS(){
  int16_t s[READ_LEN];
  size_t br;
  i2s_read(I2S_PORT,(char*)s,READ_LEN*sizeof(int16_t),&br,portMAX_DELAY);
  int n=br/sizeof(int16_t);
  if(n<=0)return 0;
  double amp=20.0,sum=0;
  for(int i=0;i<n;i++){
    int v=(int)(s[i]*amp);
    if(v>32767)v=32767;
    if(v<-32768)v=-32768;
    s[i]=(int16_t)v;
    sum+=(double)v*v;
  }
  return sqrt(sum/n);
}

/* ========= Wi-Fi Connection ========= */
bool connectWiFi(){
  WiFi.begin(ssid,password);
  Serial.print("🌐 Connecting WiFi");
  int r=0;
  while(WiFi.status()!=WL_CONNECTED&&r<25){
    delay(500);
    Serial.print(".");
    r++;
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.printf("\n✅ WiFi Connected: %s\n",WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("\n❌ WiFi Failed");
  return false;
}

/* ========= Gemini API Call: Audio Intent Detection ========= */
String detectIntentFromAudio(const String& b64){
  orangeMode = ORANGE_BLINK;  
  Serial.println("🟠 Gemini analyzing... (orange LED blinking)");
  WiFiClientSecure cli;cli.setInsecure();HTTPClient https;
  String url="https://generativelanguage.googleapis.com/v1beta/models/"
             "gemini-2.5-flash-preview-05-20:generateContent?key="+GEMINI_API_KEY;
  if(!https.begin(cli,url)){
    Serial.println("❌ HTTPS begin fail");
    orangeMode=ORANGE_OFF;
    return"NONE";
  }

  // Construct request payload
  DynamicJsonDocument doc(8192);
  JsonArray contents=doc.createNestedArray("contents");
  JsonObject content=contents.createNestedObject();
  JsonArray parts=content.createNestedArray("parts");
  String prompt=R"(
You are an audio intent detector for a treasure-hunting device.
Input: a short speech audio clip (English or Chinese).
Goal: decide whether the user wants to START/CONTINUE searching (WAKE),
STOP/PAUSE/END/mute the search (STOP), or it's irrelevant (NONE).

Rules:
- STOP has priority when ambiguous (e.g., 'be quiet', 'mute it', 'enough').
- Treat questions like 'where is my treasure?' / '我们开始吧' / '开始找宝藏' as WAKE.
- Treat phrases like 'we found it', 'stop searching', '别找了', '关闭它', '停止搜索' as STOP.
- If uncertain, answer NONE.

Respond ONLY with one of the following tokens (case-insensitive): WAKE, STOP, NONE.
)";
  parts.createNestedObject()["text"]=prompt;
  JsonObject inline_data=parts.createNestedObject()["inline_data"].to<JsonObject>();
  inline_data["mime_type"]="audio/l16;rate=16000";
  inline_data["data"]=b64;

  // Send POST request
  String payload;
  serializeJson(doc,payload);
  https.addHeader("Content-Type","application/json");
  https.setTimeout(40000);
  Serial.printf("📡 Sending JSON (%.1f KB)...\n",payload.length()/1024.0);
  int code=https.POST(payload);

  if(code<=0){
    Serial.printf("❌ POST fail: %s\n",https.errorToString(code).c_str());
    https.end();
    orangeMode=ORANGE_OFF;
    return"NONE";
  }

  String resp=https.getString();
  https.end();
  orangeMode=ORANGE_OFF;
  Serial.println("🔵 Gemini analysis complete.");

  DynamicJsonDocument rdoc(8192);
  if(deserializeJson(rdoc,resp)){
    Serial.println("❌ JSON parse error");
    return"NONE";
  }

  String text=rdoc["candidates"][0]["content"]["parts"][0]["text"]|"NONE";
  text.toUpperCase();
  text.trim();
  Serial.printf("🤖 Gemini raw: '%s'\n",text.c_str());
  if(text=="STOP")return"STOP";
  if(text=="WAKE")return"WAKE";
  return"NONE";
}

/* ========= Setup Routine ========= */
void setup(){
  Serial.begin(115200);
  delay(2000);

  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(LDR_PIN,INPUT);
  pinMode(ORANGE_LED_PIN,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  // Configure RGB PWM channels
  ledcSetup(CH_R,5000,8);
  ledcSetup(CH_G,5000,8);
  ledcSetup(CH_B,5000,8);
  ledcAttachPin(PIN_R,CH_R);
  ledcAttachPin(PIN_G,CH_G);
  ledcAttachPin(PIN_B,CH_B);

  strip.begin();
  strip.show();
  strip.setBrightness(180);

  // Initialize BLE scanner
  analogReadResolution(12);
  analogSetPinAttenuation(LDR_PIN,ADC_11db);
  NimBLEDevice::init("");
  pScan=NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCB(),false);
  pScan->setActiveScan(true);
  pScan->setInterval(80);
  pScan->setWindow(60);
  pScan->setDuplicateFilter(false);
  pScan->setMaxResults(0);

  setupI2SMic();
  xTaskCreatePinnedToCore(OrangeBlinkTask,"OrangeBlink",2048,nullptr,1,&orangeBlinkTaskHandle,1);

  if(connectWiFi())digitalWrite(LED_PIN,HIGH);
  Serial.println("✅ System Ready");
}

/* ========= Main Loop ========= */
void loop(){
  // Ambient light control for LED strip
  int ldr=analogRead(LDR_PIN);
  static bool lamp=false;
  if(!lamp&&ldr<LDR_ON_TH)lamp=true;
  if(lamp&&ldr>LDR_OFF_TH)lamp=false;
  if(lamp){strip.fill(strip.Color(255,255,255));}
  else{strip.clear();}
  strip.show();

  // Display periodic sensor values
  static unsigned long last=0,lastTrig=0;
  if(millis()-last>1000){
    last=millis();
    lastRMS=getMicRMS();
    int rssi_for_print=(latestRSSI==-127)?-127:median5(latestRSSI);
    Serial.printf("🌞 LDR=%d | 🎙 RMS=%.1f | RSSI=%d | Mode=%s\n",
      ldr,lastRMS,rssi_for_print,mode==IDLE?"IDLE":"SCAN");
  }

  // Re-enable microphone after cooldown
  if(!micEnabled&&allowMicAutoResume&&millis()>=micResumeTime){
    micEnabled=true;
    Serial.println("🎙 Mic re-enabled after cooldown.");
  }

  // Voice trigger logic (Gemini speech intent detection)
  if(micEnabled&&(millis()-lastTrig>VOICE_COOLDOWN_MS)&&lastRMS>RMS_TRIGGER_THRESHOLD){
    lastTrig=millis();
    bool wasScanning=(mode==SCAN);
    if(wasScanning)pScan->stop();

    int16_t*recBuf=(int16_t*)malloc(g_recBytes);
    if(!recBuf){Serial.println("❌ Memory allocation failed");return;}
    Serial.println("🎤 Voice trigger detected!");

    // Record short audio clip
    orangeMode = ORANGE_SOLID;
    size_t br = 0;
    unsigned long recStart = millis();
    while (millis() - recStart < 2000) {
      size_t bytesRead = 0;
      i2s_read(I2S_PORT, (uint8_t*)recBuf, READ_LEN * sizeof(int16_t), &bytesRead, portMAX_DELAY);
      br += bytesRead;
    }
    orangeMode = ORANGE_OFF;

    // Compute amplitude and filter empty audio
    int16_t maxAmp=0;
    for(size_t i=0;i<br/sizeof(int16_t);i++)
      if(abs(recBuf[i])>maxAmp)maxAmp=abs(recBuf[i]);
    if(maxAmp<300){Serial.println("🛑 Audio too quiet, skipped.");free(recBuf);return;}

    // Send to Gemini API
    String b64=Base64::encode((uint8_t*)recBuf,br);
    free(recBuf);
    String intent=detectIntentFromAudio(b64);
    Serial.printf("🔍 Intent detected: %s\n",intent.c_str());

    if(intent=="WAKE"){
      mode=SCAN;
      micEnabled=false;
      allowMicAutoResume=false;
      scanStartTime=millis();
      if(pScan->isScanning())pScan->stop();
      delay(100);
      pScan->start(0,nullptr);
      Serial.println("▶️ Enter SCAN mode");
    }
    else if(intent=="STOP"){
      mode=IDLE;
      Serial.println("⏸ Enter IDLE mode");
    }
    if(wasScanning&&intent!="WAKE"&&intent!="STOP"){
      pScan->start(0,nullptr);
    }
  }

  // Idle mode (LEDs off)
  if(mode==IDLE){setRGB(0,0,0);pScan->stop();delay(60);return;}

  // SCAN mode: distance-based feedback
  if(mode==SCAN){
    if(!pScan->isScanning()){
      pScan->start(0,nullptr);
      Serial.println("🔁 BLE scan running...");
    }

    // No BLE signal → slow green heartbeat flash
    if(latestRSSI == -127 || (millis() - lastSeenMs) > STALE_MS) {
      static unsigned long blinkT = 0;
      if (millis() - blinkT > 2000) {
        blinkT = millis();
        setRGB(0, 60, 0);
        delay(300);
        setRGB(0, 20, 0);
      }
      delay(60);
      return;
    }

    int rssi=median5(latestRSSI);
    if(rssi<=-72)setRGB(0,220,0);
    else if(rssi<-62)setRGB(0,0,255);
    else setRGB(255,0,0);

    static uint8_t q=0;
    static unsigned long qs=0,refr=0;
    if(millis()<refr){delay(20);return;}

    // Trigger melody when treasure signal quality is high
    if(rssi>=QL_LOW&&rssi<=QL_HIGH){
      if(q==0||millis()-qs>QL_WINDOW_MS){q=1;qs=millis();}
      else{q++;if(q>=QL_HITS){playFoundTone();q=0;refr=millis()+500;}}
    }else if(q>0&&millis()-qs>QL_WINDOW_MS)q=0;

    // Final found condition
    if(rssi>=TH_FOUND&&(millis()-scanStartTime>3000)){
      playFoundTone();
      pScan->stop();
      mode=IDLE;
      micEnabled=false;
      allowMicAutoResume=true;
      micResumeTime=millis()+5000;
      Serial.println("🏁 Treasure found! Mic will re-enable in 5s.");
      refr=millis()+500;
      return;
    }

    // Distance-based buzzer feedback
    if(rssi<TH_SILENT)delay(60);
    else if(rssi<TH_SLOW){beep(30);delay(900);}
    else if(rssi<TH_MED){beep(40);delay(650);}
    else if(rssi<TH_FAST){beep(45);delay(180);beep(45);delay(280);}
    else if(rssi<TH_VFAST){for(int i=0;i<3;i++){beep(55);delay(140);}}
    else{unsigned long t=millis();while(millis()-t<450){beep(35);delay(75);}}
  }
}
