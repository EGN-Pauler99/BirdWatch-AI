/*
 * BIRDTRACK ESP32 - NEON GLASS UI v8.1 AUDIO + MQTT DETECTION FIX
 * ESP32 + Bluetooth A2DP Source (TG9) + Blynk + MQTT + TFT Touch
 *
 * Screens: MENU -> DASHBOARD -> VOLUME -> MQTT LOG -> POWER
 * Detection command topic: birdtrack/esp32/device01/cmd/speaker (also accepts /detection)
 * Accepted labels: pigeon, pigeonscoop, dove, wren
 * Detection starts/restarts a 20-second audio timer.
 *
 * IMPORTANT: Replace Wi-Fi and Blynk credentials below before compiling.
 */

#define BLYNK_TEMPLATE_ID   "TMPL6lN9uD75U"
#define BLYNK_TEMPLATE_NAME "BIRDTRACKESP32"
#define BLYNK_AUTH_TOKEN    "-QX5lzCXDUgR1wRAmtYRrfcBaw0VzJCS"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <BluetoothA2DPSource.h>
#include <PubSubClient.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Preferences.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define SKIP_CALIBRATION 0
#define TOUCH_INVERT_X   0
#define TOUCH_INVERT_Y   0

struct R { int16_t x, y, w, h; };
struct LogEntry { char tag; char text[48]; unsigned long t; };

enum Screen : uint8_t { SCR_MENU = 0, SCR_DASH, SCR_VOLUME, SCR_MQTT, SCR_POWER };
enum BtnId : uint8_t {
  BT_NONE = 0, BT_BACK, BT_HOME, BT_NEXT,
  BT_M_DASH, BT_M_VOLUME, BT_M_MQTT, BT_M_POWER,
  BT_SPK, BT_VOL_DN, BT_VOL_UP, BT_POWER, BT_RECAL, BT_LOG_CLEAR
};
struct Btn { uint8_t id; R r; };

const R R_BACK     = {10, 281, 110, 34};
const R R_HOME     = {185, 281, 110, 34};
const R R_NEXT     = {360, 281, 110, 34};
const R R_M_DASH   = {15, 56, 220, 78};
const R R_M_VOLUME = {245, 56, 220, 78};
const R R_M_MQTT   = {15, 142, 220, 78};
const R R_M_POWER  = {245, 142, 220, 78};
const R R_M_INFO   = {15, 228, 450, 84};
const R R_SPK      = {70, 56, 340, 64};
const R R_VOL_DN   = {20, 156, 110, 72};
const R R_VOL_UP   = {350, 156, 110, 72};
const R R_CLEAR    = {380, 48, 85, 26};
const R R_POWER    = {70, 60, 340, 100};
const R R_RECAL    = {140, 222, 200, 38};

// CONFIG: fill in your credentials locally. Do not share tokens/passwords.
char ssid[] = "babygeegy";
char pass[] = "123456789";

const char* MQTT_SERVER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWORD = "";

char MQTT_CLIENT_ID[40] = "";
const char* TOPIC_STATUS = "birdtrack/esp32/device01/status";
const char* TOPIC_SPEAKER = "birdtrack/esp32/device01/cmd/speaker";
const char* TOPIC_VOLUME = "birdtrack/esp32/device01/cmd/volume";
const char* TOPIC_SPEAKER_STATE = "birdtrack/esp32/device01/state/speaker";
const char* TOPIC_VOLUME_STATE = "birdtrack/esp32/device01/state/volume";
const char* TOPIC_BIRD_COUNT = "birdtrack/esp32/device01/cmd/bird_count";
const char* TOPIC_DETECTION = "birdtrack/esp32/device01/detection";

// Bluetooth and tone test
BluetoothA2DPSource a2dp_source;
const char* speaker_name = "TG9";
constexpr float SAMPLE_RATE = 44100.0f;
constexpr float TONE_FREQUENCY = 2000.0f;
constexpr int16_t TONE_VOLUME = 8000;
volatile bool speakerEnabled = false; // stays silent until manual ON or bird trigger
volatile bool systemPower = true;
volatile int volumePercent = 50;

// Detection / 20-second timer
int birdCount = 0;
int previousBirdCount = 0;
bool deterrentActive = false;
bool deterrentPending = false;
unsigned long deterrentStartTime = 0;
const int BIRD_TRIGGER_COUNT = 5;
const unsigned long DETERRENT_DURATION = 20000UL;
const char* TARGET_BIRDS[] = {"pigeon", "pigeonscoop", "dove", "wren"};
const size_t TARGET_BIRD_COUNT = sizeof(TARGET_BIRDS) / sizeof(TARGET_BIRDS[0]);
char lastDetectedBird[24] = "";

TFT_eSPI tft = TFT_eSPI();
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
BlynkTimer timer;
Preferences prefs;
uint16_t calData[5];

#define COLOR_BG       0x0841
#define COLOR_CARD     0x10A2
#define COLOR_HEADER   0x18E3
#define COLOR_NAV      0x2124
#define COLOR_TEXT     TFT_WHITE
#define COLOR_MUTED    0x8C71
#define COLOR_NEON_C   0x07FF
#define COLOR_NEON_G   0x07E0
#define COLOR_NEON_P   0xF81F
#define COLOR_NEON_Y   0xFFE0
#define COLOR_NEON_R   0xF800

Screen currentScreen = SCR_MENU;
bool fullRedraw = true, uiDirty = true;
unsigned long lastUITick = 0;
uint8_t pressedBtn = BT_NONE;
unsigned long pressedUntil = 0;
constexpr int MAX_BTN = 12;
Btn regBtns[MAX_BTN];
int nReg = 0;

bool touching = false;
uint8_t heldBtn = BT_NONE;
unsigned long touchStart = 0, lastRepeat = 0, lastTouchSeen = 0, lastTouchTime = 0;

constexpr int LOG_N = 5;
LogEntry mqttLogs[LOG_N];
int logCount = 0, logIndex = 0;
bool logDirty = true;

unsigned long lastWiFiAttempt = 0;
unsigned long lastBlynkAttempt = 0;
unsigned long lastMQTTAttempt = 0;
unsigned long blynkRetryDelay = 3000;
unsigned long mqttRetryDelay = 3000;
const unsigned long WIFI_RETRY_MS = 15000;
const unsigned long RETRY_MAX_MS = 30000;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 30000;
unsigned long wifiConnectStarted = 0;

bool blynkWasConnected = false;
bool mqttWasConnected = false;
bool processingMQTTCommand = false;
bool stateNeedsPublish = true;
bool blynkNeedsUpdate = false;
unsigned long lastStatePublish = 0;
unsigned long lastBlynkFlush = 0;

// AUDIO CALLBACK: continuous stereo sine test tone, fades smoothly on/off.
// Keep this callback non-blocking: no Serial, delay, or network operations.
int32_t get_sound_data(Frame *frame, int32_t frame_count) {
  static float phase = 0.0f;
  static float gain = 0.0f;
  const float phaseStep = 2.0f * PI * TONE_FREQUENCY / SAMPLE_RATE;
  const bool enabled = speakerEnabled && systemPower;
  const float targetGain = enabled ? ((float)volumePercent / 100.0f) : 0.0f;

  for (int32_t i = 0; i < frame_count; ++i) {
    // Faster smooth ramp so the tone becomes audible promptly.
    gain += (targetGain - gain) * 0.0035f;
    if (fabsf(targetGain - gain) < 0.00001f) gain = targetGain;

    int16_t sample = 0;
    if (enabled && gain > 0.001f) {
      sample = (int16_t)(sinf(phase) * (float)TONE_VOLUME * gain);
    }

    phase += phaseStep;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    frame[i].channel1 = sample;
    frame[i].channel2 = sample;
  }
  return frame_count;
}

// -------------------- LOG --------------------
void addLog(char tag, const char* fmt, ...) {
  char tmp[44];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  LogEntry& e = mqttLogs[logIndex];
  e.tag = tag;
  snprintf(e.text, sizeof(e.text), "%s", tmp);
  e.t = millis() / 1000;
  logIndex = (logIndex + 1) % LOG_N;
  if (logCount < LOG_N) logCount++;
  logDirty = true;
}

// -------------------- BUTTON REGISTRY --------------------
void regBtn(uint8_t id, const R& r) {
  if (nReg < MAX_BTN) { regBtns[nReg].id = id; regBtns[nReg].r = r; nReg++; }
}
uint8_t hitTest(int x, int y) {
  uint8_t best = BT_NONE;
  long bestD = 0x7FFFFFFF;
  const int m = 4;
  for (int i = 0; i < nReg; i++) {
    const R& r = regBtns[i].r;
    if (x >= r.x-m && x < r.x+r.w+m && y >= r.y-m && y < r.y+r.h+m) {
      long dx = x - (r.x + r.w/2), dy = y - (r.y + r.h/2);
      long d = dx*dx + dy*dy;
      if (d < bestD) { bestD = d; best = regBtns[i].id; }
    }
  }
  return best;
}

// -------------------- DRAW HELPERS --------------------
uint16_t dimColor(uint16_t c) { return (c >> 1) & 0x7BEF; }
uint16_t blend565(uint16_t a, uint16_t b, int t) {
  int ar=(a>>11)&31, ag=(a>>5)&63, ab=a&31;
  int br=(b>>11)&31, bg=(b>>5)&63, bb=b&31;
  int rr=ar+((br-ar)*t)/255, gg=ag+((bg-ag)*t)/255, bl=ab+((bb-ab)*t)/255;
  return (uint16_t)((rr<<11)|(gg<<5)|bl);
}
void printAt(int x, int y, const String& text, uint16_t fg, uint16_t bg, int padW, uint8_t size) {
  tft.setTextColor(fg,bg); tft.setTextSize(size); tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(padW); tft.drawString(text,x,y); tft.setTextPadding(0);
}
String mmss(unsigned long s) {
  char b[12]; snprintf(b,sizeof(b),"%02lu:%02lu",(s/60)%100,s%60); return String(b);
}
void drawGlowCard(const R& r, uint16_t color, const char* title) {
  tft.fillRoundRect(r.x,r.y,r.w,r.h,10,COLOR_CARD);
  tft.drawRoundRect(r.x,r.y,r.w,r.h,10,color);
  tft.drawRoundRect(r.x+1,r.y+1,r.w-2,r.h-2,9,dimColor(color));
  if (title && title[0]) {
    tft.setTextColor(color,COLOR_CARD); tft.setTextSize(1); tft.setTextDatum(TL_DATUM);
    tft.drawString(title,r.x+12,r.y+8);
  }
}
void drawNeonButton(const R& r,int radius,const char* label,uint8_t size,uint16_t color,bool solid,bool pressed) {
  bool filled=solid||pressed;
  uint16_t fill=pressed?TFT_WHITE:(solid?color:COLOR_CARD);
  uint16_t fg=filled?TFT_BLACK:color;
  tft.fillRoundRect(r.x,r.y,r.w,r.h,radius,fill);
  tft.drawRoundRect(r.x,r.y,r.w,r.h,radius,color);
  if (!filled) tft.drawRoundRect(r.x+1,r.y+1,r.w-2,r.h-2,radius>1?radius-1:radius,dimColor(color));
  tft.setTextSize(size); tft.setTextColor(fg,fill); tft.setTextDatum(MC_DATUM);
  tft.drawString(label,r.x+r.w/2,r.y+r.h/2+1); tft.setTextDatum(TL_DATUM);
}
void iconBars(int cx,int cy,uint16_t c) {
  tft.fillRect(cx-20,cy+2,10,16,c); tft.fillRect(cx-5,cy-8,10,26,c); tft.fillRect(cx+10,cy-18,10,36,c);
}
void iconChat(int cx,int cy,uint16_t c) {
  tft.drawRoundRect(cx-20,cy-16,40,28,6,c); tft.drawRoundRect(cx-19,cy-15,38,26,5,c);
  tft.fillTriangle(cx-10,cy+12,cx-2,cy+12,cx-12,cy+22,c);
  for(int i=-1;i<=1;i++) tft.fillCircle(cx+i*10,cy-2,2,c);
}
void iconSpeaker(int cx,int cy,uint16_t c) {
  tft.fillRect(cx-20,cy-6,8,12,c);
  tft.fillTriangle(cx-12,cy-6,cx+2,cy-18,cx+2,cy+18,c);
  tft.fillTriangle(cx-12,cy-6,cx-12,cy+6,cx+2,cy+18,c);
  tft.drawFastVLine(cx+8,cy-8,16,c); tft.drawFastVLine(cx+9,cy-8,16,c);
  tft.drawFastVLine(cx+15,cy-14,28,c); tft.drawFastVLine(cx+16,cy-14,28,c);
}
void iconPower(int cx,int cy,uint16_t c) {
  tft.drawCircle(cx,cy,16,c); tft.drawCircle(cx,cy,15,c);
  tft.fillRect(cx-6,cy-20,12,12,COLOR_CARD); tft.fillRect(cx-2,cy-20,4,18,c);
}

// -------------------- HEADER / NAV --------------------
void drawHeader(const char* title) {
  tft.fillRect(0,0,480,45,COLOR_HEADER);
  for(int x=0;x<480;x+=6) tft.fillRect(x,43,6,2,blend565(COLOR_NEON_C,COLOR_NEON_P,(x*255)/479));
  tft.setTextColor(COLOR_NEON_C,COLOR_HEADER); tft.setTextSize(2); tft.setTextDatum(TL_DATUM);
  tft.drawString(title,15,14);
  if(currentScreen!=SCR_MENU) {
    char b[8]; snprintf(b,sizeof(b),"%d/4",(int)currentScreen);
    tft.setTextColor(COLOR_MUTED,COLOR_HEADER); tft.setTextSize(1); tft.drawString(b,282,19);
  }
}
void updateHeaderDots(bool force) {
  static int lastMask=-1;
  const bool st[4]={WiFi.status()==WL_CONNECTED,Blynk.connected(),mqttClient.connected(),a2dp_source.is_connected()};
  int mask=0; for(int i=0;i<4;i++) if(st[i]) mask|=(1<<i);
  if(!force&&mask==lastMask) return; lastMask=mask;
  const char letters[4]={'W','B','M','T'};
  for(int i=0;i<4;i++) {
    int cx=330+i*34; uint16_t col=st[i]?COLOR_NEON_G:COLOR_NEON_R;
    tft.fillRect(cx-13,8,26,29,COLOR_HEADER); tft.fillCircle(cx,22,10,col); tft.drawCircle(cx,22,10,TFT_WHITE);
    char b[2]={letters[i],0}; tft.setTextColor(TFT_BLACK,col); tft.setTextSize(1); tft.setTextDatum(MC_DATUM);
    tft.drawString(b,cx,23); tft.setTextDatum(TL_DATUM);
  }
}
void drawNavBar() {
  tft.fillRect(0,275,480,45,COLOR_NAV); tft.drawFastHLine(0,275,480,COLOR_NEON_C);
  drawNeonButton(R_BACK,8,"< BACK",2,COLOR_NEON_P,false,false);
  drawNeonButton(R_HOME,8,"HOME",2,COLOR_NEON_C,true,false);
  drawNeonButton(R_NEXT,8,"NEXT >",2,COLOR_NEON_C,false,false);
  regBtn(BT_BACK,R_BACK); regBtn(BT_HOME,R_HOME); regBtn(BT_NEXT,R_NEXT);
}

// -------------------- MENU --------------------
void drawMenuCard(const R& r,uint16_t color,const char* title,const char* sub,int iconId) {
  drawGlowCard(r,color,"");
  tft.setTextColor(color,COLOR_CARD); tft.setTextSize(2); tft.setTextDatum(TL_DATUM); tft.drawString(title,r.x+16,r.y+16);
  tft.setTextSize(1); tft.setTextColor(COLOR_MUTED,COLOR_CARD); tft.drawString(sub,r.x+16,r.y+52);
  int cx=r.x+r.w-48,cy=r.y+r.h/2;
  if(iconId==0)iconBars(cx,cy,color); else if(iconId==1)iconSpeaker(cx,cy,color); else if(iconId==2)iconChat(cx,cy,color); else iconPower(cx,cy,color);
}
void drawMenuStatic() {
  tft.fillScreen(COLOR_BG); drawHeader("BIRDTRACK SYSTEM");
  drawMenuCard(R_M_DASH,COLOR_NEON_C,"DASHBOARD","[ SYSTEM MONITOR ]",0);
  drawMenuCard(R_M_VOLUME,COLOR_NEON_G,"VOLUME","[ SPEAKER CONTROL ]",1);
  drawMenuCard(R_M_MQTT,COLOR_NEON_Y,"MQTT LOG","[ MESSAGE STREAM ]",2);
  drawMenuCard(R_M_POWER,COLOR_NEON_R,"POWER","[ MASTER SWITCH ]",3);
  drawGlowCard(R_M_INFO,COLOR_NEON_P,"SYSTEM INFO");
  regBtn(BT_M_DASH,R_M_DASH); regBtn(BT_M_VOLUME,R_M_VOLUME); regBtn(BT_M_MQTT,R_M_MQTT); regBtn(BT_M_POWER,R_M_POWER);
}
void updateMenu(bool force) {
  (void)force;
  bool wifiOk=WiFi.status()==WL_CONNECTED;
  String l1=String("IP ")+(wifiOk?WiFi.localIP().toString():String("-"))+"     RSSI "+(wifiOk?String(WiFi.RSSI()):String("0"))+" dBm";
  String l2=String("Uptime ")+mmss(millis()/1000)+"     Heap "+String(ESP.getFreeHeap()/1024)+" KB     Logs "+String(logCount);
  String l3=String("Blynk ")+(Blynk.connected()?"ON":"OFF")+"     MQTT "+(mqttClient.connected()?"ON":"OFF")+"     Bluetooth "+(a2dp_source.is_connected()?"LINKED":"SEARCHING");
  printAt(27,250,l1,COLOR_TEXT,COLOR_CARD,420,1); printAt(27,266,l2,COLOR_TEXT,COLOR_CARD,420,1);
  printAt(27,282,l3,COLOR_TEXT,COLOR_CARD,420,1); printAt(27,298,"Tap a card to open - HOME returns here",COLOR_MUTED,COLOR_CARD,420,1);
}

// -------------------- DASHBOARD --------------------
void drawDashboardStatic() {
  tft.fillScreen(COLOR_BG); drawHeader("DASHBOARD"); drawNavBar();
  drawGlowCard({15,56,215,98},COLOR_NEON_C,"NETWORK");
  tft.setTextColor(COLOR_MUTED,COLOR_CARD); tft.setTextSize(1); tft.setTextDatum(TL_DATUM);
  tft.drawString("WiFi",27,78);tft.drawString("IP",27,96);tft.drawString("RSSI",27,114);tft.drawString("SSID",27,132);
  drawGlowCard({245,56,220,98},COLOR_NEON_G,"BLUETOOTH (TG9)");
  tft.setTextColor(COLOR_MUTED,COLOR_CARD);tft.setTextSize(1);
  tft.drawString("Link",257,78);tft.drawString("Speaker",257,96);tft.drawString("Volume",257,114);tft.drawRoundRect(255,134,200,12,5,COLOR_TEXT);
  drawGlowCard({15,162,450,106},COLOR_NEON_Y,"CLOUD & SYSTEM");
  tft.setTextColor(COLOR_MUTED,COLOR_CARD);tft.setTextSize(1);
  tft.drawString("Blynk",27,180);tft.drawString("MQTT",177,180);tft.drawString("Uptime",327,180);
  tft.drawString("System",27,222);tft.drawString("Heap",177,222);tft.drawString("Logs",327,222);
}
void updateDashboard(bool force) {
  static int lastVol=-1; bool wifiOk=WiFi.status()==WL_CONNECTED;
  printAt(75,78,wifiOk?"ONLINE":"OFFLINE",wifiOk?COLOR_NEON_G:COLOR_NEON_R,COLOR_CARD,140,1);
  printAt(75,96,wifiOk?WiFi.localIP().toString():String("-"),COLOR_TEXT,COLOR_CARD,140,1);
  printAt(75,114,wifiOk?String(WiFi.RSSI())+" dBm":String("0 dBm"),COLOR_TEXT,COLOR_CARD,140,1);
  printAt(75,132,wifiOk?String(ssid):String("-"),COLOR_TEXT,COLOR_CARD,140,1);
  bool bt=a2dp_source.is_connected();
  printAt(315,78,bt?"CONNECTED":"SEARCHING...",bt?COLOR_NEON_G:COLOR_NEON_Y,COLOR_CARD,140,1);
  bool audioOn=speakerEnabled&&systemPower;
  printAt(315,96,!systemPower?"MUTED (PWR OFF)":(speakerEnabled?"ON":"OFF"),audioOn?COLOR_NEON_G:COLOR_NEON_R,COLOR_CARD,140,1);
  printAt(315,114,String((int)volumePercent)+"%",COLOR_TEXT,COLOR_CARD,140,1);
  if(force||lastVol!=volumePercent) {
    lastVol=volumePercent;tft.fillRect(256,135,198,10,COLOR_CARD);
    tft.fillRect(
  256,
  135,
  (198 * volumePercent) / 100,
  10,
  audioOn ? COLOR_NEON_C : COLOR_MUTED
);
  }
  printAt(27,192,Blynk.connected()?"ON":"OFF",Blynk.connected()?COLOR_NEON_G:COLOR_NEON_R,COLOR_CARD,130,2);
  printAt(177,192,mqttClient.connected()?"ON":"OFF",mqttClient.connected()?COLOR_NEON_G:COLOR_NEON_R,COLOR_CARD,130,2);
  printAt(327,192,mmss(millis()/1000),COLOR_TEXT,COLOR_CARD,130,2);
  printAt(27,234,systemPower?"ON":"OFF",systemPower?COLOR_NEON_G:COLOR_NEON_R,COLOR_CARD,130,2);
  printAt(177,234,String(ESP.getFreeHeap()/1024)+" KB",COLOR_TEXT,COLOR_CARD,130,2);
  printAt(327,234,String(logCount),COLOR_TEXT,COLOR_CARD,130,2);
}

// -------------------- VOLUME --------------------
void drawVolumeStatic() {
  tft.fillScreen(COLOR_BG);drawHeader("VOLUME CONTROL");drawNavBar();
  tft.setTextColor(COLOR_NEON_Y,COLOR_BG);tft.setTextSize(2);tft.setTextDatum(MC_DATUM);
  tft.drawString("VOLUME LEVEL",240,140);tft.setTextDatum(TL_DATUM);
  regBtn(BT_SPK,R_SPK);regBtn(BT_VOL_DN,R_VOL_DN);regBtn(BT_VOL_UP,R_VOL_UP);
}
void updateVolume(bool force) {
  static uint32_t lastSig=0xFFFFFFFF;
  uint32_t sig=(speakerEnabled?1:0)|(systemPower?2:0)|((uint32_t)volumePercent<<2)|((uint32_t)pressedBtn<<10);
  if(!force&&sig==lastSig)return;lastSig=sig;
  drawNeonButton(R_SPK,14,speakerEnabled?"SPEAKER ON":"SPEAKER OFF",3,speakerEnabled?COLOR_NEON_G:COLOR_NEON_R,speakerEnabled,pressedBtn==BT_SPK);
  drawNeonButton(R_VOL_DN,16,"-",5,COLOR_NEON_R,false,pressedBtn==BT_VOL_DN);
  drawNeonButton(R_VOL_UP,16,"+",5,COLOR_NEON_G,false,pressedBtn==BT_VOL_UP);
  tft.fillRect(136,156,208,72,COLOR_BG);
  char b[8];snprintf(b,sizeof(b),"%d%%",(int)volumePercent);
  tft.setTextColor(COLOR_TEXT,COLOR_BG);tft.setTextSize(4);tft.setTextDatum(MC_DATUM);tft.drawString(b,240,178);tft.setTextDatum(TL_DATUM);
  tft.drawRoundRect(145,208,190,14,6,COLOR_TEXT);tft.fillRect(147,210,(186*volumePercent)/100,10,(speakerEnabled&&systemPower)?COLOR_NEON_C:COLOR_MUTED);
  tft.fillRect(0,240,480,30,COLOR_BG);
  if(!systemPower){tft.setTextColor(COLOR_NEON_R,COLOR_BG);tft.setTextSize(1);tft.setTextDatum(MC_DATUM);tft.drawString("SYSTEM POWER IS OFF - audio muted",240,255);tft.setTextDatum(TL_DATUM);}
}

// -------------------- MQTT LOG --------------------
void drawMqttStatic() {
  tft.fillScreen(COLOR_BG);drawHeader("MQTT LOG");drawNavBar();
  tft.setTextColor(COLOR_NEON_Y,COLOR_BG);tft.setTextSize(1);tft.setTextDatum(TL_DATUM);
  tft.drawString("Last 5 MQTT messages (newest first)",20,56);
  regBtn(BT_LOG_CLEAR,R_CLEAR);logDirty=true;
}
void updateMqtt(bool force) {
  static uint32_t lastSig=0xFFFFFFFF;uint32_t sig=(uint32_t)pressedBtn;
  if(!force&&!logDirty&&sig==lastSig)return;lastSig=sig;logDirty=false;
  drawNeonButton(R_CLEAR,6,"CLEAR",1,COLOR_NEON_P,false,pressedBtn==BT_LOG_CLEAR);
  for(int i=0;i<LOG_N;i++) {
    int y=82+i*38;tft.fillRoundRect(15,y,450,32,6,COLOR_CARD);
    if(i>=logCount){tft.drawRoundRect(15,y,450,32,6,dimColor(COLOR_NEON_C));continue;}
    int idx=(logIndex-1-i+LOG_N)%LOG_N;const LogEntry& e=mqttLogs[idx];
    uint16_t col=COLOR_NEON_C;const char* tagText="TX";
    if(e.tag=='R'){col=COLOR_NEON_G;tagText="RX";}else if(e.tag=='E'){col=COLOR_NEON_Y;tagText="EV";}else if(e.tag=='X'){col=COLOR_NEON_R;tagText="ER";}
    tft.drawRoundRect(15,y,450,32,6,col);tft.setTextSize(1);tft.setTextDatum(TL_DATUM);
    tft.setTextColor(col,COLOR_CARD);tft.drawString(tagText,26,y+12);
    tft.setTextColor(COLOR_MUTED,COLOR_CARD);tft.drawString(mmss(e.t),55,y+12);
    tft.setTextColor(col,COLOR_CARD);tft.drawString(e.text,105,y+12);
  }
}

// -------------------- POWER --------------------
void drawPowerStatic() {
  tft.fillScreen(COLOR_BG);drawHeader("POWER CONTROL");drawNavBar();
  tft.setTextColor(COLOR_TEXT,COLOR_BG);tft.setTextSize(1);tft.setTextDatum(MC_DATUM);
  tft.drawString("Tap the big button to toggle system power",240,180);
  tft.setTextColor(COLOR_MUTED,COLOR_BG);tft.drawString("(mutes audio without rebooting)",240,198);tft.setTextDatum(TL_DATUM);
  drawNeonButton(R_RECAL,10,"RECALIBRATE TOUCH",1,COLOR_NEON_Y,false,false);
  regBtn(BT_POWER,R_POWER);regBtn(BT_RECAL,R_RECAL);
}
void updatePower(bool force) {
  static uint32_t lastSig=0xFFFFFFFF;uint32_t sig=(systemPower?1:0)|((uint32_t)pressedBtn<<1);
  if(!force&&sig==lastSig)return;lastSig=sig;
  drawNeonButton(R_POWER,16,systemPower?"SYSTEM ON":"SYSTEM OFF",3,systemPower?COLOR_NEON_G:COLOR_NEON_R,systemPower,pressedBtn==BT_POWER);
}

// -------------------- UI DISPATCH --------------------
void drawScreenStatic() {
  nReg=0;
  switch(currentScreen){case SCR_MENU:drawMenuStatic();break;case SCR_DASH:drawDashboardStatic();break;case SCR_VOLUME:drawVolumeStatic();break;case SCR_MQTT:drawMqttStatic();break;case SCR_POWER:drawPowerStatic();break;}
}
void updateUI(bool force) {
  updateHeaderDots(force);
  switch(currentScreen){case SCR_MENU:updateMenu(force);break;case SCR_DASH:updateDashboard(force);break;case SCR_VOLUME:updateVolume(force);break;case SCR_MQTT:updateMqtt(force);break;case SCR_POWER:updatePower(force);break;}
}
void goScreen(Screen s){currentScreen=s;pressedBtn=BT_NONE;fullRedraw=true;}
Screen nextScreen(Screen s){return (Screen)((s+1)%5);}
Screen prevScreen(Screen s){return (Screen)((s+4)%5);}

// -------------------- STATE / NETWORK --------------------
void publishStatus() {
  if(!mqttClient.connected())return;
  bool wifiConnected=WiFi.status()==WL_CONNECTED;
  String ip=wifiConnected?WiFi.localIP().toString():String("disconnected");
  int rssi=wifiConnected?WiFi.RSSI():0;
  char payload[240];
  snprintf(payload,sizeof(payload),"{\"wifi\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"speaker\":%s,\"power\":%s,\"volume\":%d,\"bird\":\"%s\",\"timer\":%s}",
    wifiConnected?"connected":"disconnected",ip.c_str(),rssi,speakerEnabled?"true":"false",systemPower?"true":"false",(int)volumePercent,lastDetectedBird,deterrentActive?"true":"false");
  mqttClient.publish(TOPIC_STATUS,payload,true);
}
void publishCurrentState(bool logIt) {
  if(!mqttClient.connected()){stateNeedsPublish=true;return;}
  char volumeText[8];snprintf(volumeText,sizeof(volumeText),"%d",(int)volumePercent);
  mqttClient.publish(TOPIC_SPEAKER_STATE,speakerEnabled?"1":"0",true);
  mqttClient.publish(TOPIC_VOLUME_STATE,volumeText,true);publishStatus();
  if(logIt)addLog('T',"state spk=%d vol=%d",speakerEnabled?1:0,(int)volumePercent);
  stateNeedsPublish=false;
}
void updateBlynkState() {
  if(!Blynk.connected())return;
  Blynk.virtualWrite(V0,WiFi.status()==WL_CONNECTED?1:0);
  if(WiFi.status()==WL_CONNECTED){Blynk.virtualWrite(V1,WiFi.localIP().toString());Blynk.virtualWrite(V2,WiFi.RSSI());}
  Blynk.virtualWrite(V3,speakerEnabled?1:0);Blynk.virtualWrite(V4,volumePercent);
}
void applySpeakerState(bool enabled) {
  speakerEnabled=enabled&&systemPower;
  blynkNeedsUpdate=true;stateNeedsPublish=true;uiDirty=true;
  addLog('E',"speaker %s",speakerEnabled?"ON":"OFF");
  Serial.printf("[AUDIO] Speaker command: %s | BT connected=%s | power=%s | volume=%d%%\n",
    speakerEnabled?"ON":"OFF",a2dp_source.is_connected()?"YES":"NO",systemPower?"ON":"OFF",volumePercent);
}
void applyVolume(int value) {
  volumePercent=constrain(value,0,100);blynkNeedsUpdate=true;stateNeedsPublish=true;uiDirty=true;
  Serial.printf("[AUDIO] Volume=%d%%\n",volumePercent);
}
void applyPower(bool on) {
  systemPower=on;
  if(!on){speakerEnabled=false;deterrentActive=false;deterrentPending=false;}
  stateNeedsPublish=true;uiDirty=true;addLog('E',"system power %s",on?"ON":"OFF");
  Serial.printf("[POWER] System %s\n",on?"ON":"OFF");
}
void startDeterrent() {
  if(!systemPower)return;
  deterrentActive=true;deterrentPending=false;deterrentStartTime=millis();
  speakerEnabled=true;stateNeedsPublish=true;blynkNeedsUpdate=true;uiDirty=true;
  addLog('E',"DETERRENT ON 20s");
  Serial.printf("[BIRD] Audio ON for 20 seconds | BT=%s | volume=%d%%\n",a2dp_source.is_connected()?"CONNECTED":"NOT CONNECTED",volumePercent);
}
void stopDeterrent() {
  deterrentActive=false;speakerEnabled=false;stateNeedsPublish=true;blynkNeedsUpdate=true;uiDirty=true;
  addLog('E',"DETERRENT OFF");Serial.println("[BIRD] Timer complete; audio OFF");
}
void updateDeterrent() {
  if(deterrentActive&&millis()-deterrentStartTime>=DETERRENT_DURATION){
    stopDeterrent();
    if(deterrentPending&&systemPower){deterrentPending=false;startDeterrent();}
  }
}
bool isTargetBird(const char* name) {
  if(!name||!*name)return false;
  while(*name==' '||*name=='\t')name++;
  for(size_t i=0;i<TARGET_BIRD_COUNT;i++)if(strcasecmp(name,TARGET_BIRDS[i])==0)return true;
  return false;
}
void trimText(char* s) {
  if(!s)return;
  size_t n=strlen(s);
  while(n&&(s[n-1]==' '||s[n-1]=='\t'||s[n-1]=='\r'||s[n-1]=='\n'))s[--n]='\0';
  size_t start=0;while(s[start]==' '||s[start]=='\t'||s[start]=='\r'||s[start]=='\n')start++;
  if(start)memmove(s,s+start,strlen(s+start)+1);
}
bool parseBirdDetection(const char* input,char* birdName,size_t birdNameSize) {
  if(!input||!birdName||birdNameSize<2)return false;
  birdName[0]='\0';
  while(*input==' '||*input=='\t'||*input=='\r'||*input=='\n')input++;
  if(*input!='{'){
    strncpy(birdName,input,birdNameSize-1);birdName[birdNameSize-1]='\0';trimText(birdName);
    return isTargetBird(birdName);
  }
  // JSON commands must explicitly report detected=true.
  const char* detectedKey=strstr(input,"\"detected\"");
  if(!detectedKey)return false;
  const char* colon=strchr(detectedKey,':');
  if(!colon)return false;
  colon++;while(*colon==' '||*colon=='\t')colon++;
  if(strncmp(colon,"true",4)!=0 && strncmp(colon,"1",1)!=0)return false;

  const char* keys[]={"\"name\"","\"class\"","\"label\"","\"object\""};
  for(size_t i=0;i<sizeof(keys)/sizeof(keys[0]);i++){
    const char* key=strstr(input,keys[i]);if(!key)continue;
    const char* keyColon=strchr(key,':');if(!keyColon)continue;
    const char* q=strchr(keyColon,'\"');if(!q)continue;q++;
    const char* qEnd=strchr(q,'\"');if(!qEnd||qEnd==q)continue;
    size_t n=(size_t)(qEnd-q);if(n>=birdNameSize)n=birdNameSize-1;
    memcpy(birdName,q,n);birdName[n]='\0';trimText(birdName);
    return isTargetBird(birdName);
  }
  return false;
}
void triggerNamedBird(const char* birdName) {
  if(!systemPower){addLog('E',"bird %s; power OFF",birdName);Serial.printf("[BIRD] %s detected, system power OFF\n",birdName);return;}
  strncpy(lastDetectedBird,birdName,sizeof(lastDetectedBird)-1);lastDetectedBird[sizeof(lastDetectedBird)-1]='\0';
  deterrentActive=true;deterrentPending=false;deterrentStartTime=millis();
  speakerEnabled=true;stateNeedsPublish=true;blynkNeedsUpdate=true;uiDirty=true;
  addLog('E',"BIRD %s ON 20s",birdName);
  Serial.printf("[BIRD] TARGET=%s | AUDIO ON | timer reset 20s | BT=%s | vol=%d%%\n",birdName,a2dp_source.is_connected()?"CONNECTED":"NOT CONNECTED",volumePercent);
}

// -------------------- MQTT --------------------
void mqttCallback(char* topic,byte* payload,unsigned int length) {
  char message[256];if(length==0||length>=sizeof(message))return;
  memcpy(message,payload,length);message[length]='\0';
  const char* shortTopic=strrchr(topic,'/');shortTopic=shortTopic?shortTopic+1:topic;
  addLog('R',"%s = %s",shortTopic,message);
  processingMQTTCommand=true;

  // JSON bird-detection commands are accepted on BOTH the speaker command
  // topic (used by CiRA CORE) and the legacy detection topic.
  if(strcmp(topic,TOPIC_DETECTION)==0 || strcmp(topic,TOPIC_SPEAKER)==0){
    char birdName[24];
    if(parseBirdDetection(message,birdName,sizeof(birdName))){
      triggerNamedBird(birdName);
    }else if(strcmp(topic,TOPIC_SPEAKER)==0){
      // Preserve manual ON/OFF commands on cmd/speaker.
      if(!strcasecmp(message,"1")||!strcasecmp(message,"ON"))applySpeakerState(true);
      else if(!strcasecmp(message,"0")||!strcasecmp(message,"OFF"))applySpeakerState(false);
      else Serial.printf("[BIRD] JSON/command ignored: %s\n",message);
    }else{
      Serial.printf("[BIRD] Detection ignored: %s\n",message);
    }
  }else if(strcmp(topic,TOPIC_VOLUME)==0){
    char* endPtr=nullptr;long value=strtol(message,&endPtr,10);
    if(endPtr!=message&&*endPtr=='\0'&&value>=0&&value<=100)applyVolume((int)value);
  }else if(strcmp(topic,TOPIC_BIRD_COUNT)==0){
    char* endPtr=nullptr;long value=strtol(message,&endPtr,10);
    if(endPtr!=message&&*endPtr=='\0'&&value>=0){
      previousBirdCount=birdCount;birdCount=(int)value;Serial.printf("[BIRD] Count=%d\n",birdCount);
      if(birdCount>BIRD_TRIGGER_COUNT&&previousBirdCount<=BIRD_TRIGGER_COUNT){
        if(deterrentActive){deterrentPending=true;Serial.println("[BIRD] Count trigger queued");}
        else startDeterrent();
      }
    }
  }
  processingMQTTCommand=false;
  if(stateNeedsPublish&&mqttClient.connected())publishCurrentState(true);
}
bool connectMQTT() {
  if(WiFi.status()!=WL_CONNECTED){Serial.println("[MQTT] Waiting for Wi-Fi");return false;}
  if(mqttClient.connected())return true;
  addLog('E',"MQTT connecting");
  bool connected=MQTT_USER[0]?mqttClient.connect(MQTT_CLIENT_ID,MQTT_USER,MQTT_PASSWORD):mqttClient.connect(MQTT_CLIENT_ID);
  if(connected){
    bool s1=mqttClient.subscribe(TOPIC_SPEAKER,1),s2=mqttClient.subscribe(TOPIC_VOLUME,1);
    bool s3=mqttClient.subscribe(TOPIC_BIRD_COUNT,1),s4=mqttClient.subscribe(TOPIC_DETECTION,1);
    mqttRetryDelay=3000;mqttWasConnected=true;stateNeedsPublish=true;addLog('E',"MQTT connected");
    if(!s1||!s2||!s3||!s4)addLog('X',"MQTT subscribe fail");
    Serial.printf("[MQTT] Subscribed detection: %s\n",s4?"OK":"FAIL");
    publishCurrentState(true);return true;
  }
  int rc=mqttClient.state();const char* reason="unknown";
  switch(rc){case -4:reason="timeout";break;case -3:reason="connection lost";break;case -2:reason="TCP connect failed";break;case -1:reason="disconnected";break;case 1:reason="protocol rejected";break;case 2:reason="client ID rejected";break;case 3:reason="server unavailable";break;case 4:reason="bad username/password";break;case 5:reason="not authorized";break;}
  Serial.printf("[MQTT] Connect failed rc=%d (%s)\n",rc,reason);addLog('X',"MQTT rc=%d %s",rc,reason);
  mqttRetryDelay=min(mqttRetryDelay*2,RETRY_MAX_MS);return false;
}

BLYNK_CONNECTED(){blynkWasConnected=true;blynkRetryDelay=3000;updateBlynkState();}
BLYNK_WRITE(V3){if(processingMQTTCommand)return;applySpeakerState(param.asInt()!=0);}
BLYNK_WRITE(V4){if(processingMQTTCommand)return;applyVolume(param.asInt());}
void sendStatus(){if(WiFi.status()!=WL_CONNECTED)return;updateBlynkState();if(mqttClient.connected())publishCurrentState(false);}

// -------------------- TOUCH CALIBRATION --------------------
void runCalibration() {
  tft.fillScreen(COLOR_BG);tft.setTextColor(COLOR_TEXT,COLOR_BG);tft.setTextSize(2);tft.setTextDatum(MC_DATUM);
  tft.drawString("TOUCH CALIBRATION",240,140);tft.setTextSize(1);tft.setTextColor(COLOR_MUTED,COLOR_BG);
  tft.drawString("Touch each corner target firmly",240,172);tft.setTextDatum(TL_DATUM);delay(900);
  uint16_t a,b;while(tft.getTouch(&a,&b))delay(20);delay(300);
  tft.calibrateTouch(calData,TFT_MAGENTA,COLOR_BG,15);prefs.putBytes("cal8",calData,sizeof(calData));tft.setTouch(calData);
  tft.fillScreen(COLOR_BG);tft.setTextColor(COLOR_NEON_G,COLOR_BG);tft.setTextSize(2);tft.setTextDatum(MC_DATUM);
  tft.drawString("CALIBRATION SAVED",240,160);tft.setTextDatum(TL_DATUM);delay(900);
  touching=false;heldBtn=BT_NONE;lastTouchTime=millis();
}
void bootSplash() {
  tft.fillScreen(COLOR_BG);tft.setTextDatum(MC_DATUM);tft.setTextColor(COLOR_NEON_C,COLOR_BG);tft.setTextSize(4);
  tft.drawString("BIRDTRACK",240,130);tft.setTextSize(1);tft.setTextColor(COLOR_MUTED,COLOR_BG);
  tft.drawString("Touch the screen now to recalibrate touch",240,190);tft.setTextDatum(TL_DATUM);
  unsigned long t0=millis();bool touched=false;
  while(millis()-t0<2000){uint16_t a,b;if(tft.getTouch(&a,&b)){touched=true;break;}delay(20);}
  if(touched)runCalibration();
}
void initTouch() {
  prefs.begin("bt",false);
#if SKIP_CALIBRATION
  uint16_t d[5]={300,3600,300,3600,(uint16_t)((TOUCH_INVERT_X?2:0)|(TOUCH_INVERT_Y?4:0))};tft.setTouch(d);
#else
  if(prefs.getBytes("cal8",calData,sizeof(calData))==sizeof(calData)){tft.setTouch(calData);bootSplash();}
  else runCalibration();
#endif
}

// -------------------- TOUCH --------------------
void onPress(uint8_t id) {
  switch(id){
    case BT_BACK:goScreen(prevScreen(currentScreen));return;
    case BT_HOME:goScreen(SCR_MENU);return;
    case BT_NEXT:goScreen(nextScreen(currentScreen));return;
    case BT_M_DASH:goScreen(SCR_DASH);return;
    case BT_M_VOLUME:goScreen(SCR_VOLUME);return;
    case BT_M_MQTT:goScreen(SCR_MQTT);return;
    case BT_M_POWER:goScreen(SCR_POWER);return;
    case BT_SPK:applySpeakerState(!speakerEnabled);break;
    case BT_VOL_DN:applyVolume(volumePercent-5);break;
    case BT_VOL_UP:applyVolume(volumePercent+5);break;
    case BT_POWER:applyPower(!systemPower);break;
    case BT_LOG_CLEAR:logCount=0;logIndex=0;logDirty=true;break;
    case BT_RECAL:runCalibration();fullRedraw=true;return;
    default:return;
  }
  pressedBtn=id;pressedUntil=millis()+120;uiDirty=true;
}
void handleTouch() {
  unsigned long now=millis();uint16_t x,y;
  if(!tft.getTouch(&x,&y)){
    if(touching&&now-lastTouchSeen>80){touching=false;heldBtn=BT_NONE;}return;
  }
  lastTouchSeen=now;
  if(!touching){
    if(now-lastTouchTime<120)return;
    touching=true;touchStart=now;lastRepeat=now;lastTouchTime=now;heldBtn=hitTest(x,y);
    Serial.printf("Touch x=%u y=%u -> btn %u\n",x,y,heldBtn);
    if(heldBtn!=BT_NONE)onPress(heldBtn);
  }else if(heldBtn==BT_VOL_DN||heldBtn==BT_VOL_UP){
    if(now-touchStart>400&&now-lastRepeat>110&&hitTest(x,y)==heldBtn){lastRepeat=now;onPress(heldBtn);}
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);delay(300);
  tft.init();tft.setRotation(1);initTouch();
  drawScreenStatic();fullRedraw=false;updateUI(true);

  uint64_t chipId=ESP.getEfuseMac();
  snprintf(MQTT_CLIENT_ID,sizeof(MQTT_CLIENT_ID),"birdtrack_%06lX",(unsigned long)(chipId&0xFFFFFF));
  Serial.printf("[MQTT] Client ID: %s\n",MQTT_CLIENT_ID);

  WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(true);WiFi.persistent(false);
  WiFi.begin(ssid,pass);wifiConnectStarted=millis();lastWiFiAttempt=millis();
  Blynk.config(BLYNK_AUTH_TOKEN);
  mqttClient.setServer(MQTT_SERVER,MQTT_PORT);mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(256);mqttClient.setKeepAlive(30);mqttClient.setSocketTimeout(2);
  Serial.printf("[MQTT] Broker: %s:%u\n",MQTT_SERVER,MQTT_PORT);

  // Start the A2DP source with the continuous audio callback.
  a2dp_source.start(speaker_name,get_sound_data);
  Serial.printf("[AUDIO] A2DP source started; target speaker=%s\n",speaker_name);
  timer.setInterval(5000L,sendStatus);
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now=millis();
  handleTouch();

  // Avoid repeatedly calling WiFi.reconnect while the STA is already connecting.
  if(WiFi.status()!=WL_CONNECTED){
    if(mqttClient.connected())mqttClient.disconnect();
    mqttWasConnected=false;
    if(now-wifiConnectStarted>WIFI_CONNECT_TIMEOUT_MS&&now-lastWiFiAttempt>=WIFI_RETRY_MS){
      lastWiFiAttempt=now;wifiConnectStarted=now;
      Serial.println("[WiFi] Retry begin()");
      WiFi.disconnect(false,false);delay(50);WiFi.begin(ssid,pass);
    }
  }else{
    wifiConnectStarted=now;

    if(Blynk.connected()){
      Blynk.run();if(!blynkWasConnected){blynkWasConnected=true;blynkRetryDelay=3000;}
    }else{
      blynkWasConnected=false;
      if(now-lastBlynkAttempt>=blynkRetryDelay){
        lastBlynkAttempt=now;
        if(Blynk.connect(1000)){blynkRetryDelay=3000;blynkWasConnected=true;}
        else blynkRetryDelay=min(blynkRetryDelay*2,RETRY_MAX_MS);
      }
    }

    if(mqttClient.connected()){
      mqttClient.loop();if(!mqttWasConnected){mqttWasConnected=true;mqttRetryDelay=3000;}
    }else{
      if(mqttWasConnected){mqttWasConnected=false;stateNeedsPublish=true;addLog('X',"MQTT disconnected");}
      if(now-lastMQTTAttempt>=mqttRetryDelay){lastMQTTAttempt=now;connectMQTT();}
    }
  }

  timer.run();
  updateDeterrent();

  if(blynkNeedsUpdate&&now-lastBlynkFlush>400){
    lastBlynkFlush=now;blynkNeedsUpdate=false;updateBlynkState();
  }
  if(stateNeedsPublish&&mqttClient.connected()&&now-lastStatePublish>400){
    lastStatePublish=now;publishCurrentState(true);
  }
  if(pressedBtn!=BT_NONE&&now>pressedUntil){pressedBtn=BT_NONE;uiDirty=true;}
  if(fullRedraw){
    drawScreenStatic();fullRedraw=false;lastUITick=now;uiDirty=false;updateUI(true);
  }else if(uiDirty||now-lastUITick>1000){
    lastUITick=now;uiDirty=false;updateUI(false);
  }
  delay(5);
}
