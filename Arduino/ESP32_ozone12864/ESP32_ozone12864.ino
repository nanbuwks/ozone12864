#include <LovyanGFX.hpp>
#include <SPI.h>

//#define ILI9341  // ILI9341 use PIN19
#define JLX12864G  // 128 dot x 64 dot B/W with backlight LCD


#define LCD_CS           5
#define LCD_RS          17
#define LCD_RSET        16
#define SPI_CLK         18
#define TFT_BACKLIGHT_PIN 15



#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <SPIFFS.h>


#include "DHTesp.h"
#define DHTPIN 32
#define DHTTYPE DHT11
DHTesp dht;


#define sensor1 A3  // SENSOR_VN  ... Vgas
#define sensor2 A0  // SENSOR_VP  ... Vref
#define PPIN 27
#define VIN_MONITOR_PIN A6   // GPIO34 1/2 divider voltage
#define BATT_MONITOR_PIN A7  // GPIO35 1/2 divider voltage
#define SWDETECT_PIN 25  // 
#define POWERDOWN_PIN 4  // Lo -> PowerDown
#define WiFiSetPin 14 // SW2 push with boot : AP mode on
#define UPKEY 13
#define DOWNKEY 0
#define SETKEY 14


int timeOfSleep;
int timeOfBklight;

#include "SPI.h"

hw_timer_t * timer = NULL;

long upkeylastchange = 0;
long downkeylastchange = 0;
long setkeylastchange = 0;
int upkeypushed = 0;
int downkeypushed = 0;
int setkeypushed = 0;

struct JLABEL {
  int x1;
  int y1;
  int xlength;
  int ylength;
  int fgcolor;
  int bgcolor;
  String text;
  int scale;
  lgfx::U8g2font font;
};
struct LABEL {
  int x1;
  int y1;
  int xlength;
  int ylength;
  int fgcolor;
  int bgcolor;
  String text;
  char align;
  int scale;
  GFXfont font;
};
const char* settings = "/ozon_settings.txt";
// AP mode WiFi Setting
const char* defaultWifiMode = "APMode";
const char* defaultSsid = "OZON";
const char* defaultSensitivity = "-46";
const char* defaultZeroVOffset = "-43.21";
const char* defaultEssKey = "11111111";
String wifiMode;
String ssid;
String essKey;
String channel;
String storeSensorModeStr;
String storeStrSensorSensitivity;
String storeStrSensorZeroVOffset;
String storeStrSelecttimeOfSleepsVector;
String storeStrSelectBklightTimesVector;
String storeStrSelectOutcontrolsVector;
String storeStrSelectAlermPpmVector;

int   sensorZeroVOffset;
float sensorSensitivity;

float calibLoPpm = 0; // ppm
float calibHiPpm = 10; // ppm
float calibLoValue = 0; // GasValue - RefValue
float calibHiValue = 0; // GasValue - RefValue

//  P control

float PtargetPpm = 10.0;
float PcontrolArea = 0.5; // target ± 50%
String Pstatus = "";
String modeDisp = "measure" ; // measure,setup,monitordisp
String modeSerial = "measure" ; // measure,shell

int vinValue, battValue;
int gasValue, refValue;


int WiFiSet = 0;  // 0 ... normal operation / 1 ... WiFi Setting Mode

RTC_DATA_ATTR int bootCount = 0;
// ILI9341
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC,TFT_RST);

static int touchvaluebefore = 0;
static int touchvalue = 0;
static float oldtemp = 0;

#define ILI9341_BLACK TFT_BLACK
#define ILI9341_WHITE TFT_WHITE
#define ILI9341_YELLOW TFT_YELLOW


#define BGCOLOR TFT_WHITE
#define FGCOLOR TFT_BLACK
LABEL showPPM = {7, 20, 79, 30, FGCOLOR , BGCOLOR, "   ", 'R', 1, FreeSansBold18pt7b };
//JLABEL unitPPM = {94,20,34,30,FGCOLOR, BGCOLOR,"ppm",'L',1,lgfxJapanGothicP_16};
LABEL unitPPM = {90, 20, 38, 30, FGCOLOR, BGCOLOR, "ppm", 'L', 1, FreeSansBold9pt7b};


/*
  LABEL notifyCel  = {70,270,0,40,ILI9341_YELLOW,ILI9341_BLACK ,"o",1,FreeSansBold12pt7b};
  LABEL notifyHemi  = {130,280,100,40,ILI9341_YELLOW,ILI9341_BLACK ,"78%",1,FreeSansBold24pt7b};
  LABEL notifyWiFiMode  = {130,280,100,40,ILI9341_YELLOW,ILI9341_BLACK ,"78%",1,FreeSansBold24pt7b};


  LABEL timetitle  = {  0,  0,80,50,ILI9341_BLACK,ILI9341_WHITE, "TIME:",1,FreeSansBold12pt7b};
  // LABEL timeRemain = {45,115,140,100,ILI9341_BLACK,ILI9341_WHITE,    "30",2,DSEG7Classic_Regular30pt7b};
  LABEL timemin    = {185,170, 50,40,ILI9341_BLACK,ILI9341_WHITE,   "min",1,FreeSansBold12pt7b};
  LABEL timeRUN    = { 0,280,100,40,ILI9341_BLACK,ILI9341_WHITE,   "RUN",1,FreeSansBold12pt7b};
  LABEL timeRETURN = {130,280,100,40,ILI9341_BLACK,ILI9341_WHITE,"RETURN",1,FreeSansBold12pt7b};

  LABEL settitle = {0,0,150,50,ILI9341_BLACK,ILI9341_WHITE,"DOORPLATE:",1,FreeSansBold12pt7b};
  LABEL setnode1 = {30,100,100,30,ILI9341_BLACK,ILI9341_WHITE,"PLATE(1)",1,FreeSansBold12pt7b};
  LABEL setCONNECT = {0,280,100,40,ILI9341_BLACK,ILI9341_WHITE,"SCAN",1,FreeSansBold12pt7b};
  LABEL setRETURN = {130,280,100,40,ILI9341_BLACK,ILI9341_WHITE,"RETURN",1,FreeSansBold12pt7b};


  LABEL scantitle = {0,0,150,50,ILI9341_BLACK,ILI9341_WHITE,"SCAN:",1,FreeSansBold12pt7b};
  LABEL scannode1 = {50,80,120,30,ILI9341_BLACK,ILI9341_WHITE,"PLATE(1)",1,FreeSansBold12pt7b};
  LABEL scannode2 = {50,110,120,30,ILI9341_BLACK,ILI9341_WHITE,"PLATE(2)",1,FreeSansBold12pt7b};
  LABEL scannode3 = {50,140,120,30,ILI9341_BLACK,ILI9341_WHITE,"PLATE(3)",1,FreeSansBold12pt7b};
  LABEL scannode4 = {50,170,120,30,ILI9341_BLACK,ILI9341_WHITE,"PLATE(4)",1,FreeSansBold12pt7b};
*/
/*
  JLABEL md120 = {50, 10, 80, 14,  TFT_WHITE, TFT_BLACK, "OFFTIMER", 1, lgfxJapanGothicP_8};
  JLABEL md121 = {50, 20, 80, 14,  TFT_WHITE, TFT_BLACK, "CONTROL", 1, lgfxJapanGothicP_8};
  JLABEL md122 = {50, 30, 80, 14,  TFT_WHITE, TFT_BLACK, "MONITOR", 1, lgfxJapanGothicP_8};
  JLABEL md123 = {50, 40, 80, 14,  TFT_WHITE, TFT_BLACK, "NOTIFY", 1, lgfxJapanGothicP_8};
  JLABEL md124 = {50, 50, 80, 14,  TFT_WHITE, TFT_BLACK, "RETURN", 1, lgfxJapanGothicP_8};
*/

LABEL md120 = {50, 8, 60, 7,  TFT_WHITE, TFT_BLACK, "         ", 'C' , 1, TomThumb};
LABEL md121 = {50, 16, 60, 7,  TFT_WHITE, TFT_BLACK, "         ", 'C' , 1, TomThumb};
LABEL md122 = {50, 24, 60, 7,  TFT_WHITE, TFT_BLACK, "         ", 'C' , 1, TomThumb};
LABEL md123 = {50, 32, 60, 7,  TFT_WHITE, TFT_BLACK, "         ", 'C' , 1, TomThumb};
LABEL md124 = {20, 40, 90, 7,  TFT_WHITE, TFT_BLACK, "MONITOR", 'C' , 1, TomThumb};
LABEL md125 = {20, 48, 90, 7,  TFT_WHITE, TFT_BLACK, "GRAPH", 'C' , 1, TomThumb};
LABEL md126 = {20, 56, 90, 7,  TFT_WHITE, TFT_BLACK, "RETURN", 'C', 1, TomThumb};

LABEL md110 = {10, 9, 40, 7,  TFT_BLACK, TFT_WHITE,  "BATTERY POWER", 'R' , 1, TomThumb};
LABEL md111 = {10, 17, 40, 7,  TFT_BLACK, TFT_WHITE, "BACKLIGHT OFF", 'R' , 1, TomThumb};
LABEL md112 = {10, 25, 40, 7,  TFT_BLACK, TFT_WHITE, "ALERM", 'R' , 1, TomThumb};
LABEL md113 = {10, 33, 40, 7,  TFT_BLACK, TFT_WHITE, "CONTROL", 'R', 1, TomThumb};

#define MAINTENANCELABELSNUM 7

LABEL maintenanceLabels1[ MAINTENANCELABELSNUM] = {  md120, md121, md122, md123 , md124, md125, md126};


LABEL monitorPPMsection       = {0, 0, 128,  1,  TFT_BLACK, TFT_WHITE, "", 'R' , 1, TomThumb};
LABEL monitorPPMavg           = {30 , 8, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorPPMavgtitle      = {30 , 0, 30, 8,  TFT_WHITE, TFT_BLACK, " ppmavg ", 'R' , 1, TomThumb};
LABEL monitorPPM              = {60, 8, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorPPMtitle         = {60, 0, 30, 8,  TFT_WHITE, TFT_BLACK, "  ppm  ", 'R' , 1, TomThumb};

LABEL monitorPPMmin           = {90, 16, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorPPMmintitle      = {60, 16, 30, 5,  TFT_WHITE, TFT_BLACK, "min", 'R' , 1, TomThumb};
LABEL monitorPPMmax           = {90, 21, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorPPMmaxtitle      = {60, 21, 30, 5,  TFT_WHITE, TFT_BLACK, "max", 'R' , 1, TomThumb};

LABEL monitoroffset        = {60, 33, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitoroffsettitle   = {60, 28, 30, 5,  TFT_WHITE, TFT_BLACK, "offst", 'R' , 1, TomThumb};
LABEL monitormultiple      = {90, 33, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitormultipletitle = {90, 28, 30, 5,  TFT_WHITE, TFT_BLACK, "multi", 'R' , 1, TomThumb};
LABEL monitorOffsetStr        = {60, 43, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorOffsetStrtitle   = {60, 38, 30, 5,  TFT_WHITE, TFT_BLACK, "offSt", 'R' , 1, TomThumb};
LABEL monitorSensivityStr     = {90, 43, 30, 5,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorSensivityStrtitle = {90, 38, 30, 5,  TFT_WHITE, TFT_BLACK, "mulSt", 'R' , 1, TomThumb};

LABEL monitorgasdiff          = {20, 38, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorgasdifftitle     = {00, 38, 30, 8,  TFT_WHITE, TFT_BLACK, "dif   ", 'C' , 1, TomThumb};
LABEL monitorgas              = {20, 18, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorgastitle         = {00, 18, 30, 8,  TFT_WHITE, TFT_BLACK, "gas", 'C' , 1, TomThumb};
LABEL monitorref              = {20, 28, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorreftitle         = {00, 28, 30, 8,  TFT_WHITE, TFT_BLACK, "ref", 'C' , 1, TomThumb};

LABEL monitorenvironmentsection = {0, 50, 128,  1,  TFT_BLACK, TFT_WHITE, "", 'R' , 1, TomThumb};
LABEL monitoroldtemp       = {0, 56, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitoroldtemptitle  = {0, 51, 30, 5,  TFT_WHITE, TFT_BLACK, "temp:", 'R' , 1, TomThumb};
LABEL monitorbatt             = {30, 56, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorbatttitle        = {30, 51, 30, 5,  TFT_WHITE, TFT_BLACK, "BAT:", 'R' , 1, TomThumb};
LABEL monitorvin              = {60, 56, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorvintitle         = {60, 51, 30, 5,  TFT_WHITE, TFT_BLACK, "VIN:", 'R' , 1, TomThumb};
LABEL monitorPstatus          = {90, 56, 30, 8,  TFT_BLACK, TFT_WHITE, "      ", 'R' , 1, TomThumb};
LABEL monitorPstatustitle     = {90, 51, 30, 5,  TFT_WHITE, TFT_BLACK, "P:", 'R' , 1, TomThumb};



LABEL graphPPMavg           = {0 , 56, 25, 5,  TFT_BLACK, TFT_WHITE, "      ", 'C' , 1, TomThumb};
LABEL graphPPMavgtitle      = {0, 48, 25, 5,  TFT_WHITE, TFT_BLACK, "  ppm  ", 'C' , 1, TomThumb};
LABEL graphPPM              = {25, 56, 25, 5,  TFT_BLACK, TFT_WHITE, "      ", 'C' , 1, TomThumb};
LABEL graphPPMtitle         = {25, 48, 25, 5,  TFT_WHITE, TFT_BLACK, "  ppm  ", 'C' , 1, TomThumb};
LABEL graphgasdiff          = {50, 56, 25, 5,  TFT_BLACK, TFT_WHITE, "      ", 'C' , 1, TomThumb};
LABEL graphgasdifftitle     =  {50, 48, 25, 5,  TFT_WHITE, TFT_BLACK, "dif   ", 'C' , 1, TomThumb};
LABEL graphgas              = {75, 56, 25, 5,  TFT_BLACK, TFT_WHITE, "      ", 'C' , 1, TomThumb};
LABEL graphgastitle         = {75, 48, 25, 5,  TFT_WHITE, TFT_BLACK, "gas", 'C' , 1, TomThumb};
LABEL graphref              = {100, 56, 25, 5,  TFT_BLACK, TFT_WHITE, "      ", 'C' , 1, TomThumb};
LABEL graphreftitle         = {100, 48, 25, 5,  TFT_WHITE, TFT_BLACK, "ref", 'C' , 1, TomThumb};


#define SELECTtimeOfSleepNUM 7
int selecttimeOfSleepsVector = 2; // means 5 minutes
int selecttimeOfSleeps[SELECTtimeOfSleepNUM] = {
  1, 3, 5, 10, 30, 60, -1,
};

#define SELECTBKLIGHTTIMENUM 7
int selectBklightTimesVector = 2; // means 5 minutes
int selectBklightTimes[SELECTBKLIGHTTIMENUM] = {
  1, 3, 5, 10, 30, 60, -1,
};


#define SELECTOUTCONTROLSNUM 25
int selectOutcontrolsVector = 10; // means 10 ppm
float selectOutcontrols[SELECTOUTCONTROLSNUM] = {
  0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 40, 50, 9999
};


#define SELECTALERMPPMNUM 25
int selectAlermPpmVector = 10; // means 10 ppm
float selectAlermPpms[SELECTALERMPPMNUM] = {
  0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 40, 50, 9999
};

// LABEL timeLabels[3]={ timeRemain,timeRUN,timeRETURN};
//LABEL setLabels[2]={ setCONNECT,setRETURN};
//LABEL scanLabels[4]={ scannode1,scannode2,scannode3,scannode4};
float ppms[100];
float ppms10[10];
int counter = 0;
int sleepcounter = 0;
int samplenum = 20;


unsigned short img[16 * 16];

unsigned char Icon1616BatteryFull[32] = {
  0b00011110, 0b00000000,
  0b11111111, 0b11000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b11111111, 0b11000000,
};
unsigned char Icon1616Battery3[32] = {

  0b00011110, 0b00000000,
  0b11111111, 0b11000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b11111111, 0b11000000,
};
unsigned char Icon1616Battery2[32] = {

  0b00011110, 0b00000000,
  0b11111111, 0b11000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b11111111, 0b11000000,
};
unsigned char Icon1616Battery1[32] = {

  0b00011110, 0b00000000,
  0b11111111, 0b11000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10111111, 0b01000000,
  0b10111111, 0b01000000,
  0b10000000, 0b01000000,
  0b11111111, 0b11000000,
};
unsigned char Icon1616BatteryEmpty[32] = {

  0b00011110, 0b00000000,
  0b11111111, 0b11000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b10000000, 0b01000000,
  0b11111111, 0b11000000,
};
unsigned char Icon1616ACIN[32] = {
  0b00100010, 0b00000000,
  0b00100010, 0b00000000,
  0b00100010, 0b00000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b11111111, 0b10000000,
  0b00011100, 0b00000000,
  0b00011100, 0b00000000,
  0b00001000, 0b00000000,
  0b00001000, 0b00000000,
  0b00001111, 0b10000000,
};
unsigned char Icon1616WiFi[32] = {
  0b00000000, 0b00000000,
  0b00001111, 0b10000000,
  0b00111000, 0b11100000,
  0b01100000, 0b00110000,
  0b11000000, 0b00011000,
  0b10001111, 0b10001000,
  0b00111000, 0b11100000,
  0b01100000, 0b00110000,
  0b01000000, 0b00010000,
  0b00001111, 0b10000000,
  0b00011000, 0b11000000,
  0b00000000, 0b00000000,
  0b00000111, 0b00000000,
  0b00001111, 0b10000000,
  0b00001111, 0b10000000,
  0b00000111, 0b00000000,
};


int checkattr = 0;
void IRAM_ATTR keychange() {
  sleepcounter = 0;
  int now = millis();
  // Serial.println();
  if ( 200 < now - upkeylastchange ) {
    if ( LOW == digitalRead(UPKEY)) {
      upkeypushed = 1;
      upkeylastchange = now;
    }
  }
  if ( 200 < now - downkeylastchange ) {
    if ( LOW == digitalRead(DOWNKEY)) {
      downkeypushed = 1;
      downkeylastchange = now;
    }
  }
  if ( 200 < now - setkeylastchange ) {
    if ( LOW == digitalRead(SETKEY)) {
      setkeypushed = 1;
      setkeylastchange = now;
    }
  }
}

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7565      _panel_instance;
    //lgfx::Panel_ST7735      _panel_instance;
    //lgfx::Panel_SSD1306     _panel_instance;
    lgfx::Bus_SPI        _bus_instance;   // SPIバスのインスタンス
    lgfx::Light_PWM     _light_instance;
  public:
    LGFX(void)
    {
      { // バス制御の設定を行います。
        auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。
        // SPIバスの設定
        cfg.spi_host = VSPI_HOST;     // 使用するSPIを選択  ESP32-S2,C3 : SPI2_HOST or SPI3_HOST / ESP32 : VSPI_HOST or HSPI_HOST
        // ※ ESP-IDFバージョンアップに伴い、VSPI_HOST , HSPI_HOSTの記述は非推奨になるため、エラーが出る場合は代わりにSPI2_HOST , SPI3_HOSTを使用してください。
        cfg.spi_mode = 1;             // SPI通信モードを設定 (0 ~ 3)
        cfg.freq_write = 1000000;    // 送信時のSPIクロック (最大80MHz, 80MHzを整数で割った値に丸められます)
        cfg.freq_read  = 1000000;    // 受信時のSPIクロック
        cfg.spi_3wire  = false;        // 受信をMOSIピンで行う場合はtrueを設定
        cfg.use_lock   = true;        // トランザクションロックを使用する場合はtrueを設定
        cfg.dma_channel = SPI_DMA_CH_AUTO; // 使用するDMAチャンネルを設定 (0=DMA不使用 / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=自動設定)
        // ※ ESP-IDFバージョンアップに伴い、DMAチャンネルはSPI_DMA_CH_AUTO(自動設定)が推奨になりました。1ch,2chの指定は非推奨になります。
        cfg.pin_sclk = 18;            // SPIのSCLKピン番号を設定
        cfg.pin_mosi = 23;            // SPIのMOSIピン番号を設定
        cfg.pin_miso = 19;            // SPIのMISOピン番号を設定 (-1 = disable)
        cfg.pin_dc   = 17;            // SPIのD/Cピン番号を設定  (-1 = disable)

        _bus_instance.config(cfg);    // 設定値をバスに反映します。
        _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
      }

      { // 表示パネル制御の設定を行います。
        auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。
        cfg.pin_cs           =    5;  // CSが接続されているピン番号   (-1 = disable)
        cfg.pin_rst          =    16;  // RSTが接続されているピン番号  (-1 = disable)
        cfg.pin_busy         =    -1;  // BUSYが接続されているピン番号 (-1 = disable)
        // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。
        cfg.panel_width      =   128;  // 実際に表示可能な幅
        cfg.panel_height     =   64;  // 実際に表示可能な高さ
        cfg.offset_x         =     0;  // パネルのX方向オフセット量
        cfg.offset_y         =     0;  // パネルのY方向オフセット量
        cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
        cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
        cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
        cfg.readable         =  false;  // データ読出しが可能な場合 trueに設定
        cfg.invert           = true;  // パネルの明暗が反転してしまう場合 trueに設定
        cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
        cfg.dlen_16bit       = true;  // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
        cfg.bus_shared       =  true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)
        // 以下はST7735やILI9163のようにピクセル数が可変のドライバで表示がずれる場合にのみ設定してください。
        //    cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
        //    cfg.memory_height    =   320;  // ドライバICがサポートしている最大の高さ
        _panel_instance.config(cfg);
      }
      { // バックライト制御の設定を行います。（必要なければ削除）
        auto cfg = _light_instance.config();    // バックライト設定用の構造体を取得します。

        cfg.pin_bl = 15;              // バックライトが接続されているピン番号
        cfg.invert = false;           // バックライトの輝度を反転させる場合 true
        cfg.freq   = 44100;           // バックライトのPWM周波数
        cfg.pwm_channel = 7;          // 使用するPWMのチャンネル番号

        _light_instance.config(cfg);
        _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセットします。
      }
      setPanel(&_panel_instance); // 使用するパネルをセットします。
    }
};

LGFX display;



SPISettings spiSettings = SPISettings(SPI_CLK, SPI_MSBFIRST, SPI_MODE1);




void Init_LCD()
{
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);

  pinMode(LCD_RS, OUTPUT);
  digitalWrite(LCD_RS, HIGH);

  pinMode(LCD_RSET, OUTPUT);
  digitalWrite(LCD_RSET, LOW);
  delay(500);
  digitalWrite(LCD_RSET, HIGH);

  digitalWrite(LCD_CS, LOW);
  digitalWrite(LCD_RS, LOW);

  SPI.transfer(0xAE);  // display off
  SPI.transfer(0xA0);  // ADC normal ( Column address is left to right )
  SPI.transfer(0xC8);  // line scan is reverce ( bottom to top )
  SPI.transfer(0xA3);  // LCD bias = 1/7

  SPI.transfer(0x2C);  // Voltage supply up converter on
  delay(50);
  SPI.transfer(0x2E);  // Voltage supply voltage regurator on
  delay(50);
  SPI.transfer(0x2F);  // foltage supply voltage follower on

  SPI.transfer(0x23);  // contrast rough (0x20-0x27)
  SPI.transfer(0x81);  // contrast trim command
  SPI.transfer(0x08);  // contrast trim data (0x00-0x3F)

  SPI.transfer(0xA4);  // all dot displays: normal
  SPI.transfer(0x40);  // display start line set 0 (0x40-0x7F)
  SPI.transfer(0xA7);  // display normal/reverse: normal
  SPI.transfer(0xAF);  // display on

  digitalWrite(LCD_CS, HIGH);
}

//----------------------------------------------------
//  Patternの描画
//  int x_data  X positon   0 -> 128
//  int y_data  Y positon   0 -> 64
//  char c_data Data
//  int cl      color 0: white  1:black
//----------------------------------------------------
void LCD_Pattern(int x_data, int y_data, unsigned char *Pattern_data, int cl, int scale)
{
  Serial.print("LCD_Pattern ");
  Serial.println(x_data);
  int d, y, x1, x2, counter = 0;
  char s;

  for (y = 0; y < 16; y++)
  {
    for (x1 = 0; x1 < 2; x1++)
    {
      s = 0b10000000;
      for (x2 = 0; x2 < 8; x2++)
      {
        d = 0;
        if (Pattern_data[y * 2 + x1] & s) d = 1;
        if (cl == 0)
        {
          if (d) d = 0x0000;
          else d = 0xFFFF;
        } else {
          if (d) d = 0xFFFF;
          else d = 0x0000;

        }
        Serial.print(d);
        Serial.print(" ");
        img[counter++] = d;
        s >>= 1;
      }
    }
    Serial.println();
  }
  display.pushImage(x_data, y_data, 16, 16, img);
}
void labelText(LABEL label) {

  unsigned long start = micros();

  int16_t xstart, ystart;
  uint16_t w, h;
  display.setFont( &label.font);
  display.setTextSize(label.scale);
  w =  display.textWidth( label.text);
  h =  display.fontHeight();
  /*
    Serial.println(label.x1);
    Serial.println(label.y1);
    Serial.println(label.xlength);
    Serial.println(label.ylength);
    Serial.println(label.bgcolor);
    Serial.println();
  */
  display.fillRect(label.x1, label.y1, label.xlength, label.ylength, label.bgcolor);
  int xtextbase, ytextbase;


  if ( 'L' == label.align )
    xtextbase = label.x1;
  if ( 'C' == label.align )
    xtextbase = label.x1 + label.xlength / 2 - w / 2;
  if ( 'R' == label.align ) {
    xtextbase = label.x1 + label.xlength - w;
  }
  //  ytextbase = label.y1+label.ylength/2 + h/2;
  ytextbase = label.y1;
  display.setCursor(xtextbase, ytextbase);
  display.setTextColor(label.fgcolor);
  display.println(label.text);
}
void selectLabelText(LABEL label) {
  unsigned long start = micros();
  display.fillRect(label.x1, label.y1, label.xlength, label.ylength, label.fgcolor);

  display.drawRect(label.x1, label.y1, label.xlength, label.ylength, label.bgcolor);

  int16_t xstart, ystart;
  uint16_t w, h;
  display.setFont( &label.font);
  display.setTextSize(label.scale);
  //  tft.getTextBounds( label.text,0,0,&xstart,&ystart,&w,&h);
  w = display.textWidth(label.text);
  h = display.fontHeight();
  int xtextbase, ytextbase;

  if ( 'L' == label.align )
    xtextbase = label.x1;
  if ( 'C' == label.align )
    xtextbase = label.x1 + label.xlength / 2 - w / 2;
  if ( 'R' == label.align ) {
    xtextbase = label.x1 + label.xlength - w;
  }
  //xtextbase = label.x1 + label.xlength / 2 - w / 2;
  ytextbase = label.y1;
  display.setCursor(xtextbase, ytextbase + 1);
  display.setTextColor(label.bgcolor);
  display.println(label.text);
}


int maintenanceSelect1() {
  int exitflag = 0;
  int selected;

  char charBuf[10];
  char dtostrfBuf[10];

  unsigned int selectcounter = 0;
  //  while ( 0 == exitflag )
  selectcounter = 1;
  // ダミーでキーをプッシュして最初の描画をさせる
  upkeypushed = 1;
  //  display.fillScreen(TFT_BLACK);
  display.fillScreen(TFT_WHITE);

  labelText(md110);   // title of "BATTERY POWER"
  labelText(md111);   // title of "BACKLIGHT OFF"
  labelText(md112);   // title of "ALERM"
  labelText(md113);   // title of "CONTROL"
  Serial.println("maintenanceSelect1");
  // ボタンとボタン上の表示を初期描画
  if ( -1 == selecttimeOfSleeps[selecttimeOfSleepsVector] ) {
    maintenanceLabels1[0].text = "always on";
  } else {
    sprintf(charBuf , "%d min pff" , selecttimeOfSleeps[selecttimeOfSleepsVector] );
    maintenanceLabels1[0].text = charBuf;
  }

  if ( -1 == selectBklightTimes[selectBklightTimesVector] ) {
    maintenanceLabels1[1].text = "always on";
  } else {
    sprintf(charBuf , "%d min pff" , selecttimeOfSleeps[selectBklightTimesVector] );
    maintenanceLabels1[1].text = charBuf;
  }

  if ( 9999 == selectAlermPpms[selectAlermPpmVector] ) {
    maintenanceLabels1[2].text = "always on";
  } else {
    dtostrf(selectAlermPpms[selectAlermPpmVector], 4, 1, charBuf );
    strcat(charBuf, " ppm");
    maintenanceLabels1[2].text = charBuf;
  }

  if ( 9999 == selectOutcontrols[selectOutcontrolsVector] ) {
    maintenanceLabels1[3].text = "always on";
  } else {
    dtostrf(selectOutcontrols[selectOutcontrolsVector], 4, 1, charBuf );
    strcat(charBuf, " ppm");
    maintenanceLabels1[3].text = charBuf;
  }



  while (1)
  {

    if ( 1 == upkeypushed || 1 == downkeypushed) {
      if ( 1 == upkeypushed )
        selectcounter--;
      if ( 1 == downkeypushed )
        selectcounter++;
      Serial.println(selectcounter);
      upkeypushed = 0;
      downkeypushed = 0;
      selected = (( selectcounter % MAINTENANCELABELSNUM) + MAINTENANCELABELSNUM ) % MAINTENANCELABELSNUM; // negative modulo

      Serial.print(selectcounter);    Serial.print(" ");    Serial.print(selected);    Serial.println(" ");
      for ( int i = 0; i < MAINTENANCELABELSNUM; i++ ) {
        Serial.print("selectcounter=");
        Serial.println(i);
        if ( i == selected) {
          selectLabelText(maintenanceLabels1[i]);
        } else {
          labelText(maintenanceLabels1[i]);
        }
      }
    }


    if ( 1 == setkeypushed ) {
      setkeypushed = 0;
      switch (selected) {
        case 0:  //OFFTIMER
          selecttimeOfSleepsVector++;
          if ( SELECTtimeOfSleepNUM <= selecttimeOfSleepsVector )
            selecttimeOfSleepsVector = 0;
          timeOfSleep = selecttimeOfSleeps[selecttimeOfSleepsVector] * 60;
          if ( -1 == selecttimeOfSleeps[selecttimeOfSleepsVector] ) {
            maintenanceLabels1[selected].text = "always on";
          } else {
            sprintf(charBuf , "%d min off" , selecttimeOfSleeps[selecttimeOfSleepsVector] );
            maintenanceLabels1[selected].text = charBuf;
          }
          selectLabelText(maintenanceLabels1[selected]);
          break;
        case 1:  // BackLight OFFTIMER
          selectBklightTimesVector++;
          if ( SELECTBKLIGHTTIMENUM <= selectBklightTimesVector )
            selectBklightTimesVector = 0;
          timeOfBklight = selectBklightTimes[selectBklightTimesVector] * 60;
          if ( -1 == selectBklightTimes[selectBklightTimesVector] ) {
            maintenanceLabels1[selected].text = "always on";
          } else {
            sprintf(charBuf , "%d min off" , selectBklightTimes[selectBklightTimesVector] );
            maintenanceLabels1[selected].text = charBuf;
          }
          selectLabelText(maintenanceLabels1[selected]);
          break;
        case 2:   // Alerm
          selectAlermPpmVector++;
          if ( SELECTALERMPPMNUM <= selectAlermPpmVector )
            selectAlermPpmVector = 0;
          if ( 9999 == selectAlermPpms[selectAlermPpmVector] ) {
            maintenanceLabels1[selected].text = "NO ALERM";
          } else {
            dtostrf(selectAlermPpms[selectAlermPpmVector], 4, 1, charBuf );
            strcat(charBuf, " ppm");
            maintenanceLabels1[selected].text = charBuf;
          }
          selectLabelText(maintenanceLabels1[selected]);
          break;
        case 3:
          selectOutcontrolsVector++;
          if ( SELECTOUTCONTROLSNUM <= selectOutcontrolsVector )
            selectOutcontrolsVector = 0;
          PtargetPpm = selectOutcontrols[selectOutcontrolsVector];
          if ( 9999 == selectOutcontrols[selectOutcontrolsVector] ) {
            maintenanceLabels1[selected].text = "always on";
          } else {
            dtostrf(selectOutcontrols[selectOutcontrolsVector], 4, 1, charBuf );
            strcat(charBuf, " ppm");
            maintenanceLabels1[selected].text = charBuf;
          }
          selectLabelText(maintenanceLabels1[selected]);
          break;
        case 4:
          selectLabelText(maintenanceLabels1[selected]);
          modeDisp = "monitordisp";
          return (0);
          break;
        case 5:
          selectLabelText(maintenanceLabels1[selected]);
          modeDisp = "graph";
          return (0);
          break;
        case 6:
          selectLabelText(maintenanceLabels1[selected]);
          modeDisp = "measure";
          storeData();
          return (0);
          break;

      }
    }
  }
}
WebServer server(80);

void handleRoot() {
  char temp[500];
  snprintf ( temp, 500,
             "<html>\
<head>\
<title>OZON Sensor Setting</title>\
<style>\
body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
</style>\
</head>\
<body>\
<h1>OZON Sensor Setting</h1>\
<br> \
<hr> \
<a href=\"\/setInput\">settings</a> \
</body>\
</html>");
  server.send ( 200, "text/html", temp );
}

void setInput() {
  String html = "";
  html += "<h1>OZON Sensor Input</h1>";
  html += "<form method='post' action='/sensorSet'>";
  html += "  <br>";
  html += "  <input type='text' name='qrcodedata' >Please Input QR code data<br>";
  html += "  <br>";
  html += "  <input type='submit'><br>";
  html += "</form>";
  server.send(200, "text/html", html);
}

// split function from https://algorithm.joho.info/arduino/string-split-delimiter/
int split(String data, char delimiter, String *dst) {
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if ( tmp == delimiter ) {
      index++;
      if ( index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

void sensorSet() {
  String cmds[3] = {"\0"};
  String wifiMode = "clientMode";
  String qrcodedata = server.arg("qrcodedata");
  int index = split(qrcodedata, '-', cmds);

  int offset = 0;
  for ( int i = 0; i < 5; i++) {
    int gasValue = analogRead(sensor1);
    int refValue = analogRead(sensor2);
    offset += (gasValue - refValue);
    delay(500);
  }
  offset = offset / 5;

  Serial.println(cmds[1]);
  storeStrSensorSensitivity=cmds[1];
  storeStrSensorZeroVOffset=offset;
  storeData();
  

  // --------------------24 H wait message display out
#ifdef JLX12864G
  // for JLX12864G
  //    LCD_Print_Str(5,49,"AGEING...",1,2);
#endif



  String html = "";
  html += "<h1>WiFi Settings</h1>";
  html += "Sensitivity:" + cmds[1] + "<br>";
  html += "ZeroVOffset:";
  html += offset;
  html += "<br>";
  html += "<hr>";
  html += "<h1>Please Wait 24H and Reset!</h1>";

  server.send(200, "text/html", html);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  server.send(404, "text/plain", message);
}

void storeData() {
    File f = SPIFFS.open(settings, "w");
    f.println(storeStrSensorSensitivity);
    f.println(storeStrSensorZeroVOffset);
    storeStrSelecttimeOfSleepsVector =  String(  selecttimeOfSleepsVector);
    storeStrSelectBklightTimesVector = String( selectBklightTimesVector);
    storeStrSelectOutcontrolsVector = String(  selectOutcontrolsVector);
    storeStrSelectAlermPpmVector = String(selectAlermPpmVector);
    f.println(storeStrSelecttimeOfSleepsVector);
    f.println(storeStrSelectBklightTimesVector);
    f.println(storeStrSelectOutcontrolsVector);
    f.println(storeStrSelectAlermPpmVector);

    f.println(storeSensorModeStr);
    f.close();


}

//-----------------------------------------------------------------------------------------
void setup(void) {


  Serial.begin(115200);
  delay(100);
  Serial.println("");
  Serial.println("ESXAR Program Start");
  delay(100);
  pinMode ( WiFiSetPin, INPUT_PULLUP );

  pinMode(PPIN, OUTPUT);
  digitalWrite(PPIN, LOW); // OFF

  pinMode(UPKEY, INPUT_PULLUP);
  pinMode(DOWNKEY, INPUT_PULLUP);
  pinMode(SETKEY, INPUT_PULLUP);


  display.init();
#ifdef ILI9341
  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_WHITE);
  notifyTitle.text = "OZON ppm";
  labelText(notifyTitle);
#endif
#ifdef JLX12864G
  SPI.begin();
  Init_LCD();
#endif
  display.setTextSize((std::max(display.width(), display.height()) + 255) >> 8);
  display.fillScreen(TFT_WHITE);

  WiFiSet = digitalRead(WiFiSetPin);
  //   init filesystem
  bool res = SPIFFS.begin(true); // FORMAT_SPIFFS_IF_FAILED
  if (!res) {
    Serial.println("SPIFFS.begin fail");
  }
  // check port
  int i;
  delay(1000);
  // settings read
  if ((0 == digitalRead(WiFiSetPin)) && (0 == WiFiSet)) {
    WiFiSet = 1; // Setting mode
  } else {
    WiFiSet = 0;
  }
  // set file read
  File fp = SPIFFS.open(settings, "r");
  if (!fp) {
    Serial.println("open error");
  }





  storeStrSensorSensitivity = fp.readStringUntil('\n');
  storeStrSensorZeroVOffset = fp.readStringUntil('\n');
  storeStrSelecttimeOfSleepsVector = fp.readStringUntil('\n');;
  storeStrSelectBklightTimesVector = fp.readStringUntil('\n');;
  storeStrSelectOutcontrolsVector = fp.readStringUntil('\n');;
  storeStrSelectAlermPpmVector = fp.readStringUntil('\n');;
  storeSensorModeStr = fp.readStringUntil('\n'); // always "ozone"
  Serial.print("sensorSensitivity:");
  Serial.println(storeStrSensorSensitivity);
  Serial.print("sensorZeroVOffset:");
  Serial.println(storeStrSensorZeroVOffset);
  fp.close();
  storeStrSensorSensitivity.trim();
  storeStrSensorZeroVOffset.trim();
  storeStrSelecttimeOfSleepsVector.trim();
  storeStrSelectBklightTimesVector.trim();
  storeStrSelectOutcontrolsVector.trim();
  storeStrSelectAlermPpmVector.trim();
  storeSensorModeStr.trim();
  if ( 0 == storeSensorModeStr.compareTo("ozone") ) {
    selecttimeOfSleepsVector =  (storeStrSelecttimeOfSleepsVector.toInt());
    selectBklightTimesVector =  (storeStrSelectBklightTimesVector.toInt());
    selectOutcontrolsVector =  (storeStrSelectOutcontrolsVector.toInt());
    selectAlermPpmVector =  (storeStrSelectAlermPpmVector.toInt());


  } else {
    Serial.print("SPIFFS data seems clash. Default load...");
    storeSensorModeStr = "ozone";
    storeStrSensorSensitivity = defaultSensitivity;
    storeStrSensorZeroVOffset = defaultZeroVOffset;
    essKey = defaultEssKey;
    storeData();
  }
  timeOfSleep = selecttimeOfSleeps[selecttimeOfSleepsVector] * 60;
  timeOfBklight = selectBklightTimes[selectBklightTimesVector] * 60;

  if ( 1 == WiFiSet ) {
    Serial.println("WiFiMode is ON");
    //----------WiFi access point---------
    Serial.println();
    // LCD_Pattern(0, 0, Icon1616WiFi, 0, 1);


    ssid = defaultSsid;
    essKey = defaultEssKey;
    WiFi.softAP(ssid.c_str(), essKey.c_str());
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.on("/",     handleRoot);
    server.on("/setInput",  setInput);
    server.on("/sensorSet", sensorSet);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
  }

  pinMode(SWDETECT_PIN, INPUT_PULLDOWN);
  pinMode(POWERDOWN_PIN, INPUT);

  dht.setup(DHTPIN, DHTesp::DHT11);
  delay(500);
  for ( int i = 0 ; i < 100; i++ ) {
    ppms[i] = 0;
  }
  for ( int i = 0 ; i < 10; i++ ) {
    ppms10[i] = 0;
  }

  // measure thread make
  xTaskCreateUniversal(
    task1,
    "task1",
    8192,
    NULL,
    1,
    NULL,
    APP_CPU_NUM
  );

  // power control thread make
  xTaskCreateUniversal(
    task2,
    "task2",
    8192,
    NULL,
    1,
    NULL,
    APP_CPU_NUM
  );

  // command interpriter thread make
  xTaskCreateUniversal(
    task3,
    "task3",
    8192,
    NULL,
    1,
    NULL,
    APP_CPU_NUM
  );

  //switch interrupt
  attachInterrupt(UPKEY, keychange, CHANGE);
  attachInterrupt(DOWNKEY, keychange, CHANGE);
  attachInterrupt(SETKEY, keychange, CHANGE);

  Serial.println("interrupt seted");

  // sleep timer init
  timeOfSleep = selecttimeOfSleeps[selecttimeOfSleepsVector] * 60;
  // P control value init
  PtargetPpm = selectOutcontrols[selectOutcontrolsVector];



  Serial.print("ppmsumavg");
  Serial.print("  " );
  Serial.print("ppmsumavg10int");
  Serial.print("  " );
  Serial.print("ppm");
  Serial.print("  " );
  Serial.print("ppmmin");
  Serial.print("  " );
  Serial.print("ppmmax");
  Serial.print("  " );
  Serial.print("oldtemp");
  Serial.print("  " );
  Serial.print("offset");
  Serial.print("  " );
  Serial.print("multiple");
  Serial.print("  " );
  Serial.print("storeStrSensorSensitivity" );
  Serial.print("  " );
  Serial.print("storeStrSensorZeroVOffset" );
  Serial.print("  ");
  //   Serial.print("offset=");
  Serial.print("gasValue-refValue");
  Serial.print("  ");
  //   Serial.print("gas=");
  Serial.print("gasValue");
  Serial.print("  ");
  //   Serial.print("ref=");
  Serial.print("refValue");
  Serial.print("  ");
  //   Serial.print("batt=");
  Serial.print("battValue");
  Serial.print("  ");
  //   Serial.print("vin=");
  Serial.print("vinValue");
  Serial.print("  ");
  //   Serial.print("temp=");
  Serial.print("oldtemp");
  Serial.print("  " );
  Serial.print("Pstatus");
  Serial.println("" );
}


//===============================================================================================


void loop(void) {
  // Web server handler ,  key switch handler

  server.handleClient();
  if ( 1 == setkeypushed ) {
    setkeypushed = 0;
    modeDisp = "setup";
    Serial.println("modeDisp=setup");
    maintenanceSelect1();
  }

  delay(100);

}


//===============================================================================================
// gas measurement loop

void task1(void *pvParameters) {
  char charBuf[10];
  int graph[128];
  while (1) {
    gasValue = analogRead(sensor1);
    refValue = analogRead(sensor2);
    vinValue = analogRead(VIN_MONITOR_PIN);
    battValue = analogRead(BATT_MONITOR_PIN);
    //  static int 1=0;
    if ( 0 == counter % 10 ) {
      TempAndHumidity newValues = dht.getTempAndHumidity();
      if (dht.getStatus() != 0) {
        //    Serial.println("                     DHT11 error status: " + String(dht.getStatusString()));
      } else {
        oldtemp = newValues.temperature;
      }
    }
    float ppm, ppmmax, ppmmin;

    int diffValue = gasValue - refValue;
    // rel measurement end
    float offset   =  storeStrSensorZeroVOffset.toFloat();
    float Sensitivity = storeStrSensorSensitivity.toFloat();
    float multiple =  0.81 / (Sensitivity * 510 * 1000 / 1000 / 1000);
    // Serial.println("looping...");


    ppm =  (float(diffValue) - offset) * multiple; // 0.15 to 2.15 is trim

    ppms[counter % samplenum] = ppm;
    ppms10[counter % 10] = ppm;
    float ppmsum = 0;
    float ppmsumavg10 = 0;
    ppmmax = 0; ppmmin = 100;
    for ( int i = 0 ; i < samplenum; i++ ) {
      ppmsum += ppms[i];
      if ( ppmmax < ppms[i] )
        ppmmax = ppms[i];
      if ( ppmmin > ppms[i] )
        ppmmin = ppms[i];
    }
    for ( int i = 0 ; i < 10; i++ ) {
      ppmsumavg10 += ppms10[i];
    }
    float ppmsumavg = ppmsum / samplenum;
    ppmsumavg10 = ppmsumavg10 / 10;




    // P control
    //   ppmsumavg = 11;  // debug
    float Ptrim = -0.29 * PtargetPpm;  // for 10g ozone generater
    if ( ppmsumavg - Ptrim < PtargetPpm * ( 1 - PcontrolArea) )
    {
      digitalWrite(PPIN, LOW); // ON
      Pstatus = "ON";
    } else if ( ppmsumavg - Ptrim > PtargetPpm * ( 1 + PcontrolArea) ) {
      digitalWrite(PPIN, HIGH);  // OFF
      Pstatus = "OFF";
    } else {
      if ( 0 == counter % 5 ) {
        digitalWrite(PPIN, HIGH);  // OFF
        Pstatus = "OFF";
      }
      int POffDuty = (ppmsumavg - Ptrim - PtargetPpm * ( 1 - PcontrolArea )) / (2 * PtargetPpm * PcontrolArea) * 5 ;
      POffDuty ++ ;  // trim

      //    Serial.print(counter % 10);
      //    Serial.print(" (");
      //    Serial.print(ppmsumavg);
      //    Serial.print(" -  ");
      //    Serial.print(PtargetPpm*(1-PcontrolArea));
      //    Serial.print(" / ");
      //    Serial.print(2*PtargetPpm*PcontrolArea);
      //    Serial.print(") *10 = ");
      //    Serial.print(POffDuty);
      //    Serial.print(" : ");
      //

      if ( POffDuty ==  counter % 5 ) {
        digitalWrite(PPIN, LOW); // ON
        Pstatus = "ON";

      }
    }




    if ( 0 == modeSerial.compareTo("measure")) {

      Serial.print(ppmsumavg);
      Serial.print("  " );
      int ppmsumavg10int ;
      ppmsumavg10int = ppmsumavg10 * 100;
      Serial.print(ppmsumavg10int);
      Serial.print("  " );
      Serial.print(ppm);
      Serial.print("  " );
      Serial.print(ppmmin);
      Serial.print("  " );
      Serial.print(ppmmax);
      Serial.print("  " );
      Serial.print(oldtemp);
      Serial.print("  " );
      Serial.print(offset);
      Serial.print("  " );
      Serial.print(multiple);
      Serial.print("  " );
      Serial.print( storeStrSensorSensitivity );
      Serial.print("  " );
      Serial.print( storeStrSensorZeroVOffset );
      Serial.print("  ");
      //   Serial.print("offset=");
      Serial.print(gasValue - refValue);
      Serial.print("  ");
      //   Serial.print("gas=");
      Serial.print(gasValue);
      Serial.print("  ");
      //   Serial.print("ref=");
      Serial.print(refValue);
      Serial.print("  ");
      //   Serial.print("batt=");
      Serial.print(battValue);
      Serial.print("  ");
      //   Serial.print("vin=");
      Serial.print(vinValue);
      Serial.print("  ");
      //   Serial.print("temp=");
      Serial.print(oldtemp);
      Serial.print("  " );
      Serial.print(Pstatus);
      Serial.println();
      // Serial.println(checkattr);
    }

    //  float ppm = (600-sensorValue1)/12;
    if ( ppmsumavg < 0 ) {
      ppmsumavg = 0;
    }
    if ( ppmsumavg > 100 ) {
      ppmsumavg = 99.9;
    }
    //    ppmsumavg = ppmsumavg * 20;
    //------------------------------------------------------計測値をLCDに表示---------


    if ( 0 == modeDisp.compareTo("measure")) {
      dtostrf(ppmsumavg, 4, 1, charBuf);
      //  charBuf=" 123";
      //  buffer.toCharArray(charBuf, 50);
#ifdef ILI9341
      // for ILI9341
      labelText(notifyTitle);
      notifyTemp.text = String(int(oldtemp));
      notifyTemp.text.concat(" C");
      labelText(notifyTemp);
      labelText(notifyCel);
#endif

#ifdef JLX12864G
      // for JLX12864G
      //    sprintf(charBuf, "%4.1f", ppmsumavg );
      //    LCD_Print_Str(3,22,charBuf,1,3);
      //    LCD_Print_Str(43,27,"ppm",1,2);
      display.fillScreen(TFT_WHITE);
      showPPM.text = charBuf;
      unitPPM.text = "ppm";
      labelText(showPPM);
      labelText(unitPPM);

      if ( 40 ==  storeStrSensorSensitivity.toInt() )
      {
        //     LCD_Print_Str(5,49,"",1,2);
      } else {
        //      LCD_Print_Str(5,49,"AGEING...",1,2);
      }
#endif
    }
    // -------------------------------------------------------- monitor 値をLCDに表示
    if ( 0 == modeDisp.compareTo("monitordisp")) {
      display.fillScreen(TFT_WHITE);
      dtostrf(ppmsumavg, 5, 2, charBuf);
      monitorPPMavg.text = charBuf;
      labelText(monitorPPMavg);
      labelText(monitorPPMavgtitle);
      dtostrf(ppm, 5, 2, charBuf);
      monitorPPM.text = charBuf;
      labelText(monitorPPM);
      labelText(monitorPPMtitle);
      dtostrf(ppmmin, 5, 2, charBuf);
      monitorPPMmin.text = charBuf;
      labelText(monitorPPMmin);
      labelText(monitorPPMmintitle);
      dtostrf(ppmmax, 5, 2, charBuf);
      monitorPPMmax.text = charBuf;
      labelText(monitorPPMmax);
      labelText(monitorPPMmaxtitle);
      dtostrf(oldtemp, 5, 2, charBuf);
      monitoroldtemp.text = charBuf;
      labelText(monitoroldtemp);
      labelText(monitoroldtemptitle);
      dtostrf(offset, 5, 2, charBuf);
      monitoroffset.text = charBuf;
      labelText(monitoroffset);
      labelText(monitoroffsettitle);
      dtostrf(multiple, 5, 2, charBuf);
      monitormultiple.text = charBuf;
      labelText(monitormultiple);
      labelText(monitormultipletitle);
      monitorSensivityStr.text = storeStrSensorSensitivity;
      labelText(monitorSensivityStr);
      labelText(monitorSensivityStrtitle);
      monitorOffsetStr.text = storeStrSensorZeroVOffset;
      labelText(monitorOffsetStr);
      labelText(monitorOffsetStrtitle);
      sprintf(charBuf, "%d", gasValue - refValue);
      monitorgasdiff.text = charBuf;
      labelText(monitorgasdiff);
      labelText(monitorgasdifftitle);
      sprintf(charBuf, "%d", gasValue);
      monitorgas.text = charBuf;
      labelText(monitorgas);
      labelText(monitorgastitle);
      sprintf(charBuf, "%d", refValue);
      monitorref.text = charBuf;
      labelText(monitorref);
      labelText(monitorreftitle);
      sprintf(charBuf, "%d", battValue);
      monitorbatt.text = charBuf;
      labelText(monitorbatt);
      labelText(monitorbatttitle);
      sprintf(charBuf, "%d", vinValue);
      monitorvin.text = charBuf;
      labelText(monitorvin);
      labelText(monitorvintitle);
      monitorref.text = Pstatus;
      labelText(monitorPstatus);
      labelText(monitorPstatustitle);



    }

    // -------------------------------------------------------- monitor 値をグラフ表示
    if ( 0 == modeDisp.compareTo("graph")) {
      for ( int i = 0; i < 127 ; i++ ) {
        graph[i + 1] = graph[i];
      }
      graph[0] = gasValue - refValue;

      display.fillScreen(TFT_WHITE);


      dtostrf(ppmsumavg, 5, 2, charBuf);
      graphPPMavg.text = charBuf;
      labelText(graphPPMavg);
      labelText(graphPPMavgtitle);
      dtostrf(ppm, 5, 2, charBuf);
      graphPPM.text = charBuf;
      labelText(graphPPM);
      labelText(graphPPMtitle);
      sprintf(charBuf, "%d", gasValue - refValue);
      graphgasdiff.text = charBuf;
      labelText(graphgasdiff);
      labelText(graphgasdifftitle);
      sprintf(charBuf, "%d", gasValue);
      graphgas.text = charBuf;
      labelText(graphgas);
      labelText(graphgastitle);
      sprintf(charBuf, "%d", refValue);
      graphref.text = charBuf;
      labelText(graphref);
      labelText(graphreftitle);



      for ( int i = 0; i < 128 ; i++ ) {
        display.drawLine(127 - i, 24, 127 - i, 24 + graph[i], TFT_BLACK);
      }


    }

    //
    // ---------------------------------------------------------Show Power Indicator
    if  ( 2300 < vinValue ) {
      sleepcounter = 0;
      LCD_Pattern(111, 0, Icon1616ACIN, 0, 1);
    } else if ( 1500 < battValue ) {
      LCD_Pattern(111, 0, Icon1616BatteryFull, 0, 1);
    } else if ( 1300 < battValue ) {
      LCD_Pattern(111, 0, Icon1616Battery3, 0, 1);
    } else if ( 1200 < battValue ) {
      LCD_Pattern(111, 0, Icon1616Battery2, 0, 1);
    } else if ( 1100 < battValue ) {
      LCD_Pattern(111, 0, Icon1616Battery1, 0, 1);
    } else if ( 1000 < battValue ) {
      LCD_Pattern(111, 0, Icon1616BatteryEmpty, 0, 1);
    }
    // ---------------------------------------------------------Show WiFi Icon
    if ( 1 == WiFiSet ) {

      LCD_Pattern(0, 0, Icon1616WiFi, 0, 1);
    }
    delay(1000);
  }
}

//===============================================================================================

// power control loop
void task2(void *pvParameters) {
  while (1) {

    if ( HIGH == digitalRead(SWDETECT_PIN)) {
      Serial.print("sw detect. Powerdown...");
      digitalWrite(TFT_BACKLIGHT_PIN, LOW); // OFF
      delay(100);
      pinMode(POWERDOWN_PIN, OUTPUT);
      digitalWrite(POWERDOWN_PIN, LOW);
      delay(1000);
    }

    //  static int 1=0;
    if ( timeOfSleep == sleepcounter) {
      Serial.print("timeout. Powerdown...");
      digitalWrite(TFT_BACKLIGHT_PIN, LOW); // OFF
      delay(100);
      pinMode(POWERDOWN_PIN, OUTPUT);
      digitalWrite(POWERDOWN_PIN, LOW);
      delay(1000);
    }
    

        //  static int 1=0;
    if ( timeOfBklight == sleepcounter) {
      Serial.print("backlight off...");
      digitalWrite(TFT_BACKLIGHT_PIN, LOW); // OFF
    }

delay(500);

    counter++;
    sleepcounter++;
  }
}

//=============================================================================================
void task3(void *pvParameters) {
  while (1) {
    while (Serial.available() == 0) {}     //wait for data available
    String teststr = Serial.readString();  //read until timeout
    teststr.trim();                        // remove any \r \n whitespace at the end of the String
    if (teststr == "") {
      while (teststr != "exit" ) {
        modeSerial = "shell";
        Serial.print("> ");
        while (Serial.available() == 0) {}
        teststr = Serial.readString();
        teststr.trim();
        Serial.println(teststr);


      }
    }
  }
}
