//#define ILI9341  // ILI9341 use PIN19
#define JLX12864G

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <SPIFFS.h>


#include "DHTesp.h"
#define DHTPIN 32
#define DHTTYPE DHT11
DHTesp dht;

#define PPIN 27
#define SWDETECT_PIN 25
#define POWERDOWN_PIN 4


#include "SPI.h"
// #include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <RotaryEncoder.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
// #include <Fonts/DSEG7Classic_Regular30pt7b.h>

// ILI9341
#define TFT_RST 16
#define TFT_DC 17
#define TFT_CS 5
#define TFT_BACKLIGHT_PIN 15 

#define SLEEPTIME 300 

#define WAKEUP_TOUCH_THRESHOLD 50 // 
struct LABEL {
  int x1;
  int y1;
  int xlength;
  int ylength;
  int fgcolor;
  int bgcolor;
  String text;
  int scale;
  GFXfont font;
};

const int WiFiSetPin = 15; // SW3  push with boot : AP mode on
const char* settings = "/ozon_settings.txt";
// AP mode WiFi Setting
const char* defaultWifiMode = "APMode";
const char* defaultSsid = "OZON";
const char* defaultSensitivity="-46";
const char* defaultZeroVOffset="-43.21";
const char* defaultEssKey = "11111111";
String wifiMode;
String ssid;
String essKey;
String channel;
String sensorModeStr;
String sensorSensitivityStr;
String sensorZeroVOffsetStr;
int   sensorZeroVOffset;
float sensorSensitivity;

//  P control

float PtargetPpm = 10.0;
float PcontrolArea = 0.5; // target ± 50%
String Pstatus="";




int WiFiSet = 0;  // 0 ... normal operation / 1 ... WiFi Setting Mode

RTC_DATA_ATTR int bootCount = 0;
// ILI9341
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC,TFT_RST);

static int touchvaluebefore = 0;
static int touchvalue = 0;
static float oldtemp=0;


#include <SPI.h>

#define LCD_CS           5
#define LCD_RS          17
#define LCD_RSET        16
#define SPI_CLK         18


//    Font Data
const char Font[192][5] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, // " " 0x20
    { 0x00, 0x00, 0x4f, 0x00, 0x00 }, // !   0x21
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, // "   0x22
    { 0x14, 0x7f, 0x14, 0x7f, 0x14 }, // #   0x23
    { 0x24, 0x2a, 0x7f, 0x2a, 0x12 }, // $   0x24
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, // %   0x25
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, // &   0x26
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, // '   0x27
    { 0x00, 0x1c, 0x22, 0x41, 0x00 }, // (   0x28
    { 0x00, 0x41, 0x22, 0x1c, 0x00 }, // )   0x29
    { 0x14, 0x08, 0x3e, 0x08, 0x14 }, // *   0x2A
    { 0x08, 0x08, 0x3e, 0x08, 0x08 }, // +   0x2B
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, // ,   0x2C
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, // -   0x2D
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, // .   0x2E
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, // /   0x2F
    { 0x3e, 0x51, 0x49, 0x45, 0x3e }, // 0   0x30
    { 0x00, 0x42, 0x7f, 0x40, 0x00 }, // 1   0x31
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, // 2   0x32
    { 0x21, 0x41, 0x45, 0x4b, 0x31 }, // 3   0x33
    { 0x18, 0x14, 0x12, 0x7f, 0x10 }, // 4   0x34
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5   0x35
    { 0x3c, 0x4a, 0x49, 0x49, 0x30 }, // 6   0x36
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, // 7   0x37
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8   0x38
    { 0x06, 0x49, 0x49, 0x29, 0x1e }, // 9   0x39
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, // :   0x3A
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, // ;   0x3B
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, // <   0x3C
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, // =   0x3D
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, // >   0x3E
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, // ?   0x3F
    { 0x32, 0x49, 0x79, 0x41, 0x3e }, // @   0x40
    { 0x7e, 0x11, 0x11, 0x11, 0x7e }, // A   0x41
    { 0x7f, 0x49, 0x49, 0x49, 0x36 }, // B   0x42
    { 0x3e, 0x41, 0x41, 0x41, 0x22 }, // C   0x43
    { 0x7f, 0x41, 0x41, 0x22, 0x1c }, // D   0x44
    { 0x7f, 0x49, 0x49, 0x49, 0x41 }, // E   0x45
    { 0x7f, 0x09, 0x09, 0x09, 0x01 }, // F   0x46
    { 0x3e, 0x41, 0x49, 0x49, 0x7a }, // G   0x47
    { 0x7f, 0x08, 0x08, 0x08, 0x7f }, // H   0x48
    { 0x00, 0x41, 0x7f, 0x41, 0x00 }, // I   0x49
    { 0x20, 0x40, 0x41, 0x3f, 0x01 }, // J   0x4A
    { 0x7f, 0x08, 0x14, 0x22, 0x41 }, // K   0x4B
    { 0x7f, 0x40, 0x40, 0x40, 0x40 }, // L   0x4C
    { 0x7f, 0x02, 0x0c, 0x02, 0x7f }, // M   0x4D
    { 0x7f, 0x04, 0x08, 0x10, 0x7f }, // N   0x4E
    { 0x3e, 0x41, 0x41, 0x41, 0x3e }, // O   0x4F
    { 0x7f, 0x09, 0x09, 0x09, 0x06 }, // P   0X50
    { 0x3e, 0x41, 0x51, 0x21, 0x5e }, // Q   0X51
    { 0x7f, 0x09, 0x19, 0x29, 0x46 }, // R   0X52
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, // S   0X53
    { 0x01, 0x01, 0x7f, 0x01, 0x01 }, // T   0X54
    { 0x3f, 0x40, 0x40, 0x40, 0x3f }, // U   0X55
    { 0x1f, 0x20, 0x40, 0x20, 0x1f }, // V   0X56
    { 0x3f, 0x40, 0x38, 0x40, 0x3f }, // W   0X57
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, // X   0X58
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, // Y   0X59
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, // Z   0X5A
    { 0x00, 0x7f, 0x41, 0x41, 0x00 }, // [   0X5B
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, // "\" 0X5C
    { 0x00, 0x41, 0x41, 0x7f, 0x00 }, // ]   0X5D
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, // ^   0X5E
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, // _   0X5F
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, // `   0X60
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, // a   0X61
    { 0x7f, 0x48, 0x44, 0x44, 0x38 }, // b   0X62
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, // c   0X63
    { 0x38, 0x44, 0x44, 0x48, 0x7f }, // d   0X64
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, // e   0X65
    { 0x08, 0x7e, 0x09, 0x01, 0x02 }, // f   0X66
    { 0x0c, 0x52, 0x52, 0x52, 0x3e }, // g   0X67
    { 0x7f, 0x08, 0x04, 0x04, 0x78 }, // h   0X68
    { 0x00, 0x44, 0x7d, 0x40, 0x00 }, // i   0X69
    { 0x20, 0x40, 0x44, 0x3d, 0x00 }, // j   0X6A
    { 0x7f, 0x10, 0x28, 0x44, 0x00 }, // k   0X6B
    { 0x00, 0x41, 0x7f, 0x40, 0x00 }, // l   0X6C
    { 0x7c, 0x04, 0x18, 0x04, 0x78 }, // m   0X6D
    { 0x7c, 0x08, 0x04, 0x04, 0x78 }, // n   0X6E
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, // o   0X6F
    { 0x7c, 0x14, 0x14, 0x14, 0x08 }, // p   0X70
    { 0x08, 0x14, 0x14, 0x18, 0x7c }, // q   0X71
    { 0x7c, 0x08, 0x04, 0x04, 0x08 }, // r   0X72
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, // s   0X73
    { 0x04, 0x3f, 0x44, 0x40, 0x20 }, // t   0X74
    { 0x3c, 0x40, 0x40, 0x20, 0x7c }, // u   0X75
    { 0x1c, 0x20, 0x40, 0x20, 0x1c }, // v   0X76
    { 0x3c, 0x40, 0x30, 0x40, 0x3c }, // w   0X77
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, // x   0X78
    { 0x0c, 0x50, 0x50, 0x50, 0x3c }, // y   0X79
    { 0x44, 0x64, 0x54, 0x4c, 0x44 }, // z   0X7A
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, // {   0X7B
    { 0x00, 0x00, 0x7f, 0x00, 0x00 }, // |   0X7C
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, // }   0X7D
    { 0x08, 0x08, 0x2a, 0x1c, 0x08 }, // ->  0X7E
    { 0x08, 0x1c, 0x2a, 0x08, 0x08 }, // <-  0X7F
    { 0x08, 0x46, 0x4a, 0x32, 0x1e }, // ta  0x80
    { 0x0a, 0x4a, 0x3e, 0x09, 0x08 }, // ti  0x81
    { 0x0e, 0x00, 0x4e, 0x20, 0x1e }, // tu  0x82
    { 0x04, 0x45, 0x3d, 0x05, 0x04 }, // te  0x83
    { 0x00, 0x7f, 0x08, 0x10, 0x00 }, // to  0x84
    { 0x44, 0x24, 0x1f, 0x04, 0x04 }, // na  0x85
    { 0x40, 0x42, 0x42, 0x42, 0x40 }, // ni  0x86
    { 0x42, 0x2a, 0x12, 0x2a, 0x06 }, // nu  0x87
    { 0x22, 0x12, 0x7b, 0x16, 0x22 }, // ne  0x88
    { 0x00, 0x40, 0x20, 0x1f, 0x00 }, // no  0x89
    { 0x78, 0x00, 0x02, 0x04, 0x78 }, // ha  0x8A
    { 0x3f, 0x44, 0x44, 0x44, 0x44 }, // hi  0x8B
    { 0x02, 0x42, 0x42, 0x22, 0x1e }, // hu  0x8C
    { 0x04, 0x02, 0x04, 0x08, 0x30 }, // he  0x8D
    { 0x32, 0x02, 0x7f, 0x02, 0x32 }, // ho  0x8E
    { 0x02, 0x12, 0x22, 0x52, 0x0e }, // ma  0x8F
    { 0x00, 0x2a, 0x2a, 0x2a, 0x40 }, // mi  0x90
    { 0x38, 0x24, 0x22, 0x20, 0x70 }, // mu  0x91
    { 0x40, 0x28, 0x10, 0x28, 0x06 }, // me  0x92
    { 0x0a, 0x3e, 0x4a, 0x4a, 0x4a }, // mo  0x93
    { 0x04, 0x7f, 0x04, 0x14, 0x0c }, // ya  0x94
    { 0x40, 0x42, 0x42, 0x7e, 0x40 }, // yu  0x95
    { 0x4a, 0x4a, 0x4a, 0x4a, 0x7e }, // yo  0x96
    { 0x04, 0x05, 0x45, 0x25, 0x1c }, // ra  0x97
    { 0x0f, 0x40, 0x20, 0x1f, 0x00 }, // ri  0x98
    { 0x7c, 0x00, 0x7e, 0x80, 0x30 }, // ru  0x99
    { 0x7e, 0x40, 0x20, 0x10, 0x08 }, // re  0x9A
    { 0x7e, 0x42, 0x42, 0x42, 0x7e }, // ro  0x9B
    { 0x0e, 0x02, 0x42, 0x22, 0x1e }, // wa  0x9C
    { 0x42, 0x42, 0x40, 0x20, 0x18 }, // n   0x9D
    { 0x02, 0x04, 0x01, 0x02, 0x00 }, // "   0x9E
    { 0x07, 0x05, 0x07, 0x00, 0x00 }, // .   0x9F
  { 0x00, 0x00, 0x00, 0x00, 0x00 }, //     0xA0
  { 0x70, 0x50, 0x70, 0x00, 0x00 }, //  .  0xA1
  { 0x00, 0x00, 0x0f, 0x01, 0x01 }, //  [  0xA2
  { 0x40, 0x40, 0x78, 0x00, 0x00 }, //  ]  0xA3
  { 0x10, 0x20, 0x40, 0x00, 0x00 }, //  ,  0xA4
  { 0x00, 0x18, 0x18, 0x00, 0x00 }, //  .  0xA5
  { 0x0a, 0x0a, 0x4a, 0x2a, 0x1e }, // wo  0xA6
  { 0x04, 0x24, 0x34, 0x14, 0x0c }, // a   0xA7
  { 0x20, 0x10, 0x78, 0x04, 0x00 }, // i   0xA8
  { 0x18, 0x08, 0x4c, 0x48, 0x38 }, // u   0xA9
  { 0x48, 0x48, 0x78, 0x48, 0x48 }, // e   0xAA
  { 0x48, 0x28, 0x18, 0x7c, 0x08 }, // o   0xAB
  { 0x08, 0x7c, 0x08, 0x28, 0x18 }, // ya  0xAC
  { 0x40, 0x48, 0x48, 0x78, 0x40 }, // yu  0xAD
  { 0x54, 0x54, 0x54, 0x7c, 0x00 }, // yo  0xAE
  { 0x18, 0x00, 0x58, 0x40, 0x38 }, // tu  0xAF
  { 0x08, 0x08, 0x08, 0x08, 0x08 }, //  -  0xB0
  { 0x01, 0x41, 0x3d, 0x09, 0x07 }, //  a  0xB1
  { 0x20, 0x10, 0x7c, 0x02, 0x01 }, //  i  0xB2
  { 0x0e, 0x02, 0x43, 0x22, 0x1e }, //  u  0xB3
  { 0x42, 0x42, 0x7e, 0x42, 0x42 }, //  e  0xB4
  { 0x22, 0x12, 0x0a, 0x7f, 0x02 }, //  o  0xB5
  { 0x42, 0x3f, 0x02, 0x42, 0x3e }, // ka  0xB6
  { 0x0a, 0x0a, 0x7f, 0x0a, 0x0a }, // ki  0xB7
  { 0x08, 0x46, 0x42, 0x22, 0x1e }, // ku  0xB8
  { 0x04, 0x03, 0x42, 0x3e, 0x04 }, // ke  0xB9
  { 0x42, 0x42, 0x42, 0x42, 0x7e }, // ko  0xBA
  { 0x02, 0x4f, 0x22, 0x1f, 0x02 }, // sa  0xBB
  { 0x4a, 0x4a, 0x40, 0x20, 0x1c }, // si  0xBC
  { 0x42, 0x22, 0x12, 0x2a, 0x46 }, // su  0xBD
  { 0x02, 0x3f, 0x42, 0x4a, 0x46 }, // se  0xBE
  { 0x06, 0x48, 0x40, 0x20, 0x1e } // so  0xBF
};

char v_buf[128][6];

SPISettings spiSettings = SPISettings(SPI_CLK, SPI_MSBFIRST, SPI_MODE1);


void Init_LCD()
{
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS,HIGH);

    pinMode(LCD_RS, OUTPUT);
    digitalWrite(LCD_RS,HIGH);

    pinMode(LCD_RSET, OUTPUT);
    digitalWrite(LCD_RSET,LOW);
    delay(500);
    digitalWrite(LCD_RSET,HIGH);

    digitalWrite(LCD_CS,LOW);
    digitalWrite(LCD_RS,LOW);

    SPI.transfer(0xAE);
    SPI.transfer(0xA0);
    SPI.transfer(0xC8);
    SPI.transfer(0xA3);

    SPI.transfer(0x2C);
    delay(50);
    SPI.transfer(0x2E);
    delay(50);

    SPI.transfer(0x2F);

    SPI.transfer(0x23);
    SPI.transfer(0x81);
    SPI.transfer(0x50);  // nanbuwks

    SPI.transfer(0xA4);
    SPI.transfer(0x40);
    SPI.transfer(0xA6);
    SPI.transfer(0xAF);

    digitalWrite(LCD_CS,HIGH);
}

void LCD_CLS(char data)
{
    int a,b;

    digitalWrite(LCD_CS,LOW);
    for(b=0; b<8; b ++) // nanbuwks
    {
        digitalWrite(LCD_RS,LOW);
        SPI.transfer(0xB0+b);
        SPI.transfer(0x10);
        SPI.transfer(0x00);

        digitalWrite(LCD_RS,HIGH);
        for(a=0; a<128; a++)
        {
            SPI.transfer(data);
            v_buf[a][b]=data;
        }
    }
    digitalWrite(LCD_CS,HIGH);
}

//----------------------------------------------------
//  点の描画
//  int x_data  X positon   0 -> 128
//  int y_data  Y positon   0 -> 64 // nanbuwks
//  int cl      color 0: white  1:black
//----------------------------------------------------
void LCD_PSET(int x_data, int y_data, int cl)
{

    int a,b;
    char c;

//  y_data
    a=y_data >> 3; b= y_data & 0x07;
    c=0x1;
    while(b)
    {
        c <<= 1; b --;
    }

    if(cl) v_buf[x_data][a] |= c;
    else
    {
        c = ~c; v_buf[x_data][a] &= c;
    }

    digitalWrite(LCD_CS,LOW);
    digitalWrite(LCD_RS,LOW);
    SPI.transfer(0xB0+a);
    c=x_data >> 4; c |= 0x10;
    SPI.transfer(c);
    c=x_data & 0xf;
    SPI.transfer(c);
    digitalWrite(LCD_RS,HIGH);
    SPI.transfer(v_buf[x_data][a]);
    digitalWrite(LCD_CS,HIGH);
}

//----------------------------------------------------
//  Fontの描画
//  int x_data  X positon   0 -> 128
//  int y_data  Y positon   0 -> 48
//  char c_data Data
//  int cl      color 0: white  1:black
//----------------------------------------------------
void LCD_Print_C(int x_data, int y_data, char c_data, int cl,int scale)
{
    int a,b,c,d;
    char s;

    a = c_data - 0x20;
    for(b=0; b<5; b ++)
    {
        s=0x1;
        for(c=0; c<8; c ++)
        {
            d=0;
            if(Font[a][b] & s) d=1;
            if(cl == 0)
            {
                if(d) d=0;
                else d=1;
            }
            for ( int dx=0; dx<scale;dx++) {
              for ( int dy=0; dy<scale;dy++) {
                LCD_PSET(x_data*scale+dx,  y_data + c*scale+dy  ,d);
              }
            }
            
            s <<= 1;
        }
        x_data ++;
    }
    for(c=0; c<8; c ++)
    {
        d=0;
        if(cl == 0) d=1;
        LCD_PSET(x_data,y_data + c,d);
    }
}

//----------------------------------------------------
//  Strtの描画
//  int x_data  X positon   0 -> 128
//  int y_data  Y positon   0 -> 48
//  char *c_data Data
//  int cl      color 0: white  1:black
//----------------------------------------------------
void LCD_Print_Str(int x_data, int y_data, char *c_data, int cl,int scale)
{
    int a;
    a = strlen(c_data);
    while(a)
    {
        if(*c_data == 0xef)
        {
            c_data += 2;
            a -= 2;
        }
        LCD_Print_C(x_data,y_data,*c_data,cl,scale);
//        Serial.println(*c_data,HEX);
        a --; x_data += 6; c_data ++;
    }
}



LABEL notifyTitle = {0, 80,319,40,ILI9341_BLACK ,ILI9341_WHITE,"HANDY CLEAN",1,FreeSans24pt7b};
LABEL notifyTemp  = {10,200,100,40,ILI9341_YELLOW,ILI9341_BLACK ,"29 C",1,FreeSansBold24pt7b};
LABEL notifyCel  = {70,270,0,40,ILI9341_YELLOW,ILI9341_BLACK ,"o",1,FreeSansBold12pt7b};
LABEL notifyHemi  = {130,280,100,40,ILI9341_YELLOW,ILI9341_BLACK ,"78%",1,FreeSansBold24pt7b};

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


// LABEL timeLabels[3]={ timeRemain,timeRUN,timeRETURN};
LABEL setLabels[2]={ setCONNECT,setRETURN};
LABEL scanLabels[4]={ scannode1,scannode2,scannode3,scannode4};
float ppms[100];
float ppms10[10];
int counter=0;
int samplenum=20;
void labelText(LABEL label) {
  unsigned long start = micros();
  tft.fillRect(label.x1, label.y1-1, label.xlength,label.ylength+13, label.bgcolor);
  int16_t xstart,ystart;
  uint16_t w,h;
  tft.setFont( &label.font);
  tft.setTextSize(label.scale);
  tft.getTextBounds( label.text,0,0,&xstart,&ystart,&w,&h);
  int xtextbase,ytextbase;
  xtextbase = label.x1+label.xlength/2 - w/2;
  ytextbase = label.y1+label.ylength/2 + h/2;
  tft.setCursor(xtextbase, ytextbase);
  tft.setTextColor(label.fgcolor);
  tft.println(label.text);  
}

WebServer server(80);

void handleRoot(){
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
int split(String data, char delimiter, String *dst){
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));  
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

  int sensor1 = A5;  // GPIO33  ... Vgas
  int sensor2 = A6;  // GPIO34  ... Vref
  
  int offset=0;
  for ( int i = 0; i<5;i++){
    int gasValue = analogRead(sensor1);
    int refValue = analogRead(sensor2);
    offset += (gasValue - refValue);
    delay(500); 
  }
  offset = offset / 5;
  
  Serial.println(cmds[1]);

  File f = SPIFFS.open(settings, "w");
  f.println(sensorModeStr);
  f.println(cmds[1]);
  f.println(offset);
  f.close();


  // --------------------24 H wait message display out
#ifdef JLX12864G
// for JLX12864G
    LCD_Print_Str(5,49,"AGEING...",1,2);
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

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  server.send(404, "text/plain", message);
}

void setup(void){
  Serial.begin(115200);
  delay(1000);
  Serial.println("");
  Serial.println("ESXAR Program Start");
  delay(1000);
  pinMode ( WiFiSetPin, INPUT_PULLUP );

  pinMode(PPIN,OUTPUT);
  digitalWrite(PPIN,LOW); // OFF

  pinMode(TFT_BACKLIGHT_PIN,OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN,HIGH); // ON  

  WiFiSet=digitalRead(WiFiSetPin);
//   init filesystem
  bool res = SPIFFS.begin(true); // FORMAT_SPIFFS_IF_FAILED
  if (!res) {
      Serial.println("SPIFFS.begin fail");
  }
  // check port
  int i;
  delay(1000);
  // settings read
  if ((0 ==digitalRead(WiFiSetPin)) && (0 ==WiFiSet)){
    WiFiSet=1; // Setting mode  
  } else {
    WiFiSet=0;
  }
  // set file read
  File fp = SPIFFS.open(settings, "r");
  if (!fp) {
      Serial.println("open error");
  }

  sensorModeStr = fp.readStringUntil('\n'); // always "OZON"
  sensorSensitivityStr = fp.readStringUntil('\n'); // always "clientMode"
  sensorZeroVOffsetStr = fp.readStringUntil('\n');
  Serial.print("sensorSensitivity:");
  Serial.println(sensorSensitivityStr);
  Serial.print("sensorZeroVOffset:");
  Serial.println(sensorZeroVOffsetStr);
  fp.close();
  sensorModeStr.trim();
  sensorSensitivityStr.trim();
  sensorZeroVOffsetStr.trim();
  if ( 0 == sensorModeStr.compareTo("ozon") ){
  } else {
       Serial.print("SPIFFS data seems clash. Default load...");
       sensorModeStr="ozon";
       sensorSensitivityStr=defaultSensitivity;
       sensorZeroVOffsetStr=defaultZeroVOffset;
       essKey=defaultEssKey;
       File f = SPIFFS.open(settings, "w");
       f.println(sensorModeStr);
       f.println(defaultSensitivity);
       f.println(defaultZeroVOffset);
       f.close();
  }
  if ( 1 == WiFiSet ) { 
    Serial.println("WiFiMode is ON");
    //----------WiFi access point---------
    Serial.println();
    Serial.print("Configuring access point...");
    ssid=defaultSsid;
    essKey=defaultEssKey;
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
#ifdef ILI9341
  pinMode(TFT_BACKLIGHT_PIN,OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN,HIGH);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_WHITE);
  notifyTitle.text="OZON ppm";
  labelText(notifyTitle);
#endif
#ifdef JLX12864G
    SPI.begin();
    Init_LCD();
    LCD_CLS(0);
#endif

  pinMode(SWDETECT_PIN,INPUT_PULLDOWN);
  pinMode(POWERDOWN_PIN,INPUT);

  dht.setup(DHTPIN,DHTesp::DHT11);
  delay(500);
  for ( int i = 0 ; i<100; i++ ){
    ppms[i]=0;
  }
  for ( int i = 0 ; i<10; i++ ){
    ppms10[i]=0;
  }

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
  Serial.print("sensorSensitivityStr" );
  Serial.print("  " );
  Serial.print("sensorZeroVOffsetStr" );
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
//   Serial.print("temp=");
  Serial.print("oldtemp");
  Serial.print("  " );
  Serial.print("Pstatus");
  Serial.println("" );
  



}
void loop(void){
  // Handle incoming connections 
   server.handleClient();
    
  //int sensor1 = A5;  // GPIO33  ... Vgas
  //int sensor2 = A6;  // GPIO34  ... Vref
  //int batt = A7;
  int sensor1 = A3;  // SENSOR_VN  ... Vgas
  int sensor2 = A0;  // SENSOR_VP  ... Vref
  int batt = A7;
  
  int gasValue = analogRead(sensor1);
  int refValue = analogRead(sensor2);
  int battValue = analogRead(batt);
//  static int 1=0;
  if ( 0 == counter % 10 ) {
  TempAndHumidity newValues = dht.getTempAndHumidity();
    if (dht.getStatus() != 0) {
  //    Serial.println("                     DHT11 error status: " + String(dht.getStatusString()));
    } else {
      oldtemp = newValues.temperature;
    }
  }
  float ppm,ppmmax,ppmmin;

int diffValue = gasValue-refValue;
// rel measurement end
float offset   =  sensorZeroVOffsetStr.toFloat();
float Sensitivity = sensorSensitivityStr.toFloat();
float multiple =  0.81/(Sensitivity*510*1000/1000/1000);
// Serial.println("looping...");


  ppm = -1*(float(diffValue)-offset)*multiple;  // 0.15 to 2.15 is trim
  
  ppms[counter % samplenum] = ppm;
  ppms10[counter % 10] = ppm;
  float ppmsum=0;
  float ppmsumavg10=0;
  ppmmax=0;ppmmin=100;
  for ( int i = 0 ; i<samplenum; i++ ){
    ppmsum += ppms[i];
    if ( ppmmax < ppms[i] )
        ppmmax = ppms[i];
    if ( ppmmin > ppms[i] )
        ppmmin = ppms[i];
  }
  for ( int i = 0 ; i<10; i++ ){
    ppmsumavg10 += ppms10[i];
  }
  float ppmsumavg = ppmsum/samplenum;
  ppmsumavg10=ppmsumavg10/10;




// P control
 //   ppmsumavg = 11;  // debug
 float Ptrim = -2.9;  // for 10g ozone generater 
  if ( ppmsumavg - Ptrim < PtargetPpm * ( 1-PcontrolArea) )
  {
      digitalWrite(PPIN,LOW);  // ON
      Pstatus="ON";
  } else if ( ppmsumavg - Ptrim > PtargetPpm * ( 1+PcontrolArea) ) {
    digitalWrite(PPIN, HIGH);  // OFF
    Pstatus="OFF";
  } else {
    if ( 0 == counter % 5 ) {
      digitalWrite(PPIN, HIGH);  // OFF
      Pstatus="OFF";
    }
    int POffDuty = (ppmsumavg - Ptrim - PtargetPpm * ( 1-PcontrolArea )) / (2*PtargetPpm*PcontrolArea)*5 ; 
    POffDuty ++ ;  // trim
    /*
    Serial.print(counter % 10);

    Serial.print(" (");
    Serial.print(ppmsumavg);
    Serial.print(" -  ");
    Serial.print(PtargetPpm*(1-PcontrolArea));
    Serial.print(" / ");
    Serial.print(2*PtargetPpm*PcontrolArea);
    Serial.print(") *10 = ");
    Serial.print(POffDuty);
    Serial.print(" : ");
*/
       
    if ( POffDuty ==  counter % 5 ){
      digitalWrite(PPIN,LOW);  // ON
      Pstatus="ON";
      
    }
  }



  Serial.print(ppmsumavg);
  Serial.print("  " );
  int ppmsumavg10int ;
  ppmsumavg10int = ppmsumavg10*100;
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
  Serial.print( sensorSensitivityStr );
  Serial.print("  " );
  Serial.print( sensorZeroVOffsetStr );
  Serial.print("  ");
//   Serial.print("offset=");
  Serial.print(gasValue-refValue);
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
//   Serial.print("temp=");
  Serial.print(oldtemp);
  Serial.print("  " );
  Serial.print(Pstatus);
  Serial.println("" );
  

//  float ppm = (600-sensorValue1)/12;
  if ( ppmsumavg < 0 ){
    ppmsumavg = 0;  
  }
  if ( ppmsumavg > 100 ){
    ppmsumavg = 99.9;  
  }
  char charBuf[10];
  notifyTitle.text=String(ppmsumavg);
  notifyTitle.text.toCharArray(charBuf, 50);
#ifdef ILI9341
   // for ILI9341
  labelText(notifyTitle);
  notifyTemp.text=String(int(oldtemp));
  notifyTemp.text.concat(" C");
  labelText(notifyTemp);
  labelText(notifyCel);
#endif
#ifdef JLX12864G
// for JLX12864G
    sprintf(charBuf, "%4.1f", ppmsumavg );
    LCD_Print_Str(3,22,charBuf,1,3);
    LCD_Print_Str(43,27,"ppm",1,2);

    if ( 40 ==  sensorSensitivityStr.toInt() )
    { 
      LCD_Print_Str(5,49,"",1,2);
    } else {
      LCD_Print_Str(5,49,"AGEING...",1,2);      
    }

    
#endif
  if ( HIGH==digitalRead(SWDETECT_PIN)){
    Serial.print("sw detect. Powerdown...");
    digitalWrite(TFT_BACKLIGHT_PIN,LOW); // OFF 
    delay(100);
    pinMode(POWERDOWN_PIN,OUTPUT);
    digitalWrite(POWERDOWN_PIN,LOW);
    delay(1000);
      
  }

  if ( SLEEPTIME==counter){
    Serial.print("timeout. Powerdown...");
    digitalWrite(TFT_BACKLIGHT_PIN,LOW); // OFF 
    delay(100);
    pinMode(POWERDOWN_PIN,OUTPUT);
    digitalWrite(POWERDOWN_PIN,LOW);
    delay(1000);
      
  }

  delay(500);
 
  
  counter++;

}
