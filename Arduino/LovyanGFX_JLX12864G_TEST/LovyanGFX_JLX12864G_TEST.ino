#include <LovyanGFX.hpp>
#include <SPI.h>

#define LCD_CS           5
#define LCD_RS          17
#define LCD_RSET        16
#define SPI_CLK         18
#define TFT_BACKLIGHT_PIN 15

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
      cfg.freq_write = 10000000;    // 送信時のSPIクロック (最大80MHz, 80MHzを整数で割った値に丸められます)
      cfg.freq_read  = 10000000;    // 受信時のSPIクロック
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
    digitalWrite(LCD_CS,HIGH);

    pinMode(LCD_RS, OUTPUT);
    digitalWrite(LCD_RS,HIGH);

    pinMode(LCD_RSET, OUTPUT);
    digitalWrite(LCD_RSET,LOW);
    delay(500);
    digitalWrite(LCD_RSET,HIGH);

    digitalWrite(LCD_CS,LOW);
    digitalWrite(LCD_RS,LOW);

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

    digitalWrite(LCD_CS,HIGH);
}


void setup(void)
{

    
  display.init();
      
  SPI.begin();
  Init_LCD();
    
  display.setTextSize((std::max(display.width(), display.height()) + 255) >> 8);
  display.fillScreen(TFT_WHITE);
}

uint32_t count = ~0;
void loop(void)
{
  display.startWrite();
  display.setRotation(++count & 7);
  display.setColorDepth((count & 8) ? 16 : 24);
  display.setTextColor(TFT_BLACK);
  display.drawNumber(display.getRotation(), 16, 0);
  display.drawString("R", 30, 16);
  display.drawString("G", 40, 16);
  display.drawString("B", 50, 16);
  display.drawRect(30,30,display.width()-60,display.height()-60,count*7);
  display.drawFastHLine(0, 0, 10);
  display.endWrite();
}
