#include <TuyaWifi.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <Chrono.h>
#include "OneButton.h"

#define u16 uint16_t
#define u8 uint8_t   // Digital IO pin connected to the NeoPixels.
#define u32 uint32_t // Number of NeoPixels
//#define BUTTON_PIN PA12
#define KEY_PIN D9
#define PIXEL_PIN A6   // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 10 // Number of NeoPixels
#define MIC_PIN A0     //音频采集
//开关(可下发可上报)
//备注:
#define DPID_SWITCH_LED 20
//模式(可下发可上报)
//备注:
#define DPID_WORK_MODE 21
//亮度值(可下发可上报)
//备注:
#define DPID_BRIGHT_VALUE 22
//音乐灯(只下发)
//备注:类型：字符串；
//Value: 011112222333344445555；
//0：   变化方式，0表示直接输出，1表示渐变；
//1111：H（色度：0-360，0X0000-0X0168）；
//2222：S (饱和：0-1000, 0X0000-0X03E8）；
//3333：V (明度：0-1000，0X0000-0X03E8）；
//4444：白光亮度（0-1000）；
//5555：色温值（0-1000）
#define DPID_MUSIC_DATA 27
//调节(只下发)
//备注:类型：字符串 ;
//Value: 011112222333344445555  ;
//0：   变化方式，0表示直接输出，1表示渐变;
//1111：H（色度：0-360，0X0000-0X0168）;
//2222：S (饱和：0-1000, 0X0000-0X03E8);
//3333：V (明度：0-1000，0X0000-0X03E8);
//4444：白光亮度（0-1000）;
//5555：色温值（0-1000）
#define DPID_CONTROL_DATA 28
//炫彩情景(可下发可上报)
//备注:专门用于幻彩灯带场景
#define DPID_DREAMLIGHT_SCENE_MODE 51
//炫彩本地音乐律动(可下发可上报)
//备注:专门用于幻彩灯带外置麦克风音乐律动
#define DPID_DREAMLIGHTMIC_MUSIC_DATA 52
//点数/长度设置(可下发可上报)
//备注:幻彩灯带裁剪之后重新设置长度
#define DPID_LIGHTPIXEL_NUMBER_SET 53

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

///* Current device DP values */
unsigned char dp_bool_value = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
//unsigned char dp_string_value[21] = {"0"};
//unsigned char hex[10] = {"0"};

struct
{
  uint8_t Mode;
  uint8_t Bright;
  uint8_t Scene_Mode;
  uint8_t index;
  uint16_t PixelNum;
} RGB_LED;
struct
{
  uint32_t Color;
  uint8_t index;
} LED_Wipe;
struct
{
  uint32_t Color;
  uint8_t index;
  uint8_t a;
  uint8_t b;
  uint8_t c;
} LED_Chase;
struct
{
  uint32_t Color1;
  uint32_t Color2;
  uint8_t a;
} LED_RUN;
struct
{
  uint32_t Color;
  uint8_t index;
  uint8_t i;
  uint32_t firstPixelHue;

} LED_RainBow;
struct
{
  uint32_t Color;
  uint8_t index;
} LED_Direct;
enum
{
  OFF,
  WIPE,
  RAINBOW,
  CHASE,
  RUN_LIGHT,
  Direct_Display,
  LOCALMUSIC,
  LOCALMUSIC2,
  OTHER,
};
struct time_keeping
{
  unsigned long times_start;
  short times;
} high;
int avgs[5] = {-1};
/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] = {
    {DPID_SWITCH_LED, DP_TYPE_BOOL},
    {DPID_WORK_MODE, DP_TYPE_ENUM},
    {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
    {DPID_MUSIC_DATA, DP_TYPE_STRING},
    {DPID_CONTROL_DATA, DP_TYPE_STRING},
    {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
    {DPID_DREAMLIGHTMIC_MUSIC_DATA, DP_TYPE_RAW},
    {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
};

unsigned char pid[] = {"******"}; //*********处替换成涂鸦IoT平台自己创建的产品的PID
unsigned char mcu_ver[] = {"1.0.0"};

SoftwareSerial DebugSerial(D3, D4);
Chrono ledChrono;
Chrono ledWipe;
Chrono ledChase;
Chrono ledRainBow;
Chrono ledDirect;
Chrono ledRun;
Chrono ledMusic;
OneButton button(KEY_PIN, true);
TuyaWifi my_device;
//unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length);
//void dp_update_all(void);
void setup()
{
  //Initialize networking keys.
  pinMode(KEY_PIN, INPUT_PULLUP);
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

  DebugSerial.begin(9600);
  Serial.begin(9600);
  //Initialize led port, turn off led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //incoming all DPs and their types array, DP numbers
  //Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  my_device.set_dp_cmd_total(dp_array, 8);
  //register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  //register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);

  attachInterrupt(digitalPinToInterrupt(KEY_PIN), checkTicks, CHANGE);
  button.attachClick(singleClick);
  RGB_LED.Bright = 200;
  RGB_LED.PixelNum = PIXEL_COUNT;
}

void loop()
{
  my_device.uart_service();
  RGB_LED_Deal();
  button.tick();
  Network_LED_Blink();
}
void RGB_LED_Deal(void)
{
  switch (RGB_LED.Mode)
  {
  case OFF: //OFF
    //colorfill(strip.Color(0, 0, 0), 0);
    break;
  case WIPE:
    colorWipe(LED_Wipe.Color);
    break;

  case RAINBOW:
    rainbow();
    break;

  case CHASE:
    theaterChase(LED_Chase.Color);
    break;
  case Direct_Display:
    colorfill(LED_Direct.Color);
    break;
  case RUN_LIGHT:
    RunLight(LED_RUN.Color1, LED_RUN.Color2);
    break;
  case LOCALMUSIC:
    LocalMusicMode();
    break;
  case LOCALMUSIC2:
    LocalMusicMode2();
    break;
  case OTHER:
    break;
  default:
    break;
  }
}
void singleClick()
{
  my_device.mcu_set_wifi_mode(SMART_CONFIG);
}
void checkTicks()
{
  // include all buttons here to be checked
  button.tick(); // just call tick() to check the state.
}
void Network_LED_Blink(void)
{
  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW))
  {
    if (ledChrono.hasPassed(500))
    {
      ledChrono.restart();
      digitalToggle(LED_BUILTIN);
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void colorWipe(uint32_t color)
{
  if (ledWipe.hasPassed(50))
  {
    ledWipe.restart();
    if (LED_Wipe.index < strip.numPixels())
    {
      strip.setPixelColor(LED_Wipe.index, color); //  Set pixel's color (in RAM)
      strip.setBrightness(RGB_LED.Bright);
      strip.show(); //  Update strip to matc
      LED_Wipe.index++;
    }
    else
    {
      LED_Wipe.index = 0;
    }
  }
}
void theaterChase(uint32_t color)
{
  if (ledChase.hasPassed(200))
  {
    ledChase.restart();
    //  'b' counts from 0 to 2...
    strip.clear(); //   Set all pixels in RAM to 0 (off)
    // 'c' counts up from 'b' to end of strip in steps of 3...
    for (LED_Chase.c = LED_Chase.b; LED_Chase.c < strip.numPixels(); LED_Chase.c += 3)
    {
      strip.setPixelColor(LED_Chase.c, color); // Set pixel 'c' to value 'color'
    }
    strip.setBrightness(RGB_LED.Bright);
    strip.show(); // Update strip with new contents
    LED_Chase.b++;
    if (LED_Chase.b >= 3)
    {
      LED_Chase.b = 0;
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(void)
{
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  if (ledRainBow.hasPassed(30))
  {
    ledRainBow.restart();
    for (LED_RainBow.i = 0; LED_RainBow.i < strip.numPixels(); LED_RainBow.i++)
    { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = LED_RainBow.firstPixelHue + (LED_RainBow.i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(LED_RainBow.i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.setBrightness(RGB_LED.Bright);
    strip.show(); // Update strip with new contents
    if (LED_RainBow.firstPixelHue < 3 * 65536)
    {
      LED_RainBow.firstPixelHue += 256;
    }
    else
    {
      LED_RainBow.firstPixelHue = 0;
    }
  }
}

void LocalMusicMode(void)
{
  int n, height;
  float k;
  uint32_t c;
  uint16_t h;
  uint8_t i;
  int value, mapped, avg;
  if (ledMusic.hasPassed(10))
  {
    ledMusic.restart();
    value = analogRead(MIC_PIN);
    value = (value <= 500) ? 0 : abs(value - 500);
    if (value == 0)
      return;
    mapped = (float)fscale(0, 500, 0, (float)500, (float)value, 2.0);
    avg = compute_average(avgs, 5);
    insert(mapped, avgs, 5);
    height = fscale(0, 500, 0, (float)strip.numPixels(), (float)avg, -1);

    DebugSerial.println(height);

    if (height < 0)
      height = 0;
    for (uint8_t i = 0; i < strip.numPixels(); i++)
    {
      // turn off LEDs above the current level
      if (i >= height)
      {
        strip.setPixelColor(i, 0);
      }
      else
      {
        h = (i * 65536L / strip.numPixels());
        strip.setBrightness(RGB_LED.Bright);
        c = strip.gamma32(strip.ColorHSV(h, 250, RGB_LED.Bright));
        strip.setPixelColor(i, c);
      }
    }
    strip.show();
  }
}
void LocalMusicMode2(void)
{
  int Bright;
  float k;
  uint32_t c;
  uint16_t h;
  uint8_t i;
  uint32_t sum;
  int value, mapped, avg;
  if (ledMusic.hasPassed(10))
  {
    ledMusic.restart();
    for (i = 0; i < 20; i++)
    {
      sum += analogRead(MIC_PIN);
    }
    value = sum / 20;
    value = (value <= 500) ? 0 : abs(value - 500);
    if (value == 0)
      return;
    mapped = (float)fscale(0, 500, 0, (float)500, (float)value, 2.0);
    avg = compute_average(avgs, 5);
    insert(mapped, avgs, 5);
    Bright = fscale(0, 500, 0, 255, (float)avg, -3);
    if (Bright < 5)
      Bright = 0;
    if (Bright > 255)
      Bright = 255;
    c = strip.Color(0, 200, 200);
    strip.fill(c, 0, strip.numPixels());
    strip.setBrightness(Bright);
    strip.show();
  }
}
/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  switch (dpid)
  {
  case DPID_SWITCH_LED:

    dp_bool_value = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
    if (dp_bool_value)
    {
      //Turn on
      RGB_LED.Mode = Direct_Display;
      strip.setBrightness(RGB_LED.Bright);
      LED_Direct.Color = strip.gamma32(strip.ColorHSV(0, 0, RGB_LED.Bright));
    }
    else
    {
      //Turn off
      RGB_LED.Mode = OFF;
      strip.clear();
      strip.show();
    }
    //Status changes should be reported.
    my_device.mcu_dp_update(dpid, value, length);
    break;

  case DPID_WORK_MODE:
    RGB_LED.Mode = OTHER;
    dp_enum_value = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
    switch (dp_enum_value)
    {
    case 0: // white mode
      //colorfill(strip.Color(255, 255, 255), RGB_LED.Bright);
      break;
    case 1: // colour mode

      break;
    case 2: // scene mode

      break;
    case 3: // music mode

      break;
    }
    //Status changes should be reported.
    my_device.mcu_dp_update(dpid, value, length);
    break;

  case DPID_BRIGHT_VALUE: //亮度
    dp_value_value = my_device.mcu_get_dp_download_data(dpid, value, length);
    RGB_LED.Bright = dp_value_value;
    strip.setBrightness(RGB_LED.Bright);
    strip.show();
    my_device.mcu_dp_update(dpid, RGB_LED.Bright, length);
    break;

  case DPID_MUSIC_DATA: //音乐律动
    RGB_LED.Mode = OTHER;
    my_device.mcu_dp_update(dpid, value, length);
    colour_data_control(value, length);
    break;

  case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
    my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, value, length);
    RGB_LED.Scene_Mode = value[1];
    switch (RGB_LED.Scene_Mode)
    {
    case 0:
      RGB_LED.Mode = WIPE;
      LED_Wipe.index = 0;
      LED_Wipe.Color = strip.Color(200, 0, 0);

      break;
    case 1:
      RGB_LED.Mode = WIPE;
      LED_Wipe.index = 0;
      LED_Wipe.Color = strip.Color(255, 100, 90);
      break;
    case 2:
      RGB_LED.Mode = WIPE;
      LED_Wipe.index = 0;
      LED_Wipe.Color = strip.Color(255, 100, 5);
      break;
    case 3:
      RGB_LED.Mode = WIPE;
      LED_Wipe.index = 0;
      LED_Wipe.Color = strip.Color(0, 200, 200);
      break;
    case 4:
      RGB_LED.Mode = CHASE;
      LED_Chase.a = 0;
      LED_Chase.b = 0;
      LED_Chase.c = 0;
      LED_Chase.Color = strip.Color(200, 80, 150);
      break;
    case 5:
      RGB_LED.Mode = CHASE;
      LED_Chase.a = 0;
      LED_Chase.b = 0;
      LED_Chase.c = 0;
      LED_Chase.Color = strip.Color(127, 0, 0);
      break;
    case 6:
      RGB_LED.Mode = CHASE;
      LED_Chase.a = 0;
      LED_Chase.b = 0;
      LED_Chase.c = 0;
      LED_Chase.Color = strip.Color(0, 0, 127);
      break;
    case 7:
      RGB_LED.Mode = CHASE;
      LED_Chase.a = 0;
      LED_Chase.b = 0;
      LED_Chase.c = 0;
      LED_Chase.Color = strip.Color(0, 100, 100);
      break;
    case 8:
      LED_RainBow.firstPixelHue = 0;
      LED_RainBow.i = 0;
      RGB_LED.Mode = RAINBOW;
      break;
    case 9:
      RGB_LED.Mode = RUN_LIGHT;
      LED_RUN.Color1 = strip.gamma32(strip.ColorHSV(30000, 250, RGB_LED.Bright));
      LED_RUN.Color2 = strip.gamma32(strip.ColorHSV(60000, 250, RGB_LED.Bright));
      LED_RUN.a = 0;
      break;
    }

    break;

  case DPID_DREAMLIGHTMIC_MUSIC_DATA: //本地音乐律动
    my_device.mcu_get_dp_download_data(dpid, value, length);
    my_device.mcu_dp_update(dpid, value, length);
    switch (value[2])
    {
    case 0:
      RGB_LED.Mode = LOCALMUSIC;
      break;
    case 1:
      RGB_LED.Mode = LOCALMUSIC2;
      break;
    }

    break;
  case DPID_LIGHTPIXEL_NUMBER_SET: //长度设置
    RGB_LED.PixelNum = my_device.mcu_get_dp_download_data(dpid, value, length);
    my_device.mcu_dp_update(dpid, value, length);
    strip.clear();
    strip.show();
    LED_Wipe.index = 0;
    strip.updateLength(RGB_LED.PixelNum);
    break;
  default:
    break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_LED, dp_bool_value, 1);
  my_device.mcu_dp_update(DPID_BRIGHT_VALUE, RGB_LED.Bright, 1);
  my_device.mcu_dp_update(DPID_LIGHTPIXEL_NUMBER_SET, RGB_LED.PixelNum, 1);
}

//拓展
void colorfill(uint32_t color)
{
  strip.fill(color, 0, strip.numPixels());
  strip.show(); //  Update strip to match
}

void RunLight(uint32_t c1, uint32_t c2)
{
  uint8_t i;
  if (ledRun.hasPassed(150))
  {
    ledRun.restart();
    strip.fill(c1, 0, strip.numPixels());
    LED_RUN.a++;
    if (LED_RUN.a >= strip.numPixels())
    {
      LED_RUN.a = 0;
    }
    for (i = 0; i < 3; i++)
    {
      if ((i + LED_RUN.a) < strip.numPixels())
        strip.setPixelColor(LED_RUN.a + i, c2);
      else
        strip.setPixelColor(((i + LED_RUN.a) - strip.numPixels()), c2);
    }

    strip.show();
  }
}
/**
 * @brief  str to short
 * @param[in] {a} Single Point
 * @param[in] {b} Single Point
 * @param[in] {c} Single Point
 * @param[in] {d} Single Point
 * @return Integrated value
 * @note   Null
 */
u32 __str2short(u32 a, u32 b, u32 c, u32 d)
{
  return (a << 12) | (b << 8) | (c << 4) | (d & 0xf);
}

/**
  * @brief ASCALL to Hex
  * @param[in] {asccode} 当前ASCALL值
  * @return Corresponding value
  * @retval None
  */
u8 __asc2hex(u8 asccode)
{
  u8 ret;

  if ('0' <= asccode && asccode <= '9')
    ret = asccode - '0';
  else if ('a' <= asccode && asccode <= 'f')
    ret = asccode - 'a' + 10;
  else if ('A' <= asccode && asccode <= 'F')
    ret = asccode - 'A' + 10;
  else
    ret = 0;

  return ret;
}

void colour_data_control(const unsigned char value[], u16 length)
{
  u8 string_data[13];
  u16 h, s, v;
  u8 r, g, b;
  u16 hue;
  u8 sat, val;

  u32 c = 0;

  string_data[0] = value[0]; //渐变、直接输出
  string_data[1] = value[1];
  string_data[2] = value[2];
  string_data[3] = value[3];
  string_data[4] = value[4];
  string_data[5] = value[5];
  string_data[6] = value[6];
  string_data[7] = value[7];
  string_data[8] = value[8];
  string_data[9] = value[9];
  string_data[10] = value[10];
  string_data[11] = value[11];
  string_data[12] = value[12];

  h = __str2short(__asc2hex(string_data[1]), __asc2hex(string_data[2]), __asc2hex(string_data[3]), __asc2hex(string_data[4]));
  s = __str2short(__asc2hex(string_data[5]), __asc2hex(string_data[6]), __asc2hex(string_data[7]), __asc2hex(string_data[8]));
  v = __str2short(__asc2hex(string_data[9]), __asc2hex(string_data[10]), __asc2hex(string_data[11]), __asc2hex(string_data[12]));

  hue = h * 182;
  sat = s / 4;
  val = v / 4;
  c = strip.gamma32(strip.ColorHSV(hue, sat, val)); // hue -> RGB
  // DebugSerial.println(hue);
  // DebugSerial.println(sat);
  // DebugSerial.println(val);
  strip.fill(c, 0, RGB_LED.PixelNum);
  strip.show(); // Update strip with new contents
}
//Compute average of a int array, given the starting pointer and the length
int compute_average(int *avgs, int len)
{
  int sum = 0;
  for (int i = 0; i < len; i++)
    sum += avgs[i];

  return (int)(sum / len);
}

//Insert a value into an array, and shift it down removing
//the first value if array already full
void insert(int val, int *avgs, int len)
{
  for (int i = 0; i < len; i++)
  {
    if (avgs[i] == -1)
    {
      avgs[i] = val;
      return;
    }
  }

  for (int i = 1; i < len; i++)
  {
    avgs[i - 1] = avgs[i];
  }
  avgs[len - 1] = val;
}

//Function imported from the arduino website.
//Basically map, but with a curve on the scale (can be non-uniform).
float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve)
{

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10)
    curve = 10;
  if (curve < -10)
    curve = -10;

  curve = (curve * -.1);  // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin)
  {
    inputValue = originalMin;
  }
  if (inputValue > originalMax)
  {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin)
  {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange; // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax)
  {
    return 0;
  }

  if (invFlag == 0)
  {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }
  else // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}
