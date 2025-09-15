#include<Arduino.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <lvgl.h>
#define LGFX_USE_V1
#include <SPI.h>
#include <LovyanGFX.hpp>
#include "FT6236.h"
#include "ui.h"
/*
   If you want to use the LVGL examples,
  make sure to install the lv_examples Arduino library
  and uncomment the following line.
*/
// #include <C:/Users/12397/Documents/Arduino/libraries/lvgl/examples/lv_examples.h>
// #include <C:/Users/12397/Documents/Arduino/libraries/lvgl/demos/lv_demos.h>
const int i2c_touch_addr = TOUCH_I2C_ADD;


#define LCD_BL 46

#define SDA_FT6236 38
#define SCL_FT6236 39
//FT6236 ts = FT6236();

/**************************************************************/
// --- WiFi Credentials ---
const char ssid[]     = "colocation";
const char password[] = "colocation";
/**************************************************************/
const char broker[]   = "collocationstudy.kaatru.org";
const int  port       = 1883;
/**************************************************************/
// --- Topic ---
const char topic[]    = "dev/SIN11";
/**************************************************************/

/**************************************************************/
// --- MQTT & WiFi Clients ---
/**************************************************************/
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
#define JSON_BUFFER_SIZE 512

/* ---------- reconnect timing parameters ---------- */
const unsigned long WIFI_RETRY_MS = 2000;   // 2 s back‑off
const unsigned long MQTT_RETRY_MS = 5000;   // 5 s back‑off

unsigned long lastWiFiAttempt  = 0;
unsigned long lastMQTTAttempt  = 0;

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488 _panel_instance;
    lgfx::Bus_Parallel16 _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        cfg.port = 0;
        cfg.freq_write = 80000000;
        cfg.pin_wr = 18;
        cfg.pin_rd = 48;
        cfg.pin_rs = 45;

        cfg.pin_d0 = 47;
        cfg.pin_d1 = 21;
        cfg.pin_d2 = 14;
        cfg.pin_d3 = 13;
        cfg.pin_d4 = 12;
        cfg.pin_d5 = 11;
        cfg.pin_d6 = 10;
        cfg.pin_d7 = 9;
        cfg.pin_d8 = 3;
        cfg.pin_d9 = 8;
        cfg.pin_d10 = 16;
        cfg.pin_d11 = 15;
        cfg.pin_d12 = 7;
        cfg.pin_d13 = 6;
        cfg.pin_d14 = 5;
        cfg.pin_d15 = 4;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }

      {
        auto cfg = _panel_instance.config();

        cfg.pin_cs = -1;
        cfg.pin_rst = -1;
        cfg.pin_busy = -1;
        cfg.memory_width = 320;
        cfg.memory_height = 480;
        cfg.panel_width = 320;
        cfg.panel_height = 480;
        cfg.offset_x = 0;
        cfg.offset_y = 0;
        cfg.offset_rotation = 2;
        cfg.dummy_read_pixel = 8;
        cfg.dummy_read_bits = 1;
        cfg.readable = true;
        cfg.invert = false;
        cfg.rgb_order = false;
        cfg.dlen_16bit = true;
        cfg.bus_shared = true;
        _panel_instance.config(cfg);
      }
      setPanel(&_panel_instance);
    }
};

LGFX tft;
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 5 ];


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();

  lv_disp_flush_ready( disp );
}

void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  int pos[2] = {0, 0};

  ft6236_pos(pos);
  if (pos[0] > 0 && pos[1] > 0)
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = tft.width()-pos[1];
    data->point.y = pos[0];
    Serial.printf("x-%d,y-%d\n", data->point.x, data->point.y);
  }
  else {
    data->state = LV_INDEV_STATE_REL;
  }
}

/**************************************************************/
// --- MQTT Message Handler ---
/**************************************************************/
void onMqttMessage(int messageSize) {
  //Serial.println("Hello came inside the function");
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError err = deserializeJson(doc, mqttClient);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.f_str());
    return;
  }

  if (doc.containsKey("sPM1")) {
    float val = doc["sPM1"].as<float>();
    //Serial.println(val);
    lv_arc_set_value(ui_pm1Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelPM, buf);
    lv_label_set_text(ui_pm1Val, buf);
    
  }

  if (doc.containsKey("sPM2")) {
    float val = doc["sPM2"].as<float>();
    //Serial.println(val);
    lv_arc_set_value(ui_pm25Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelPM, buf);
    lv_label_set_text(ui_pm25Val, buf);
    //lv_chart_set_next_value(ui_Chart2, ui_Chart2_series_1, (int)val);
  }

  if (doc.containsKey("sPM4")) {
    float val = doc["sPM4"].as<float>();
    //Serial.println(val);
    lv_arc_set_value(ui_pm4Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelPM, buf);
    lv_label_set_text(ui_pm4Val, buf);
    //lv_chart_set_next_value(ui_Chart2, ui_Chart2_series_2, (int)val);
  }

   if (doc.containsKey("sPM10")) {
    float val = doc["sPM10"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcPM, (int)val);
    lv_arc_set_value(ui_pm10Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    lv_label_set_text(ui_pm10Val, buf);
    //lv_label_set_text(ui_LabelPM, buf);
    //lv_chart_set_next_value(ui_Chart2, ui_Chart2_series_3, (int)val);
  }

  if (doc.containsKey("sTemp")) {
    float val = doc["sTemp"].as<float>();
    //Serial.println(val);
    lv_arc_set_value(ui_tempArc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelTemp, buf);
    lv_label_set_text(ui_tempVal, buf);
    //lv_label_set_text(ui_tempVal2, buf);
  }

  if (doc.containsKey("k30Co2")) {
    float val = doc["k30Co2"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_Co2Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_Co2Val, buf);
  }

   if (doc.containsKey("sVocI")) {
    float val = doc["sVocI"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_Arc1, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_VOC_Value, buf);
  }

  if (doc.containsKey("co_ppb")) {
    float val = doc["co_ppb"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_CoArc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_CoVal, buf);
  }

  if (doc.containsKey("so2_ppb")) {
    float val = doc["so2_ppb"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_So2Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_So2Val, buf);
  }

  if (doc.containsKey("no2_ppb")) {
    float val = doc["no2_ppb"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_No2Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_No2Val, buf);
  }

  if (doc.containsKey("o3_ppb")) {
    float val = doc["o3_ppb"].as<float>();
    //Serial.println(val);
    //lv_arc_set_value(ui_ArcRH, (int)val);
    lv_arc_set_value(ui_O3Arc, (int)val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_O3Val, buf);
  }

   if (doc.containsKey("rh")) {
    float val = doc["rh"].as<float>();
    lv_arc_set_value(ui_humArc, (int)val);
    //Serial.println(val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    //lv_label_set_text(ui_LabelRH, buf);
    lv_label_set_text(ui_HumVal, buf);
    //lv_label_set_text(ui_humidityVal2, buf);
  }

  //Serial.println("Just before the chart function");

  //lv_chart_refresh(ui_Chart2);
  //Serial.println("came to end of the message fucntion");

  // if (doc.containsKey("dTS")) {
  //   unsigned long epochSeconds = doc["dTS"].as<unsigned long>();
  //   struct tm* timeinfo = localtime((time_t*)&epochSeconds);
  //   if (timeinfo) {
  //     char timeBuf[16];
  //     strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", timeinfo);
  //     lv_label_set_text(ui_timeVal, timeBuf);
  //     lv_label_set_text(ui_timeVal2, timeBuf);
  //   }
  // }

  if (doc.containsKey("dTS")) {
  unsigned long epochSeconds = doc["dTS"].as<unsigned long>();
  struct tm* timeinfo = localtime((time_t*)&epochSeconds);   // already gives you local TZ if you've called configTime()

  if (timeinfo) {
    /* ---------- TIME ---------- */
    char timeBuf[16];                    // "HH:MM:SS\0"
    strftime(timeBuf,  sizeof(timeBuf),  "%H:%M:%S", timeinfo);
    lv_label_set_text(ui_timeVal,  timeBuf);
    lv_label_set_text(ui_timeVal2, timeBuf);

    /* ---------- DATE ---------- */
    char dateBuf[16];                    // "14‑07‑2025\0"
    strftime(dateBuf,  sizeof(dateBuf),  "%d-%m-%Y", timeinfo);
    //lv_label_set_text(ui_dateVal, dateBuf);   // <‑‑ make ui_dateVal in your screen
    lv_label_set_text(ui_dateVal2,  dateBuf);
    lv_label_set_text(ui_dateval, dateBuf);
  }
 }

  }

/* ---------- helper functions ---------- */
void connectWiFi()
{
  Serial.print(F("\n[WiFi] connecting"));
  WiFi.disconnect();                 // drop any half‑open sessions
  WiFi.begin(ssid, password);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    lv_timer_handler();              // keep the GUI alive
    Serial.print('.');
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F(" ✔  IP="));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F(" ✖  timeout"));
  }
}

void connectMQTT()
{
  Serial.print(F("[MQTT] connecting"));
  /* try a few times before giving up so we can return to loop() quickly */
  for (uint8_t i = 0; i < 3 && !mqttClient.connected(); ++i) {
    if (mqttClient.connect(broker, port)) {
      mqttClient.subscribe(topic);
      Serial.println(F(" ✔"));
      return;
    }
    Serial.print(F(" ✖ "));
    Serial.println(mqttClient.connectError());
    delay(300);
    lv_timer_handler();
  }
  Serial.println();  // move to new line after retries
}

void touch_init()
{
  // I2C init
  Wire.begin(SDA_FT6236, SCL_FT6236);
  byte error, address;

  Wire.beginTransmission(i2c_touch_addr);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.print("I2C device found at address 0x");
    Serial.print(i2c_touch_addr, HEX);
    Serial.println("  !");
  }
  else if (error == 4)
  {
    Serial.print("Unknown error at address 0x");
    Serial.println(i2c_touch_addr, HEX);
  }
}

void setup()
{
  Serial.begin( 115200 ); /* prepare for possible serial debug */

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  }

  /* any other one‑time setup ... */
  mqttClient.onMessage(onMqttMessage);
  
  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */
  tft.fillScreen(TFT_BLACK);
  delay(500);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  touch_init();

  //  if (!ts.begin(0, SDA_FT6236, SCL_FT6236)) {
  //    Serial.println("Unable to start the capacitive touch Screen.");
  //  }
  touch_init();

  lv_init();
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 5 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );

#if 0
  /* Create simple label */
  lv_example_get_started_4();

#else
  /* Try an example from the lv_examples Arduino library
     make sure to include it as written above.

  */
  // uncomment one of these demos
  // lv_demo_widgets();            // OK
  ui_init();
#endif
  Serial.println( "Setup done" );
}

void loop()
{
      /* 1. Ensure links are alive (non‑blocking) -------------------------- */
  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastWiFiAttempt > WIFI_RETRY_MS) {
    lastWiFiAttempt = millis();
    connectWiFi();
  }

  if (WiFi.status() == WL_CONNECTED &&
      !mqttClient.connected() &&
      millis() - lastMQTTAttempt > MQTT_RETRY_MS) {
    lastMQTTAttempt = millis();
    connectMQTT();
  }

  /* 2. Normal “work” -------------------------------------------------- */
  if (mqttClient.connected()) {
    mqttClient.poll();               // keep‑alive + message handling
  
  lv_timer_handler(); /* let the GUI do its work */
  delay( 5 );
  }
}


// static void btn_event_cb(lv_event_t * e)
// {
//   lv_event_code_t code = lv_event_get_code(e);
//   lv_obj_t * btn = lv_event_get_target(e);
//   if (code == LV_EVENT_CLICKED) {
//     static uint8_t cnt = 0;
//     cnt++;

//     /*Get the first child of the button which is the label and change its text*/
//     lv_obj_t * label = lv_obj_get_child(btn, 0);
//     lv_label_set_text_fmt(label, "Button: %d", cnt);
//   }
// }

// /**
//    Create a button with a label and react on click event.
// */
// void lv_example_get_started_4(void)
// {
//   lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
//   lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
//   lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
//   lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

//   lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
//   lv_label_set_text(label, "Button");                     /*Set the labels text*/
//   lv_obj_center(label);
// }
