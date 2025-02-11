
/***************************
   LVGL Widgets
   This is a widgets demo for LVGL - Light and Versatile Graphics Library
   import from: https://github.com/lvgl/lvgl.git

   Dependent libraries:
   LVGL: https://github.com/lvgl/lvgl.git

   Touch libraries:
   FT6X36: https://github.com/strange-v/FT6X36.git
   GT911: https://github.com/TAMCTec/gt911-arduino.git
   XPT2046: https://github.com/PaulStoffregen/XPT2046_Touchscreen.git

   LVGL Configuration file:
   Copy your_arduino_path/libraries/lvgl/lv_conf_template.h
   to your_arduino_path/libraries/lv_conf.h
   Then find and set:
   #define LV_COLOR_DEPTH     16
   #define LV_TICK_CUSTOM     1

   For SPI display set color swap can be faster, parallel screen don't set!
   #define LV_COLOR_16_SWAP   1

   Optional: Show CPU usage and FPS count
   #define LV_USE_PERF_MONITOR 1
 **************************/
#include <lvgl.h>
#include <demos/lv_demos.h>
#include "ui.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"
#define BPM_CHARACTERISTIC_UUID "abcdef02-1234-5678-1234-56789abcdef0"
#define SPO2_CHARACTERISTIC_UUID "abcdef03-1234-5678-1234-56789abcdef0"
#define OTOT_CHARACTERISTIC_UUID "abcdef04-1234-5678-1234-56789abcdef0"  // Corrected UUID for OTOT

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLECharacteristic *pBpmCharacteristic = NULL;
BLECharacteristic *pSpo2Characteristic = NULL;
BLECharacteristic *pOtotCharacteristic = NULL;  // Corrected to match defined characteristic

/***************************
 **************************/
#include <Arduino_GFX_Library.h>
#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
  40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
  45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
  5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
  8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
  bus,
  800 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
  480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
#endif
#include "touch.h"

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

int jempol = 1;
int telunjuk = 1;
int tengah = 1;
int kia = 1;
int kelingking = 1;
int pwm = 0;
int mtor = 1;
int vacum = 1;
int pump = 1;
int nilai_bpm = 0;
int nilai_spo2 = 0;
int nilai_otot = 0;
lv_chart_series_t *ui_Chart1_series_1;

void setup()
{
  Serial.begin(115200);
  Serial.println("LVGL Widgets Demo");

  // Init touch device

  // Init Display
  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  gfx->fillScreen(RED);
  delay(500);
  gfx->fillScreen(GREEN);
  delay(500);
  gfx->fillScreen(BLUE);
  delay(500);
  gfx->fillScreen(WHITE);
  delay(500);
  lv_init();
  delay(10);
  touch_init();
  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4 , MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 4);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();
    ui_Chart1_series_1 = lv_chart_add_series(ui_Chart1, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

    Serial.println("Setup done");
  }

  // Initialize BLE
  BLEDevice::init("ESP32_LVGL_Device");
  pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Karakteristik untuk mengontrol relai dan motor
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Karakteristik untuk menerima nilai BPM
  pBpmCharacteristic = pService->createCharacteristic(
                         BPM_CHARACTERISTIC_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );

  // Karakteristik untuk menerima nilai SpO2
  pSpo2Characteristic = pService->createCharacteristic(
                          SPO2_CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_WRITE |
                          BLECharacteristic::PROPERTY_NOTIFY
                        );

  // Karakteristik untuk menerima nilai otot
  pOtotCharacteristic = pService->createCharacteristic(
                          OTOT_CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_WRITE |
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE Initialized");
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  jari();
  motor();
  sensor();
  delay(5);
}

void jari() {
  jempol = lv_obj_has_state(ui_Button5, LV_STATE_CHECKED) ? 0 : 1;
  telunjuk = lv_obj_has_state(ui_Button4, LV_STATE_CHECKED) ? 0 : 1;
  tengah = lv_obj_has_state(ui_Button3, LV_STATE_CHECKED) ? 0 : 1;
  kia = lv_obj_has_state(ui_Button2, LV_STATE_CHECKED) ? 0 : 1;
  kelingking = lv_obj_has_state(ui_Button6, LV_STATE_CHECKED) ? 0 : 1;
  String dataToSend = String(jempol) + "," + String(telunjuk) + "," +
                      String(tengah) + "," + String(kia) + "," + String(kelingking);

  // Send via BLE
  pCharacteristic->setValue(dataToSend.c_str());
  pCharacteristic->notify();
}

void motor() {
  pwm = lv_arc_get_value(ui_Arc1);
  mtor = lv_obj_has_state(ui_Switch1, LV_STATE_CHECKED) ? 0 : 1;
  vacum = lv_obj_has_state(ui_vacum, LV_STATE_CHECKED) ? 0 : 1;
  pump = lv_obj_has_state(ui_pump, LV_STATE_CHECKED) ? 0 : 1;
  String dataToSend = String(pwm) + "," + String(mtor) + "," +
                      String(vacum) + "," + String(pump);

  // Send via BLE
  pCharacteristic->setValue(dataToSend.c_str());
  pCharacteristic->notify();
}

void sensor() {
  // Terima nilai BPM dari BLE
  if (pBpmCharacteristic && pBpmCharacteristic->getValue().length() > 0) {
    nilai_bpm = atoi(pBpmCharacteristic->getValue().c_str());
  }

  // Terima nilai SpO2 dari BLE
  if (pSpo2Characteristic && pSpo2Characteristic->getValue().length() > 0) {
    nilai_spo2 = atoi(pSpo2Characteristic->getValue().c_str());
  }

  // Terima nilai otot dari BLE
  if (pOtotCharacteristic && pOtotCharacteristic->getValue().length() > 0) {
    nilai_otot = atof(pOtotCharacteristic->getValue().c_str());
  }

  // Update tampilan UI dengan nilai BPM
  lv_arc_set_value(ui_Arc5, nilai_bpm);
  char bpm_str[10];
  sprintf(bpm_str, "%d", nilai_bpm);
  lv_label_set_text(ui_Label7, bpm_str);

  // Update tampilan UI dengan nilai SpO2
  lv_arc_set_value(ui_Arc4, nilai_spo2);
  char spo2_str[10];
  sprintf(spo2_str, "%d", nilai_spo2);
  lv_label_set_text(ui_Label6, spo2_str);

  // Update tampilan UI dengan nilai otot
  lv_chart_set_next_value(ui_Chart1, ui_Chart1_series_1, nilai_otot);
  lv_chart_refresh(ui_Chart1);
}
