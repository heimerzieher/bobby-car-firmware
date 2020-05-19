#include <lvgl.h>
#include <Ticker.h>
#include <TFT_eSPI.h>
#include <HardwareSerial.h>

extern "C"
{
    #include "SerialCommunication.h"
}

#include "motorcontroller.h"
#include "state.h"
#include "config.h"
#include "screenmanager.h"
#include "screens/mainscreen.h"
#include "screens/screensaver.h"

Ticker tick; /* timer for interrupt handler */
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

#if USE_LV_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc)
{

  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  delay(100);
}
#endif

/*** Motor controllers ***/
MotorController frontController(Serial1);
MotorController rearController(Serial2);

/*** Global State***/

State state;

ScreenManager screenManager;

/* Update UI task */
void update_ui(lv_task_t * task)
{
    screenManager.getActiveScreen()->update();
    screenManager.getActiveScreen()->customDraw();
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint16_t c;

    tft.startWrite(); /* Start new TFT transaction */
    tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
    for (int y = area->y1; y <= area->y2; y++) {
      for (int x = area->x1; x <= area->x2; x++) {
        c = color_p->full;
        tft.writeColor(c, 1);
        color_p++;
      }
    }
    tft.endWrite(); /* terminate TFT transaction */

    lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/* Interrupt driven periodic handler */
static void lv_tick_handler(void)
{
  lv_tick_inc(LVGL_TICK_PERIOD);
}

/* Reading input device (simulated encoder here) */
bool read_encoder(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
  static int32_t last_diff = 0;
  int32_t diff = 0; /* Dummy - no movement */
  int btn_state = LV_INDEV_STATE_REL; /* Dummy - no press */

  data->enc_diff = diff - last_diff;;
  data->state = btn_state;

  last_diff = diff;

  return false;
}

void setup() {

  //Serial.begin(9600); /* prepare for possible serial debug */

  //Serial1 auf Pin 12 und 13
  //Serial1.begin(38400,SERIAL_8N1,12,13);
  //Serial2 auf Pin 22 und 23
  //Serial2.begin(38400,SERIAL_8N1,34,35);

  frontController.serial.begin(38400,SERIAL_8N1, 12, 13);
  rearController.serial.begin(38400,SERIAL_8N1, 34, 35);

  lv_init();

#if USE_LV_LOG != 0
  lv_log_register_print(my_print); /* register print function for debugging */
#endif

  tft.begin(); /* TFT init */
  tft.setRotation(3); /* Landscape orientation */

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 480;
  disp_drv.ver_res = 320;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);


  /*Initialize the touch pad*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_ENCODER;
  indev_drv.read_cb = read_encoder;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  //Set the theme..
  lv_theme_t * th = lv_theme_night_init(210, NULL);     //Set a HUE value and a Font for the Night Theme
  lv_theme_set_current(th);

  //lv_obj_t * scr = lv_cont_create(NULL, NULL);
  //lv_disp_load_scr(scr);

  screenManager.LoadScreen<MainScreen>();


  /* UI update task */

  lv_task_t * update_ui_task = lv_task_create(update_ui, 200, LV_TASK_PRIO_MID, nullptr);

  lv_task_ready(update_ui_task);

}

unsigned long iTimeSend = 0;

void loop()
{
  unsigned long timeNow = millis();
  frontController.receiveFeedback();
  rearController.receiveFeedback();

  //command.leftMotorInput = feedback.leftMotorInput;
  //command.rightMotorInput = feedback.rightMotorInput;

  //command.leftMotorSettings = feedback.leftMotorSettings;
  //command.rightMotorSettings = feedback.rightMotorSettings;

  //command.overrideMotorInput = (uint16_t)true;
  //command.leftMotorInput.pwm = 200;
  //command.rightMotorInput.pwm = -200;

  // Send commands
  if (iTimeSend > timeNow) return;

  iTimeSend = timeNow + TIME_SEND;

  frontController.command.leftMotorInput = state.frontControllerState.leftMotorInput;
  frontController.command.rightMotorInput = state.frontControllerState.rightMotorInput;

  frontController.command.leftMotorSettings = state.frontControllerState.leftMotorSettings;
  frontController.command.rightMotorSettings = state.frontControllerState.rightMotorSettings;

  // TODO: both motors
  frontController.command.overrideMotorInput = state.frontControllerState.overrideLeftMotorInput;

  rearController.command.leftMotorInput = state.rearControllerState.leftMotorInput;
  rearController.command.rightMotorInput = state.rearControllerState.rightMotorInput;

  rearController.command.leftMotorSettings = state.rearControllerState.leftMotorSettings;
  rearController.command.rightMotorSettings = state.rearControllerState.rightMotorSettings;

  // TODO: both motors
  rearController.command.overrideMotorInput = state.rearControllerState.overrideLeftMotorInput;

  rearController.command.leftMotorSettings.iMotorMax = 20;
  rearController.command.rightMotorSettings.iMotorMax = 20;
  rearController.command.leftMotorSettings.iDcCurrentMax = 22;
  rearController.command.rightMotorSettings.iDcCurrentMax = 22;

  frontController.transmitCommand();
  rearController.transmitCommand();

  lv_task_handler(); /* let the GUI do its work */
  //delay(0);
}
