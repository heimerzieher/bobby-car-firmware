#include "mainscreen.h"

#include <Arduino.h>
#include <lvgl.h>

#include "motorcontroller.h"

extern MotorController frontController;
extern MotorController rearController;

//TODO from config
static int maxCurrent = 15;

MainScreen::MainScreen()
{

}

void MainScreen::create(void)
{
    this->lv_screen = lv_cont_create(NULL, NULL);

    /* Speed indicator */

    /* Gauge */

    /*Create a style*/
    lv_style_copy(&this->speed_gauge_style, &lv_style_pretty);
    this->speed_gauge_style.body.main_color = lv_color_hex3(0xAAA);     /*Line color at the beginning*/
    this->speed_gauge_style.body.grad_color =  lv_color_hex3(0xAAA);    /*Line color at the end*/
    this->speed_gauge_style.body.padding.left = 10;                      /*Scale line length*/
    this->speed_gauge_style.body.padding.inner = 8 ;                    /*Scale label padding*/
    this->speed_gauge_style.body.border.color = lv_color_hex3(0x333);   /*Needle middle circle color*/
    this->speed_gauge_style.line.width = 5;
    this->speed_gauge_style.text.color = lv_color_hex3(0xFFF);
    this->speed_gauge_style.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/

    /*Describe the color for the needles*/
    static lv_color_t needle_colors[1];
    needle_colors[0] = LV_COLOR_RED;

    /*Create a gauge*/
    this->speed_gauge = lv_gauge_create(this->lv_screen, NULL);
    lv_gauge_set_style(speed_gauge, LV_GAUGE_STYLE_MAIN, &this->speed_gauge_style);
    lv_gauge_set_needle_count(this->speed_gauge, 1, needle_colors);
    lv_obj_set_size(this->speed_gauge, 200, 200);
    lv_obj_align(this->speed_gauge, NULL, LV_ALIGN_CENTER, 0, 20);

    lv_gauge_set_range(this->speed_gauge, 0, 25);
    lv_gauge_set_critical_value(this->speed_gauge, 20);

    /*Set the values*/
    //lv_gauge_set_value(this->speed_gauge, 0, 0);

    /* Speed indicator label*/
    lv_style_copy(&this->speed_indicator_style, &lv_style_pretty);         //Copy a built-in style as a starting point

    this->speed_indicator_style.text.font = &lv_font_roboto_28;

    speed_indicator_obj = lv_obj_create(this->lv_screen, NULL);
    lv_obj_align(speed_indicator_obj, NULL, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style(speed_indicator_obj, &this->speed_indicator_style);

    this->speed_indicator_label = lv_label_create(this->speed_indicator_obj, NULL);
    lv_label_set_text(this->speed_indicator_label, "0 km/h");
    lv_obj_align(this->speed_indicator_label, NULL, LV_ALIGN_CENTER, 0, 0);

    /* ODO Indicator */
    this->odo_indicator_label = lv_label_create(this->lv_screen, NULL);
    lv_label_set_text(this->odo_indicator_label, "0 km");
    lv_obj_align(this->odo_indicator_label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

    /* Charge/Discharge indicator */
    /*Create style for the Arcs*/
    lv_style_copy(&this->current_arc_style, &lv_style_plain);
    this->current_arc_style.line.color = lv_color_hex(0x219D448);           /*Arc color*/
    this->current_arc_style.line.width = 8;                       /*Arc width*/

    /*Create an Arc*/
    //this->current_arc = lv_arc_create(this->lv_screen, NULL);
    //lv_arc_set_style(this->current_arc, LV_ARC_STYLE_MAIN, &this->current_arc_style);          /*Use the new style*/
    /*lv_arc_set_angles(this->current_arc, 90, 180);
    lv_obj_set_size(this->current_arc, 250, 250);
    lv_obj_align(this->current_arc, NULL, LV_ALIGN_CENTER, 0, 20);*/

    /* current draw Indicator */
    this->current_label = lv_label_create(this->lv_screen, NULL);
    lv_label_set_text(this->current_label, "0 A");
    lv_obj_align(this->current_label, NULL, LV_ALIGN_CENTER, 0, -120);

    /* battery voltage Indicator */
    this->batvoltage_label = lv_label_create(this->lv_screen, NULL);
    lv_label_set_text(this->batvoltage_label, "0 V");
    lv_obj_align(this->batvoltage_label, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

    /* charging line meter */
    /*Create a style for the line meter*/
    lv_style_copy(&this->charging_meter_style, &lv_style_pretty_color);
    this->charging_meter_style.line.width = 5;
    this->charging_meter_style.line.color = LV_COLOR_SILVER;
    this->charging_meter_style.body.main_color = lv_color_hex(0x219D448);         /*Light blue*/
    this->charging_meter_style.body.grad_color = lv_color_hex(0x219D448);         /*Dark blue*/
    this->charging_meter_style.body.padding.left = 16;                           /*Line length*/

    /*Create a line meter */
    this->charging_meter = lv_lmeter_create(this->lv_screen, NULL);
    lv_lmeter_set_range(this->charging_meter, 0, maxCurrent);                   /*Set the range*/
    lv_lmeter_set_value(this->charging_meter, 0);                       /*Set the current value*/
    lv_lmeter_set_scale(this->charging_meter, 90, 11);                  /*Set the angle and number of lines*/
    lv_lmeter_set_angle_offset(this->charging_meter, 45);
    lv_lmeter_set_style(this->charging_meter, LV_LMETER_STYLE_MAIN, &this->charging_meter_style);           /*Apply the new style*/
    lv_obj_set_size(this->charging_meter, 250, 250);
    lv_obj_align(this->charging_meter, NULL, LV_ALIGN_CENTER, 0, 20);

    /* discharging line meter */
    /*Create a style for the line meter*/
    lv_style_copy(&this->discharging_meter_style, &lv_style_pretty_color);
    this->discharging_meter_style.line.width =5;
    this->discharging_meter_style.line.color = LV_COLOR_RED;            /* Red (the scale is inverted)*/
    this->discharging_meter_style.body.main_color = LV_COLOR_SILVER;
    this->discharging_meter_style.body.grad_color = LV_COLOR_SILVER;
    this->discharging_meter_style.body.padding.left = 16;                           /*Line length*/

    /*Create a line meter */
    this->discharging_meter = lv_lmeter_create(this->lv_screen, NULL);
    lv_lmeter_set_range(this->discharging_meter, 0, maxCurrent);                   /*Set the range*/
    lv_lmeter_set_value(this->discharging_meter, maxCurrent);                       /*Set the current value*/
    lv_lmeter_set_scale(this->discharging_meter, 90, 11);                  /*Set the angle and number of lines*/
    lv_lmeter_set_angle_offset(this->discharging_meter, 315);
    lv_lmeter_set_style(this->discharging_meter, LV_LMETER_STYLE_MAIN, &this->discharging_meter_style);           /*Apply the new style*/
    lv_obj_set_size(this->discharging_meter, 250, 250);
    lv_obj_align(this->discharging_meter, NULL, LV_ALIGN_CENTER, 0, 20);

    /* Board Status */
    this->board_status_container = lv_cont_create(this->lv_screen, NULL);
    lv_obj_set_auto_realign(this->board_status_container, true);
    lv_obj_align_origo(this->board_status_container, NULL, LV_ALIGN_OUT_LEFT_MID, 30, 0); //TODO FIX  /*This parametrs will be sued when realigned*/
    lv_cont_set_fit(this->board_status_container, LV_FIT_TIGHT);
    lv_cont_set_layout(this->board_status_container, LV_LAYOUT_COL_L);
    lv_cont_set_style(this->board_status_container, LV_CONT_STYLE_MAIN, &lv_style_pretty_color);

    lv_style_copy(&this->board_status_head_style, &lv_style_pretty_color);
    this->board_status_head_style.text.font = &lv_font_roboto_22;

    lv_style_copy(&this->board_status_style, &lv_style_pretty_color);
    this->board_status_style.text.font = &lv_font_roboto_16;
    this->board_status_style.text.color = LV_COLOR_RED;

    this->front_board_status_head = lv_label_create(this->board_status_container, NULL);
    lv_label_set_text(this->front_board_status_head, "Front");
    lv_label_set_style(this->front_board_status_head, LV_LABEL_STYLE_MAIN, &this->board_status_head_style);

    this->front_board_status = lv_label_create(this->board_status_container, NULL);
    lv_label_set_text(this->front_board_status, "Status...");
    lv_label_set_style(this->front_board_status, LV_LABEL_STYLE_MAIN, &this->board_status_style);

    this->back_board_status_head = lv_label_create(this->board_status_container, NULL);
    lv_label_set_text(this->back_board_status_head, "Rear");
    lv_label_set_style(this->back_board_status_head, LV_LABEL_STYLE_MAIN, &this->board_status_head_style);

    this->back_board_status = lv_label_create(this->board_status_container, NULL);
    lv_label_set_text(this->back_board_status, "Status...");
    lv_label_set_style(this->back_board_status, LV_LABEL_STYLE_MAIN, &this->board_status_style);

    /* Light Button */
    this->light_button = lv_btn_create(this->lv_screen, NULL);
    lv_obj_set_event_cb(this->light_button, this->light_button_event_handler);
    lv_obj_align(this->light_button, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    //lv_btn_set_toggle(this->light_button, true);
    //lv_btn_toggle(this->light_button);
    lv_btn_set_fit2(this->light_button, LV_FIT_NONE, LV_FIT_TIGHT);

    this->light_button_label = lv_label_create(this->light_button, NULL);
    lv_label_set_text(this->light_button_label, "Light");

    /* Settings Button */
    this->settings_button = lv_btn_create(this->lv_screen, NULL);
    lv_obj_set_event_cb(this->settings_button, this->settings_button_event_handler);
    lv_obj_align(this->settings_button, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 20);
    lv_btn_set_fit2(this->settings_button, LV_FIT_NONE, LV_FIT_TIGHT);
    this->settings_button_label = lv_label_create(this->settings_button, NULL);
    lv_label_set_text(this->settings_button_label, " " LV_SYMBOL_SETTINGS " ");

    lv_style_copy(&this->settings_button_label_style, &lv_style_pretty_color);
    this->settings_button_label_style.text.font = &lv_font_roboto_22;

    lv_label_set_style(this->settings_button_label, LV_LABEL_STYLE_MAIN, &this->settings_button_label_style);
}

void MainScreen::update(void)
{
    // status of front and back board
    if(frontController.isOnline)
        lv_label_set_text(this->front_board_status, "On");
    else
        lv_label_set_text(this->front_board_status, "Off");

    if(rearController.isOnline)
        lv_label_set_text(this->back_board_status, "On");
    else
        lv_label_set_text(this->back_board_status, "Off");

    // speed in kmh
    // wheel circumference: 0.54 cm
    uint16_t kmh = round(((((float)abs(frontController.feedback.leftMotorStatus.nRpm) + (float)abs(frontController.feedback.rightMotorStatus.nRpm)
                            + (float)abs(rearController.feedback.leftMotorStatus.nRpm) + (float)abs(rearController.feedback.rightMotorStatus.nRpm))/4.0f)
                          *60.0f*0.54f) / 1000.0f);

    // current im ampere
    float current = (float)frontController.feedback.leftMotorStatus.dcCurrent/50.0f +
                    (float)frontController.feedback.rightMotorStatus.dcCurrent/50.0f +
                    (float)rearController.feedback.leftMotorStatus.dcCurrent/50.0f +
                    (float)rearController.feedback.rightMotorStatus.dcCurrent/50.0f;

    // voltage
    float batVolt = ((float)frontController.feedback.batVoltage/100.0f)+((float)rearController.feedback.batVoltage/100.0f) / 2.0f;


    if(current >= 0) // Charging
    {
        lv_lmeter_set_value(this->charging_meter, (int)abs(round(current)));
    }

    else // discharging
    {
        // substract from maxCurrent because of invcerted scale
        lv_lmeter_set_value(this->discharging_meter, maxCurrent-(int)abs(round(current)));
    }

    lv_label_set_text_fmt(this->batvoltage_label, LV_SYMBOL_BATTERY_FULL " %.2f V", batVolt);
    lv_obj_align(this->batvoltage_label, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);


    lv_label_set_text_fmt(this->current_label, "%.2f A", current);


    lv_label_set_text_fmt(this->speed_indicator_label, "%d km/h", kmh);
    lv_gauge_set_value(this->speed_gauge, 0, kmh);

}
