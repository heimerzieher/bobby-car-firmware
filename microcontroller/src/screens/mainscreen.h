#ifndef MAINSCREEN_H
#define MAINSCREEN_H

#include "screen.h"

class MainScreen : public Screen
{
public:
    MainScreen();
    void create(void) override;
    void update(void) override;

    static void light_button_event_handler(lv_obj_t* obj, lv_event_t event)
    {
        if(event == LV_EVENT_VALUE_CHANGED)
        {
                //printf("Toggled\n");
        }
    }

    static void settings_button_event_handler(lv_obj_t* obj, lv_event_t event)
    {
        if(event == LV_EVENT_VALUE_CHANGED)
        {
                //printf("Toggled\n");
        }
    }
private:
    lv_obj_t* speed_gauge;
    lv_style_t speed_gauge_style;
    lv_style_t speed_indicator_style;
    lv_obj_t* speed_indicator_obj;
    lv_obj_t* speed_indicator_label;
    lv_obj_t* odo_indicator_label;
    lv_style_t current_arc_style;
    lv_obj_t* current_arc;
    lv_obj_t* current_label;
    lv_obj_t* batvoltage_label;

    lv_style_t charging_meter_style;
    lv_obj_t* charging_meter;

    lv_style_t discharging_meter_style;
    lv_obj_t* discharging_meter;

    lv_obj_t* board_status_container;
    lv_obj_t* front_board_status_head;
    lv_obj_t* front_board_status;
    lv_obj_t* back_board_status_head;
    lv_obj_t* back_board_status;
    lv_style_t board_status_head_style;
    lv_style_t board_status_style;

    lv_obj_t* light_button;
    lv_obj_t* light_button_label;

    lv_obj_t* settings_button;
    lv_obj_t* settings_button_label;
    lv_style_t settings_button_label_style;
};

#endif // MAINSCREEN_H
