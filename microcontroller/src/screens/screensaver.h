#ifndef SCREENSAVER_H
#define SCREENSAVER_H

#include <lvgl.h>
#include <TFT_eSPI.h>
#include "screen.h"
#include "screenmanager.h"
#include "screens/mainscreen.h"

class ScreenSaver : public Screen
{
public:
    ScreenSaver();

    void create(void) override;
    void update(void) override;
    void customDraw(void) override;

    lv_obj_t* testlabel;
};

#endif // SCREENSAVER_H
