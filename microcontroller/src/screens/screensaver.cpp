#include "screensaver.h"

extern TFT_eSPI tft;
extern ScreenManager screenManager;

ScreenSaver::ScreenSaver()
{

}

void ScreenSaver::create(void)
{
    this->lv_screen = lv_cont_create(NULL, NULL);

    this->testlabel = lv_label_create(this->lv_screen, NULL);

    lv_label_set_text(this->testlabel, "This is a Screen Saver");
}

void ScreenSaver::update()
{

}

void ScreenSaver::customDraw()
{
    tft.fillScreen(TFT_BLACK);

    // Draw some random circles
      for (int i = 0; i < 40; i++)
      {
        int rx = random(60);
        int ry = random(60);
        int x = rx + random(480 - rx - rx);
        int y = ry + random(320 - ry - ry);
        tft.fillEllipse(x, y, rx, ry, random(0xFFFF));
      }

      delay(2000);
      tft.fillScreen(TFT_BLACK);

      for (int i = 0; i < 40; i++)
      {
        int rx = random(60);
        int ry = random(60);
        int x = rx + random(480 - rx - rx);
        int y = ry + random(320 - ry - ry);
        tft.drawEllipse(x, y, rx, ry, random(0xFFFF));
      }

      delay(2000);
}
