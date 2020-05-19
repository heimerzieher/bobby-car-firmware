#ifndef CONFIG_H
#define CONFIG_H

#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication


#define LVGL_TICK_PERIOD 20

/*================
 *  THEME USAGE
 *================*/
#define LV_THEME_LIVE_UPDATE    0   /*1: Allow theme switching at run time. Uses 8..10 kB of RAM*/

#define LV_USE_THEME_TEMPL      0   /*Just for test*/
#define LV_USE_THEME_DEFAULT    0   /*Built mainly from the built-in styles. Consumes very few RAM*/
#define LV_USE_THEME_ALIEN      0   /*Dark futuristic theme*/
#define LV_USE_THEME_NIGHT      1   /*Dark elegant theme*/
#define LV_USE_THEME_MONO       0   /*Mono color theme for monochrome displays*/
#define LV_USE_THEME_MATERIAL   0   /*Flat theme with bold colors and light shadows*/
#define LV_USE_THEME_ZEN        0   /*Peaceful, mainly light theme */
#define LV_USE_THEME_NEMO       0   /*Water-like theme based on the movie "Finding Nemo"*/

/*==================
 *    FONT USAGE
 *===================*/

/* The built-in fonts contains the ASCII range and some Symbols with  4 bit-per-pixel.
 * The symbols are available via `LV_SYMBOL_...` defines
 * More info about fonts: https://docs.littlevgl.com/#Fonts
 * To create a new font go to: https://littlevgl.com/ttf-font-to-c-array
 */

/* Robot fonts with bpp = 4
 * https://fonts.google.com/specimen/Roboto  */
#define LV_FONT_ROBOTO_12    1
#define LV_FONT_ROBOTO_16    1
#define LV_FONT_ROBOTO_22    1
#define LV_FONT_ROBOTO_28    1


/* Serial Commands */
#define TIME_SEND           100

#define SERIAL_TIMEOUT_COUNTS 1000


#endif // CONFIG_H
