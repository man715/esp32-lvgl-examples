#ifndef DIAPLAY_H_
#define DISPLAY_H_ 
// GPIO Pins for LCD i80 Display

#define LCD_POWER_ON            15  // power
#define LCD_BL                  38 // back light

// Data Pins
#define LCD_D0                  39
#define LCD_D1                  40
#define LCD_D2                  41
#define LCD_D3                  42
#define LCD_D4                  45
#define LCD_D5                  46
#define LCD_D6                  47
#define LCD_D7                  48

// Control Pins
#define LCD_WR                  8 // write
#define LCD_RD                  9 // read
#define LCD_DC                  7 // data / command
#define LCD_CS                  6 // chip selection
#define LCD_RES                 5 // reset

// Display settings
#define BACK_LIGHT_ON_LEVEL     1
#define BACK_LIGHT_OFF_LEVEL    !BACK_LIGHT_ON_LEVEL // we set it this way so all you need to change is the BACK_LIGHT_ON_LEVEL if on is 0 for your device
#define LCD_H_RES               320
#define LCD_V_RES               170
#define LCD_BUF_SIZE            (LCD_H_RES * LCD_V_RES)
#define LCD_CMD_BITS            8 // how many bits wide a command is
#define LCD_PARAM_BITS          8 // how many bits wide a param is

// Clock Settings
#define LCD_PIXEL_CLOCK_HZ      (6528000) // still not sure where this comes from

// LVGL Settings
#define LVGL_TICK_PERIOD    2 // lvgl tick period should be between 1 and 10

#endif /*END HELLO-WORLD_H_*/