/**
 ********************************************************************************
 * @file    hello-world.c
 * @author  man715
 * @date    2022-12-03
 * @brief   This is a simple demo of how to use ESP32 S3 with LVGL 
 ********************************************************************************
 */

/************************************
 * INCLUDE
 ************************************/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h" // GPIO functions
#include "esp_lcd_panel_vendor.h" // the st7789 functions
#include "esp_lcd_panel_ops.h" // panel operations
#include "esp_lcd_panel_io.h" // panel input and output functions
#include "esp_err.h" // error handeling 

#include "lvgl.h"

#include "display.h"

/************************************
 * PROTOTYPES
 ************************************/
void configure_pins();
lv_disp_t * create_panel();
lv_disp_t * setup_lvgl(lv_disp_drv_t *, esp_lcd_panel_handle_t );
static bool lvgl_flush_ready_cb(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t *, void *);
static void lvgl_increase_tick(void *);
void lvgl_flush_cb(lv_disp_drv_t *, const lv_area_t *, lv_color_t *);
static void createLvglTimerTask(void);

/************************************
 * GLOBAL PROTOTOYPES
 ************************************/
void lv_example_get_started_1(lv_disp_t *);

/************************************
 * STATIC VARIABLES
 ************************************/
static lv_disp_drv_t disp_drv; // can stay

/************************************
 * MAIN APPLICATION
 ************************************/
void app_main(void)
{
    configure_pins();

    // Turn off backlight
    gpio_set_level(LCD_BL, BACK_LIGHT_OFF_LEVEL);
    // Set RD to high
    gpio_set_level(LCD_RD, 1);

    lv_disp_t * disp = create_panel();
    

    /************************************
     * Turn on Back Light
     ************************************/
    gpio_set_level(LCD_POWER_ON, true);
    gpio_set_level(LCD_BL, BACK_LIGHT_ON_LEVEL);
    
    lv_example_get_started_1(disp);
    createLvglTimerTask();
}

// Configure the Pins
void configure_pins()
{
    /************************************
     * Configure GPIO Pins
     ************************************/   

    gpio_config_t outputPins = {
        GPIO_SEL_9 | GPIO_SEL_15 | GPIO_SEL_38,
        GPIO_MODE_OUTPUT,
        GPIO_PULLUP_DISABLE,
        GPIO_PULLDOWN_DISABLE,
        GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&outputPins));
}

// Create the display panel
lv_disp_t * create_panel()
{
    esp_lcd_panel_handle_t panel_handle = NULL; // does not need to be static
    /************************************
     * Configure i80 bus
     ************************************/
    
    // Declare the bus
    esp_lcd_i80_bus_handle_t i80_bus = NULL;

    // Create the i80 bus config
    esp_lcd_i80_bus_config_t i80_bus_config = {
        .dc_gpio_num = LCD_DC,
        .wr_gpio_num = LCD_WR,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            LCD_D0,
            LCD_D1,
            LCD_D2,
            LCD_D3,
            LCD_D4,
            LCD_D5,
            LCD_D6,
            LCD_D7
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_BUF_SIZE * sizeof(uint16_t)
    };
    // Register the bus
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&i80_bus_config, &i80_bus));

    /************************************
     * Create the Input/Output Handle
     ************************************/
    
    
    // Create the IO handle
    static esp_lcd_panel_io_handle_t panel_io_handle = NULL; 
    
        // Create the panel IO handle config
    esp_lcd_panel_io_i80_config_t panel_io_handle_config = {
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 20,
        .on_color_trans_done = lvgl_flush_ready_cb,
        .user_ctx = &disp_drv, // gets pased to the on_color_trans_done user_ctx
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1
        }
    };

    /************************************
     * Create the panel handle
     ************************************/
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &panel_io_handle_config, &panel_io_handle));

    // Create the Panel Handle
    

    // create the panel handle config
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RES,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16
    };

    // create the panel 
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_handle, &panel_config, &panel_handle));

    /************************************
     * Panel Handle Configuration
     ************************************/
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_swap_xy(panel_handle,true);
    esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_set_gap(panel_handle, 0, 35);

    lv_disp_t *disp = setup_lvgl(&disp_drv, panel_handle);
    return disp;
}

lv_disp_t * setup_lvgl(lv_disp_drv_t *disp_drv, esp_lcd_panel_handle_t panel_handle)
{
     /************************************
     * LVGL Setup
     ************************************/

    // Setup the timer to tell LVGL how many milliseconds have passed
    esp_timer_create_args_t lvgl_timer_args = {
        .callback = &lvgl_increase_tick,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_handle = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_timer_args, &lvgl_tick_handle));

    // Run the timer every LVGL_TICK_PERIOD * 1000 ()
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_handle, LVGL_TICK_PERIOD * 1000));

    // create the draw/display buffer2
    static lv_disp_draw_buf_t draw_buf;

    // Initialize LVGL
    lv_init();

    // static lv_color_t buf1[LCD_BUF_SIZE / 10];
    lv_color_t *buf1 = heap_caps_malloc(LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_BUF_SIZE);
    assert(buf1);

    // register a function which can cpy the rendered image to an area of the display
    lv_disp_drv_init(disp_drv);
    disp_drv->flush_cb = lvgl_flush_cb;
    disp_drv->draw_buf = &draw_buf;
    disp_drv->hor_res = LCD_H_RES;
    disp_drv->ver_res = LCD_V_RES;
    disp_drv->user_data = panel_handle;
    lv_disp_t * disp = lv_disp_drv_register(disp_drv);
    return disp;
}

// Flushes the buffer?
static bool lvgl_flush_ready_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    // Get the display driver that is passed into the user_ctx
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);    
    return false;
}

static void lvgl_increase_tick(void *args)
{
    lv_tick_inc(LVGL_TICK_PERIOD);
}

// Writes to the display
void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    // Get the panel handle from the dispaly that was passed
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) disp->user_data;

    //copy the buffer's content to the display
    int offsetX1 = area->x1;
    int offsetY1 = area->y1;
    int offsetX2 = area->x2 +1 ;
    int offsetY2 = area->y2 + 1;
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetX1, offsetY1, offsetX2, offsetY2, color_p));
}

static void lvglTimerTask(void *param)
{
    while(1)
    {
        lv_timer_handler();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Should never return
    vTaskDelete(NULL);
}

static void createLvglTimerTask(void)
{
    xTaskCreatePinnedToCore(lvglTimerTask, "lvgl_timer", 10000, NULL, 4, NULL, 1);
}
