#include "pico/stdlib.h"
//#include <pico/cyw43_arch.h>
#include "TFT_eSPI/TFT_eSPI.h"
#include "lvgl/lvgl.h"
#include "ui/ui.h"


//#define __UI_PROJECT_ROTATION__ 0  //should be overwritten by SquareLine Studio later (above v1.3.3), when it's done, comment out this line
//#define __UI_PROJECT_OFFSET_X__ 0
//#define __UI_PROJECT_OFFSET_Y__ 0

//#define BUILTIN_LED CYW43_WL_GPIO_LED_PIN
//#define TOUCH_IRQ_PIN 22


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_driver, const lv_area_t *area, lv_color_t *color_p )
{
    static TFT_eSPI * TFT_eSPI_ptr;

    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    TFT_eSPI_ptr = (TFT_eSPI*) disp_driver->user_data;
    TFT_eSPI_ptr->startWrite();
    TFT_eSPI_ptr->setAddrWindow( area->x1, area->y1, w, h );
    TFT_eSPI_ptr->pushColors( ( uint16_t * )&color_p->full, w * h, true );
    TFT_eSPI_ptr->endWrite();

    lv_disp_flush_ready( disp_driver );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    static TFT_eSPI * TFT_eSPI_ptr;
    uint16_t touchX = 0, touchY = 0;

    TFT_eSPI_ptr = (TFT_eSPI*) indev_driver->user_data;
    bool touched = TFT_eSPI_ptr->getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}


int main()
{
    enum ScreenSettings { PICO_FRAME_PERIOD = 10 /*milliseconds*/ ,
                          PICO_HORIZONTAL_RESOLUTION = 128,
                          PICO_VERTICAL_RESOLUTION   = 64,
                          PICO_SCREEN_ROTATION = /*__UI_PROJECT_ROTATION__*/ 0,
                          PICO_TOUCH_START_X = 300, PICO_TOUCH_START_Y=300, PICO_TOUCH_END_X = 3500, PICO_TOUCH_END_Y = 3500,
                          PICO_TOUCH_ROTATION = /*PICO_SCREEN_ROTATION &*/ 0, PICO_TOUCH_INVERT_X = 0, PICO_TOUCH_INVERT_Y = 1,
                          PICO_SCREENBUFFER_SIZE = (PICO_HORIZONTAL_RESOLUTION * PICO_VERTICAL_RESOLUTION / 10) };

    static uint16_t TouchScreen_Calibration_Data[5] = { PICO_TOUCH_START_X, PICO_TOUCH_END_X, PICO_TOUCH_START_Y, PICO_TOUCH_END_Y
                                                        , (PICO_TOUCH_ROTATION)|(PICO_TOUCH_INVERT_X<<1)|(PICO_TOUCH_INVERT_Y<<2) };

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf[ PICO_SCREENBUFFER_SIZE ];

    static TFT_eSPI TFT_eSPI_screen = TFT_eSPI(PICO_HORIZONTAL_RESOLUTION, PICO_VERTICAL_RESOLUTION); /* TFT instance */


    stdio_init_all(); //if (cyw43_arch_init()) return -1; //cyw43_arch_gpio_put(BUILTIN_LED,0);

    lv_init();
    TFT_eSPI_screen.begin();
    TFT_eSPI_screen.setRotation( PICO_SCREEN_ROTATION );
    TFT_eSPI_screen.setTouch(TouchScreen_Calibration_Data);

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, PICO_SCREENBUFFER_SIZE );

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = PICO_HORIZONTAL_RESOLUTION;
    disp_drv.ver_res = PICO_VERTICAL_RESOLUTION;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register( &disp_drv );
    lv_disp_set_default(disp);
    disp_drv.user_data = (void*) &TFT_eSPI_screen;

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );
    indev_drv.user_data = (void*) &TFT_eSPI_screen;

    ui_init();

    while (1) {
        lv_timer_handler(); // let the GUI do its work
        sleep_ms(PICO_FRAME_PERIOD);
    }

 return 0;
}

