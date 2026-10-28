/*
 * LCD_REMAIN.c
 *
 *  Created on: 2026Äê10ÔÂ19ÈÕ
 *      Author: Freed
 */
#include"LCD_REMAIN.h"
void tft180_Show_READY()
{
    tft180_init();
    tft180_full(0x0000);
    tft180_set_color(0xffff,0x0000);
    tft180_show_string(0,0,"Ready Start");
}
