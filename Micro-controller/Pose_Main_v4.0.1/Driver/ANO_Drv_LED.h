#ifndef __ANO_DRV_LED_H__
#define __ANO_DRV_LED_H__

#include "include.h"

//void ANO_LED_Init(void);
//void ANO_LED_M_ON(void);
//void ANO_LED_M_OFF(void);
//void ANO_LED_0_ON(void);
//void ANO_LED_0_OFF(void);
//void ANO_LED_1_ON(void);
//void ANO_LED_1_OFF(void);
//
void LED_1ms_DRV(void);
//void LED_Dyty(float );
//
//u8 led_breath(float,u16 T);
//u8 led_flash(float dT,u16 group_n,u16 on_ms,u16 off_ms,u16 group_dT_ms);
void LED_Flash_Duty(u16 _on_time);
void LED_Flash_Control(u8 _en);

extern u8 LED_warn;
extern u8 led_duty;

void POSE_LED_Init(void);
void POSE_LED_0_ON(void);
void POSE_LED_0_OFF(void);
#endif

