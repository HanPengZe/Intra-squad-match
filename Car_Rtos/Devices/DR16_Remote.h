#ifndef __DR16_REMOTE
#define __DR16_REMOTE

#include "dma.h"
#include "usart.h"
#endif
typedef struct
{
 struct
 {
 int16_t  ch0;
 int16_t  ch1;
 int16_t  ch2;
 int16_t  ch3;
 uint8_t s1;
 uint8_t s2;
 }rc;
 struct
 {
 int16_t x;
 int16_t y;
 int16_t z;
 uint8_t press_l;
 uint8_t press_r;
 }mouse;
 struct
 {
 uint16_t v;
 }key;
}RC_Ctl_t;
extern  RC_Ctl_t RC_Ctl;
extern uint8_t DBUS_RX_buffer[18];
