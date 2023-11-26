#include "DR16_Remote.h"
RC_Ctl_t RC_Ctl;
uint8_t DBUS_RX_buffer[18];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)
{
	RC_Ctl.rc.ch0 = (DBUS_RX_buffer[0] | (DBUS_RX_buffer [1] << 8)) & 0x07ff;
	RC_Ctl.rc.ch1 = ((DBUS_RX_buffer[1] >> 3) | (DBUS_RX_buffer [2] << 5)) & 0x07ff;
	RC_Ctl.rc.ch2 = ((DBUS_RX_buffer [2] >> 6) | (DBUS_RX_buffer [3] << 2) | (DBUS_RX_buffer [4] << 10)) & 0x07ff;
	RC_Ctl.rc.ch3 = ((DBUS_RX_buffer [4] >> 1) | (DBUS_RX_buffer [5] << 7)) & 0x07ff;
	RC_Ctl.rc.s1  = ((DBUS_RX_buffer [5] >> 4) & 0x000C) >> 2;
	RC_Ctl.rc.s2  = ((DBUS_RX_buffer [5] >> 4) & 0x0003);
	
	RC_Ctl.mouse.x  = DBUS_RX_buffer [6] | (DBUS_RX_buffer [7] << 8);
	RC_Ctl.mouse.y  = DBUS_RX_buffer [8] | (DBUS_RX_buffer [9] << 8);
	RC_Ctl.mouse.z  = DBUS_RX_buffer [10] | (DBUS_RX_buffer [11] << 8);
	RC_Ctl.mouse.press_l = DBUS_RX_buffer [12];
	RC_Ctl.mouse.press_r = DBUS_RX_buffer [13];
	RC_Ctl.key.v = DBUS_RX_buffer [14] | (DBUS_RX_buffer [15] << 8);
	
	RC_Ctl.rc.ch0-=1024;
  RC_Ctl.rc.ch1-=1024;
	RC_Ctl.rc.ch2-=1024;
	RC_Ctl.rc.ch3-=1024;
	
} 
  

