#ifndef LKM_BNO086_APP_H
#define LKM_BNO086_APP_H

#include <stdbool.h>

/* Call once after HAL_Init(), SystemClock_Config(), GPIO/SPI2/USART6 init. */
void LKM_BNO086_App_Init(void);
bool LKM_BNO086_App_ProductIdVerified(void);

#endif
