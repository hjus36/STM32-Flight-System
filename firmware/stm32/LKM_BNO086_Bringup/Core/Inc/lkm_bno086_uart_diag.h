#ifndef LKM_BNO086_UART_DIAG_H
#define LKM_BNO086_UART_DIAG_H

#include <stdint.h>

/* UART6 debug monitor for the existing BNO086 diagnostic application.
 * NOT a flight-loop transport. No SPI, PWM, ESC, arm or motor commands.
 * Call Init once AFTER LKM_BNO086_Diag_Init and MX_USART6_UART_Init.
 * Call Service AFTER LKM_BNO086_Diag_Service in the main diagnostic while(1).
 * Never call from an interrupt, ISR, or real-time PID update.
 */
void LKM_BNO086_UartDiag_Init(void);
void LKM_BNO086_UartDiag_Service(void);
uint32_t LKM_BNO086_UartDiag_TxErrors(void);

#endif /* LKM_BNO086_UART_DIAG_H */
