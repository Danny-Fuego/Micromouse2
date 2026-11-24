#include "bt.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;

void BT_SendLine(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 1000);
}

void BT_Sendf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
}
