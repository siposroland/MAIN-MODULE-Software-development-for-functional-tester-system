/*
 * log.c
 *
 *  Created on: Jan 2, 2016
 *      Author: mori
 */

#include "log.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define LOGGER_MAXLEN 	250
#define LOGGER_TIMEOUT	10


void write_string(const char *data)
{
	uint8_t temp[200] = {0};
	strcpy(&temp, data);
	temp[strlen(data)] = '\r';
	temp[strlen(data)+1] = '\n';
	CDC_Transmit_HS((uint8_t *)temp, strlen(temp));
	//HAL_UART_Transmit(&hUartHandle, (uint8_t *)data, strlen(data), LOGGER_TIMEOUT);
}

void LOG(const char *__msg, ...)
{
    char buffer[LOGGER_MAXLEN] = {0};
    va_list ap;
    va_start (ap, __msg);
    vsprintf (buffer, __msg, ap);
    va_end (ap);

    write_string(buffer);
    //write_string("\n");
}

void LOG1(const char *__msg, ...)
{
    char buffer[LOGGER_MAXLEN] = {0};
    va_list ap;
    va_start (ap, __msg);
    vsprintf (buffer, __msg, ap);
    va_end (ap);

    write_string(buffer);
}

void LOG_ARRAY1(const uint8_t *data, int size)
{
    int i =0;
    for(i = 0; i < size; ++i)
    {
        LOG1("%02X ", data[i]);
    }
}

void LOG_ARRAY(const uint8_t *data, int size)
{
	LOG_ARRAY1(data, size);
    write_string("\r\n");
}
