/*
 * protocol8086.c
 *
 *  Created on: 2020. 4. 29.
 *      Author: Jinsan Kwon
 */

#include <protocol8086_stm32g4.h>
#include <stdint.h>
#include <stddef.h>

static enum UART_STATE uartState;
static void (*parseDoneCallBack)(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5) = NULL;
void (*uartSend)(uint8_t *buf, uint8_t szBuf) = NULL;

static int parserState = 0;

void _encapsulateData(uint8_t *buf8, uint8_t one, uint8_t two, uint8_t three, uint8_t four, uint8_t five)
{
	buf8[0] = UART_STX;
	buf8[1] = one;
	buf8[2] = two;
	buf8[3] = three;
	buf8[4] = four;
	buf8[5] = five;
	buf8[6] = one + two + three + four + five;
	buf8[7] = UART_ETX;
}

void _parseDone(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5)
{
	extern uint8_t adcStatus;
	extern uint32_t adc_vbat;
	extern uint32_t adc_vacc;
	uint8_t sendBuf[8] = {0,};

	if(parseDoneCallBack != NULL) parseDoneCallBack(data1, data2, data3, data4, data5);
	else
	{
		/* Default Command Behaviour Parser */
		switch(data1)
		{
		case CMD_HELLO:
			_encapsulateData(sendBuf, CMD_HELLO2, PWR_MONITOR, 0, 0, 0);
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_HELLO2:
			break;
		case CMD_PING:
			_encapsulateData(sendBuf, CMD_PONG, PWR_MONITOR, 0, 0, 0);
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_PONG:
			break;
		case CMD_BKUP:
			if(data2 == DVR_RECORDER && data3 == 1)
			{
				// cmd_state = CMD_BKUP_GOT;
				;
			}
			break;
		case CMD_BKOK:
			break;
		case CMD_HALT:
			break;
		case CMD_HTOK:
			break;
		case CMD_STAT:
			_encapsulateData(sendBuf, CMD_STAT_RSP, PWR_MONITOR, adcStatus, (uint8_t)(adc_vbat * 183 / 4096), (uint8_t)(adc_vacc * 183 / 4096));
			if(uartSend != NULL) uartSend(sendBuf, 8);
			break;
		case CMD_STAT_RSP:
			break;
		default:
			break;
		}
	}
}


void parserInit()
{
	parserState = 1;
}

void setParseDoneCallback(void (*func)(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t))
{
	parseDoneCallBack = func;
}

void setUartSendFunc(void (*func)(uint8_t *, uint8_t))
{
	uartSend = func;
}

void sendCmd(uint8_t cmd, uint8_t one, uint8_t two, uint8_t three, uint8_t four)
{
	uint8_t sendBuf[8] = {0,};
	_encapsulateData(sendBuf, cmd, one, two, three, four);
	if(uartSend != NULL) uartSend(sendBuf, 8);
}

void parseData(uint8_t data)
{
	static uint8_t databuf[8] = {0,};
	static uint32_t recvdcnt = 0;
	uint8_t tmpChksum = 0;

	if(parserState)
	{
		switch(uartState)
		{
		case UART_STATE_IDLE:
			if(data == UART_STX)
			{
				recvdcnt = 0;
				uartState = UART_STATE_RECEIVING;
				databuf[recvdcnt++] = data;
			}
			break;
		case UART_STATE_RECEIVING:
			if(recvdcnt < 8)
			{
				databuf[recvdcnt++] = data;
				if(recvdcnt == 8)
				{
					if(data == UART_ETX)
					{
						tmpChksum = databuf[1] + databuf[2] + databuf[3] + databuf[4] + databuf[5];
						if(tmpChksum == databuf[6])
						{
							_parseDone(databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
						}
					}
					//recvdcnt = 0;
					uartState = UART_STATE_IDLE;
				}
			}
			else
			{
				// Highly unlikely case
				//recvdcnt = 0;
				uartState = UART_STATE_IDLE;
			}
			break;
		default:
			break;
		}
	}
}
