#ifndef __PROTOCOL8086_H__
#define __PROTOCOL8086_H__

#include <stdint.h>

#define UART_STX 0x80
#define UART_ETX 0x86

#define CMD_HELLO	0x01
#define CMD_HELLO2	0x02
#define CMD_PING	0x03
#define CMD_PONG	0x04
#define CMD_BKUP	0x05
#define CMD_BKOK	0x06
#define CMD_HALT	0x07
#define CMD_HTOK	0x08
#define CMD_STAT	0x09
#define CMD_STAT_RSP	0x0a

#define PWR_MONITOR		1
#define DVR_RECORDER	2

enum UART_STATE {
	UART_STATE_IDLE,
	UART_STATE_RECEIVING,
};


void parserInit();
void setParseDoneCallback(void (*func)(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t));
void setUartSendFunc(void (*func)(uint8_t *, uint8_t));
void sendCmd(uint8_t cmd, uint8_t one, uint8_t two, uint8_t three, uint8_t four);
void parseData(uint8_t data);

#endif
