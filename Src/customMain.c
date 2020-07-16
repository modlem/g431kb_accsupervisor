/*
 * customMain.c
 *
 *  Created on: 2020. 4. 29.
 *      Author: Jinsan Kwon
 */
//define HELLO_WORLD
//define ECHO_TEST
//define LOWPOWER_TEST
#define RELAY_SERVICE

#define _USART_DMA_BUF_LENGTH		512

#include "customMain.h"
#include "main.h"
#include "protocol8086_stm32g4.h"

extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc2;
static uint8_t _usart1DMAbuffer[_USART_DMA_BUF_LENGTH] = {0, };
//static uint8_t _usart2DMAbuffer[_USART_DMA_BUF_LENGTH] = {0, };
static uint8_t sendErrCnt = 0;
static uint8_t standbyPending = 0;
static volatile uint8_t txDoneFlag = 0;

#ifdef HELLO_WORLD
uint8_t pszData[] = "Hello\r\n\0";
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	txDoneFlag = 1;
}

void customUsartSendFunc(uint8_t *data, uint8_t len)
{
	if(HAL_OK != HAL_UART_Transmit_DMA(&huart1, data, len)) sendErrCnt++;
	while(txDoneFlag == 0);
	txDoneFlag = 0;
}

void customSetup()
{
#ifdef HELLO_WORLD
	HAL_UART_Transmit_DMA(&huart2, pszData, 7);
#endif
#ifdef ECHO_TEST
	HAL_UART_Receive_DMA(&huart1, _usart1DMAbuffer, _USART_DMA_BUF_LENGTH);
	HAL_UART_Receive_DMA(&huart2, _usart2DMAbuffer, _USART_DMA_BUF_LENGTH);
#endif

#ifdef LOWPOWER_TEST
	RTC_TimeTypeDef theTime;
	RTC_DateTypeDef theDate;
	uint32_t nowTick;

	parserInit();
	setUartSendFunc(customUsartSendFunc);
	HAL_UART_Receive_DMA(&huart1, _usart1DMAbuffer, _USART_DMA_BUF_LENGTH);
	HAL_Delay(20);
	nowTick = HAL_GetTick();

	HAL_RTC_GetTime(&hrtc, &theTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &theDate, RTC_FORMAT_BCD);
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{
		//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		sendCmd(CMD_HELLO, PWR_MONITOR, (uint8_t)nowTick, theTime.Seconds, 1);
	}
	else
	{
		sendCmd(CMD_HELLO, PWR_MONITOR, (uint8_t)nowTick, theTime.Seconds, 0);
	}
#ifdef LOWPOWER_STANDBY
	HAL_PWR_EnterSTANDBYMode();
#endif
#endif

#ifdef RELAY_SERVICE
	// TODO: UART protocol (STM to JETSON) not implemented
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	parserInit();
	setUartSendFunc(customUsartSendFunc);
	HAL_UART_Receive_DMA(&huart1, _usart1DMAbuffer, _USART_DMA_BUF_LENGTH);
	HAL_Delay(20);
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
	{
		sendCmd(CMD_HELLO, PWR_MONITOR, 0, 0, 0);
	}
#endif
}

void usartProcess()
{
	static uint32_t usart1_oldPos = 0;
	uint32_t usart1_pos = 0;

	usart1_pos = _USART_DMA_BUF_LENGTH - huart1.hdmarx->Instance->CNDTR;
	if(usart1_pos != usart1_oldPos)
	{
		if(usart1_pos > usart1_oldPos)
		{
			while(usart1_oldPos < usart1_pos)
			{
				parseData(_usart1DMAbuffer[usart1_oldPos++]);
			}
		}
		else
		{
			while(usart1_oldPos < _USART_DMA_BUF_LENGTH)
			{
				parseData(_usart1DMAbuffer[usart1_oldPos++]);
			}
			usart1_oldPos = 0;
			while(usart1_oldPos < usart1_pos)
			{
				parseData(_usart1DMAbuffer[usart1_oldPos++]);
			}
		}
	}
	if(usart1_oldPos >= _USART_DMA_BUF_LENGTH) usart1_oldPos = 0;
}

uint8_t adcStatus = 0;
uint32_t lastCriticalTick = 0;
#define V_ACC		(1 << 1)
#define V_BAT		(1 << 2)
#define V_CRITICAL	(1 << 3)
#define V_MASK	(V_ACC | V_BAT)

// Div: 18.3v; x * 446777 / 100000
// Conf 1.
// Critical: 11.799v, ON Threshold: 11.9959v
//define V_BAT_MIN	2641
//define V_BAT_THR	2685
// Conf 2.
// Critical: 11.2v, ON Threshold: 11.5v
#define V_BAT_MIN	2507
#define V_BAT_THR	2574

#define V_ACC_MIN	V_BAT_MIN
#define V_ACC_THR	V_BAT_THR
#define ADC_SAMPLE_CNT	10
uint32_t adc_vbat = 0;
uint32_t adc_vacc = 0;

typedef enum
{
	CMD_NULL,
	CMD_HELLO_SENT,
	CMD_HELLO_GOT,
	CMD_BKUP_SENT,
	CMD_BKUP_GOT,
	CMD_BKOK_SENT,
	CMD_BKOK_GOT,
	CMD_HALT_SENT,
	CMD_HALT_GOT,
	CMD_HTOK_SENT,
	CMD_HTOK_GOT,
} Cmd_State;

typedef enum
{
	JST_IDLE,
	JST_BACKUP_PROCESSING,
	JST_DOWN,
} Jetson_State;

Cmd_State cmd_state = CMD_NULL;
Jetson_State jst_state = JST_IDLE;
uint32_t lastBkupRqstSent = 0;
static uint8_t bkupRqstSentCnt = 0;

// TODO: UART protocol based state negotiation
Jetson_State getJetsonState()
{
	return jst_state;
}

void adcProcess()
{
	uint32_t acc_avg = 0;
	uint32_t bat_avg = 0;
	uint32_t idx = 0;
	uint8_t tmpState = 0;

	do
	{
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		acc_avg += HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		bat_avg += HAL_ADC_GetValue(&hadc2);
	} while(++idx < ADC_SAMPLE_CNT);

	adc_vacc = acc_avg / ADC_SAMPLE_CNT;
	adc_vbat = bat_avg / ADC_SAMPLE_CNT;

	if(bat_avg < (V_BAT_MIN * ADC_SAMPLE_CNT)) tmpState = V_CRITICAL;
	else
	{
		if(bat_avg > (V_BAT_THR * ADC_SAMPLE_CNT))
		{
			tmpState = V_BAT;
		}
		if(acc_avg > (V_ACC_THR * ADC_SAMPLE_CNT))
		{
			tmpState = V_BAT | V_ACC;
		}
	}

	adcStatus = tmpState;
	//sendCmd(((adc_vacc >> 8) & 0xff), (adc_vacc & 0xff), ((adc_vbat >> 8) & 0xff), (adc_vbat & 0xff), 0);
}

uint8_t relayState = 0;

void relayOpen()
{
	HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PB7_Pin);
	HAL_PWREx_EnablePullUpPullDownConfig();
	relayState = 0;
}

void relayClose()
{
	HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_SET);
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PB7_Pin);
	HAL_PWREx_EnablePullUpPullDownConfig();
	relayState = 1;
}

uint8_t isJetsonAlive()
{
	uint8_t idx;
	uint8_t aliveCnt = 0;

	for(idx = 0; idx < 10; idx++)
	{
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, PB6_Pin)) aliveCnt++;
		HAL_Delay(50);
	}
	return aliveCnt;
}

void relayProcess()
{
#if 0
	RTC_TimeTypeDef theTime;
	RTC_DateTypeDef theDate;

	HAL_RTC_GetDate(&hrtc, &theDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &theTime, RTC_FORMAT_BIN);
#endif
	if(adcStatus & V_BAT)
	{
		//adcStatus &= ~(V_CRITICAL);
		if(adcStatus & V_ACC)
		{
			// On-line -> Close relay
			relayClose();
		}
		else
		{
			// Off-line -> Send backup command
			if(isJetsonAlive() > 0)
			{
				relayClose();
#if 0
				if(cmd_state != CMD_BKUP_GOT)
				{
					// TODO: We shouldn't use HAL_GetTick - it DOESN'T tick after entering low power mode.
					// TODO: Use RTC GetTime function instead.
					if(cmd_state == CMD_BKUP_SENT && lastBkupRqstSent > HAL_GetTick())
					{
						// Do nothing; wait for 5 second...
					}
					else
					{
						standbyPending = 1;
						sendCmd(CMD_BKUP, PWR_MONITOR, 0, 0, 0);
						cmd_state = CMD_BKUP_SENT;
						lastBkupRqstSent = HAL_GetTick() + 5000;
						bkupRqstSentCnt++;
						// No response -> Open relay
						if(bkupRqstSentCnt > 10) relayOpen();
					}
				}
#endif
			}
			else
			{
				// Jetson is down -> Open relay
				relayOpen();
			}
		}
	}
	else
	{
#if 0
		// Critical battery -> Send immediate shutdown command & wait 3 min
		// If situation still exists, open relay
		if(adcStatus & V_CRITICAL)
		{
			if(lastCriticalTick <= HAL_GetTick())
			{
				// Open relay
				relayOpen();
			}
		}
		else
		{
			adcStatus |= V_CRITICAL;
			lastCriticalTick = HAL_GetTick() + 180000;
		}
#else
		relayOpen();
#endif
	}
}

void customLoop()
{
#ifdef RELAY_SERVICE
#if 0
	// DEBUG
	{
	RTC_TimeTypeDef theTime;
	RTC_DateTypeDef theDate;
	uint32_t nowTick;

	nowTick = HAL_GetTick();

	HAL_RTC_GetTime(&hrtc, &theTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &theDate, RTC_FORMAT_BCD);
	sendCmd(CMD_HELLO, PWR_MONITOR, (uint8_t)nowTick, theTime.Seconds, 0);
	}
#endif

	adcProcess();
	usartProcess();
	relayProcess();

	if(standbyPending == 0)
	{
		if(relayState == 0)
		{
			HAL_PWR_EnterSTANDBYMode();
		}
		else
		{
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
			HAL_ResumeTick();
		}
	}
#endif

#ifdef LOWPOWER_TEST
	RTC_TimeTypeDef theTime;
	RTC_DateTypeDef theDate;
	uint32_t nowTick;

	nowTick = HAL_GetTick();

	HAL_RTC_GetTime(&hrtc, &theTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &theDate, RTC_FORMAT_BCD);
	sendCmd(CMD_HELLO, PWR_MONITOR, (uint8_t)nowTick, theTime.Seconds, 0);
#ifdef LOWPOWER_STANDBY
	HAL_PWR_EnterSTANDBYMode();
#else
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	HAL_ResumeTick();
#endif
#endif

#ifdef ECHO_TEST
	static uint32_t uart1_oldPos = 0;
	static uint32_t usart2_oldPos = 0;
	uint32_t uart1_pos = 0;
	uint32_t usart2_pos = 0;

	// USART 1
	uart1_pos = _USART_DMA_BUF_LENGTH - huart1.hdmarx->Instance->CNDTR;
	if(uart1_pos != uart1_oldPos)
	{
		if(uart1_pos > uart1_oldPos)
		{
			if(HAL_OK == HAL_UART_Transmit_DMA(&huart1, &_usart1DMAbuffer[uart1_oldPos], uart1_pos - uart1_oldPos)) uart1_oldPos = uart1_pos;
		}
		else
		{
			if(HAL_OK == HAL_UART_Transmit_DMA(&huart1, &_usart1DMAbuffer[uart1_oldPos], _USART_DMA_BUF_LENGTH - uart1_oldPos))
			{
				uart1_oldPos = 0;
				if(HAL_OK == HAL_UART_Transmit_DMA(&huart1, &_usart1DMAbuffer[uart1_oldPos], uart1_pos)) uart1_oldPos = uart1_pos;
			}
		}
	}
	if(uart1_oldPos >= _USART_DMA_BUF_LENGTH) uart1_oldPos = 0;

	// USART 2
	usart2_pos = _USART_DMA_BUF_LENGTH - huart2.hdmarx->Instance->CNDTR;
	if(usart2_pos != usart2_oldPos)
	{
		if(usart2_pos > usart2_oldPos)
		{
			if(HAL_OK == HAL_UART_Transmit_DMA(&huart2, &_usart2DMAbuffer[usart2_oldPos], usart2_pos - usart2_oldPos)) usart2_oldPos = usart2_pos;
		}
		else
		{
			if(HAL_OK == HAL_UART_Transmit_DMA(&huart2, &_usart2DMAbuffer[usart2_oldPos], _USART_DMA_BUF_LENGTH - usart2_oldPos))
			{
				usart2_oldPos = 0;
				if(HAL_OK == HAL_UART_Transmit_DMA(&huart2, &_usart2DMAbuffer[usart2_oldPos], usart2_pos)) usart2_oldPos = usart2_pos;
			}
		}
	}
	if(usart2_oldPos >= _USART_DMA_BUF_LENGTH) usart2_oldPos = 0;
#endif
}
