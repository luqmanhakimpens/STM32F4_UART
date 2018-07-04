
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

char buff[];

void USART3_IRQHandler(void) //USART1 USB_SERIAL ISR
{
    if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)//USART6 RX ISR
    {
    	USART_SendData(USART3,USART_ReceiveData(USART3));
    }
}

void USART6_init()
{
		GPIO_InitTypeDef			GPIO_InitStructure;
		USART_InitTypeDef			USART_InitStructure;
		NVIC_InitTypeDef			NVIC_InitStructure;

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable UART clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	  /* Configure USART Tx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure USART Rx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate 			= 115200;
	  USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits 			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity 				= USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART6, &USART_InitStructure);

	  /* Enable USART */
	  USART_Cmd(USART6, ENABLE);
	  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel 					= USART6_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}


void USART_SendString(USART_TypeDef* USARTx,char *stringBuff)
{
    while(*stringBuff)// Loop while there are more characters to send.
    {
        USART_SendData(USARTx, *stringBuff++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET); //wait for next character to send
    }
}

int main(void)
{
    /* System Init */
    SystemInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while(!RCC_WaitForHSEStartUp());

    //USART3_init();
    USART6_init();
    //USART3_init();

    while (1)
    {
    	sprintf(buff,"system core clock=%d",SystemCoreClock);
    	USART_SendString(USART6,buff);
		for(unsigned int q=0;q<=5000000;++q);

     }

}

#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

char buff[];

void USART3_IRQHandler(void) //USART1 USB_SERIAL ISR
{
    if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)//USART6 RX ISR
    {
    	USART_SendData(USART3,USART_ReceiveData(USART3));
    }
}

void USART6_init()
{
		GPIO_InitTypeDef			GPIO_InitStructure;
		USART_InitTypeDef			USART_InitStructure;
		NVIC_InitTypeDef			NVIC_InitStructure;

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable UART clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	  /* Configure USART Tx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure USART Rx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate 			= 115200;
	  USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits 			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity 				= USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART6, &USART_InitStructure);

	  /* Enable USART */
	  USART_Cmd(USART6, ENABLE);
	  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel 					= USART6_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}


void USART_SendString(USART_TypeDef* USARTx,char *stringBuff)
{
    while(*stringBuff)// Loop while there are more characters to send.
    {
        USART_SendData(USARTx, *stringBuff++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET); //wait for next character to send
    }
}

int main(void)
{
    /* System Init */
    SystemInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while(!RCC_WaitForHSEStartUp());

    //USART3_init();
    USART6_init();
    //USART3_init();

    while (1)
    {
    	sprintf(buff,"system core clock=%d",SystemCoreClock);
    	USART_SendString(USART6,buff);
		for(unsigned int q=0;q<=5000000;++q);

     }

}
