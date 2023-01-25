/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOs
  * @version V1.0
  * @date    24/10/2017
  * @brief   FreeRTOS Example project.
  ******************************************************************************
*/


/*
 *
 * Messages Queues
 * 2017-2018
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <lcd.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include "queue.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at which the temperature is read. */
#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )

/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

static void I2C2_Config(void);

static I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data);

static uint8_t I2C_read(uint8_t dev_addr, uint8_t reg_addr);

/* Configure the ADC */
static void prvSetupADC( void );

/* Simple LED toggle task. */
static void prvFlashTask1( void *pvParameters );

/* LCD activity task. */
static void prvLcdTask( void *pvParameters );

/* Configure the ADC */
static void prvSetupADC( void );

/* Button task. */
static void prvButtonTask( void *pvParameters );

static void prvTempTask(void *pvParameters);

static void prvTask4(void *pvParameters);

static void prvUSART2Interrupt ( void );
static void prvSetupEXTI1( void );

/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/


/* Task 1 handle variable. */
TaskHandle_t HandleTask1;

/* Task 2 handle variable. */
TaskHandle_t HandleTask2;

/* Task 3 handle variable. */
TaskHandle_t HandleTask3;

/* Task 4 handle variable. */
TaskHandle_t HandleTask4;

TaskHandle_t HandleTask5;

TaskHandle_t HandleTask6;

/*Create a Queue*/
QueueHandle_t xQueue;  /* Global variable. */

//Ex.2
QueueHandle_t xQueue1;  /* Global variable. */

//Ex.3
QueueHandle_t xQueue2;  /* Global variable. */

QueueHandle_t xQueue3;


typedef struct val{
	TickType_t tick;
	int32_t temp;
}val;

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupADC();


    I2C2_Config();
    prvSetupADC();
    prvSetupUSART2();
    prvUSART2Interrupt();
    prvSetupEXTI1();

    //Create a message queue with a size of 10 and element size of 1 byte
    xQueue = xQueueCreate(10, sizeof(char));

    //Check if the queue was created successfully
    if( xQueue == 0 ) {

    	/* Queue was not created and must not be used. */

    	}else
		{
    		/* Queue created successfully. */
		}

    //Create a message queue with a size of 10 and element size of 1 byte
    xQueue1 = xQueueCreate(1, sizeof(TickType_t));

    //Check if the queue was created successfully
    if( xQueue1 == 0 ) {

    	/* Queue was not created and must not be used. */

    	}else
		{
    		/* Queue created successfully. */
		}

    xQueue2 = xQueueCreate(20, sizeof(int32_t));
    if( xQueue2 == 0 ) {

       	/* Queue was not created and must not be used. */

       	}else
   		{
       		/* Queue created successfully. */
   		}

    xQueue3 = xQueueCreate(20, sizeof(val));
    if( xQueue3 == 0 ) {

       	/* Queue was not created and must not be used. */

       	}else
   		{
       		/* Queue created successfully. */
   		}

	/* Create the tasks */
 	xTaskCreate( prvLcdTask, "Lcd", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, &HandleTask1 );
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask2 );
    xTaskCreate( prvTempTask, "Temp", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY, &HandleTask3 );
    xTaskCreate( prvButtonTask, "Button",configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY, &HandleTask4 );

    xTaskCreate( prvTask4, "Usart", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleTask6 );
    /* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/


static void prvFlashTask1( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY );
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)));
	}
}

/*-----------------------------------------------------------*/


/* Example task to present characters in ther display. */
static void prvLcdTask( void *pvParameters )
{
	lcd_init( );
	char ulVar;

	for(;;)
	{
		xQueueReceive(xQueue, &ulVar, (TickType_t) portMAX_DELAY );
		lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, ulVar, 0xFFFF, 10 );
		vTaskDelay( (TickType_t ) 2000 / portTICK_RATE_MS);
	}
}
/*-----------------------------------------------------------*/


/* Temperature task - demo to read the ADC and get the temperature. */
/* Change this task accordingly. */
static void prvTempTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    int ADC1ConvertedValue=0;
    int32_t temp;
    char buf[30];

    TickType_t tick;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
    	vTaskDelayUntil ( &xLastExecutionTime, mainTEMP_DELAY);
        /* Read Sensor */
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET );

        ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        /*work with x 1000 to avoid float*/
        temp = (int32_t) ADC1ConvertedValue;
        temp = ( temp * 3300 ) / 4096; // finds mv
        temp = ((( 1400 - temp ) * 1000 ) / 430 ) + 250;

        /*The temp variable has the temperature value times 10 */

        //Ex.3

        tick= xTaskGetTickCount();

    	sprintf(buf, "%ld %ld\r\n", temp, tick); //todo apresentar no lcd

    	prvSendMessageUSART2(buf);

    }
}
/*-----------------------------------------------------------*/

static void prvTask4(void *pvParameters){
	lcd_init();
	char buf[30];
	val valores;
	for(;;){

	xQueueReceive(xQueue3, &valores, (TickType_t) portMAX_DELAY);
	sprintf(buf, "%ld %ld\r\n", valores.temp, valores.tick);

	prvSendMessageUSART2(buf);

	}
}


static void prvButtonTask( void *pvParameters ){
	//lcd_init( );
	//static BaseType_t pxHigherPriorityTaskWoken;

	TickType_t t1, t2 =0 , dif;

	char buf[30];
	for(;;){

		//xQueueReceiveFromISR(xQueue1, &t1, pxHigherPriorityTaskWoken);
		xQueueReceive(xQueue1, &t1, (TickType_t) portMAX_DELAY);
		//GPIO_WriteBit(GPIOB, GPIO_Pin_1, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)));
		dif= (t1-t2);
		//sprintf(buf, "1: %ld ,  2: %ld %ld\r\n", t1, t2, dif);

		t2=t1;
		prvSendMessageUSART2(buf);



		if(dif < 800){
			lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, '2', 0xFFFF, 10 );
			//lcd_draw_char( 0, 0, buf, 0xFFFF, 1 );
			//prvSendMessageUSART2(buf);
		}
		else{
			lcd_draw_char( 63-(5*10)/2, 79-(7*10)/2, '1', 0xFFFF, 10 );
		}
		/*
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
					USART_SendData(USART2, t1);
		USART_ClearFlag(USART2,USART_FLAG_TXE);*/
	}
}

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/



static void prvSetupGPIO( void )
{
    /* GPIO configuration - GREEN LED*/
    GPIO_InitTypeDef GPIO_InitStructure;



    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);



	//Inicialização I2C
	GPIO_InitTypeDef GPIO_InitTypeDefStruct;
	GPIO_StructInit(&GPIO_InitTypeDefStruct);
	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // SCL, SDA, SA0
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);


    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*-----------------------------------------------------------*/

void I2C2_Config(void)
{
	//Configuração I2C

	I2C_InitTypeDef I2C_InitStructure;

	I2C_StructInit(&I2C_InitStructure);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

}

void I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data)
{
	while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // while the bus is busy

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	I2C_GenerateSTART(I2C2, ENABLE); // send I2C start condition
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter); // send device slave address for write
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, reg_addr); // send the device internal register address
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C2, reg_data); // send data to register
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t I2C_read(uint8_t dev_addr, uint8_t reg_addr)
{
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // while the bus is busy

	I2C_GenerateSTART(I2C2, ENABLE); // send I2C start condition
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter); // send device slave address for write
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, reg_addr); //0x80 | reg_addr); // send the device internal register address
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C2, ENABLE); // repeated start
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Receiver); // send device slave address for read
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));

	uint8_t read_reg = I2C_ReceiveData(I2C2);

	I2C_GenerateSTOP(I2C2, ENABLE);
	I2C_AcknowledgeConfig(I2C2, DISABLE);

	return read_reg;
}

static void prvSetupADC( void )
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}

void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }

/*-----------------------------------------------------------*/



static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}
/*-----------------------------------------------------------*/


static void prvSetupADC( void )
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16,1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

}
/*-----------------------------------------------------------*/

static void prvUSART2Interrupt ( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

static void prvSetupEXTI1( void )
{
    /*NVIC configuration*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Configure Key Button EXTI Line to generate an interrupt on falling edge*/
    EXTI_InitTypeDef EXTI_InitStructure;

    /*Connect Key Button EXTI Line to Key Button GPIO Pin*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//Rising Edge
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}


