
/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <lcd.h>
#include <stm32f10x.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include "queue.h"
#include "semphr.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)

#define DEBOUNCE_TIME_MS 100

char OUTX_L = 0;
char OUTX_H = 0;
int16_t OUTX = 0;
char OUTY_L = 0;
char OUTY_H = 0;
int16_t OUTY = 0;
char OUTZ_L = 0;
char OUTZ_H = 0;
int16_t OUTZ = 0;

char buffer[15];
char bufferpc[30];

/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY			( ( TickType_t ) 1000 / portTICK_RATE_MS )
/* The rate at which the temperature is read. */
#define mainTEMP_DELAY			( ( TickType_t ) 100 / portTICK_RATE_MS )

/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Simple LED toggle task. */
static void prvFlashTask1( void *pvParameters );

/* LCD activity task. */
static void prvLcdTask( void *pvParameters );
/* ADC temperature read task. */
static void prvTempTask( void *pvParameters );

static void prvEixos(void *pvParameters);

static void prvBater(void *pvParameters);

/* Button task. */
static void prvButtonTask( void *pvParameters );

static void prvUSART2Interrupt ( void );
static void prvSetupEXTI15_10(void);
static void prvUsartTask( void *pvParameters );

/* Configure the ADC */
static void prvSetupADC( void );

int flag_parar = 0;

/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/***************************************/

void SPI(void);

uint8_t SPI_Send(uint8_t data);


/* Task 1 handle variable. */
TaskHandle_t HandleTask1;

/* Task 2 handle variable. */
TaskHandle_t HandleTask2;

/* Task 3 handle variable. */
TaskHandle_t HandleTask3;

/* Task 4 handle variable. */
TaskHandle_t HandleTask4;

/* Task 5 handle variable. */
TaskHandle_t HandleTask5;

/* Task 5 handle variable. */
TaskHandle_t HandleTask6;

/* Task 5 handle variable. */
TaskHandle_t HandleTask7;


/*Create a Queue*/
QueueHandle_t xQueue;  /* Global variable. */

QueueHandle_t button_queue;  /* Global variable. */

QueueHandle_t xQueue3;

/* Semaphore Binary Handle Variable */

SemaphoreHandle_t xSemaphoreBinary;

SemaphoreHandle_t xSemaphoreBinary1;

SemaphoreHandle_t xSemaphoreBinary2;

uint32_t tickPrev = 0;
typedef struct button_t {
    uint16_t gpio_pin;
    GPIO_TypeDef* gpio_port;
    char character;
    uint32_t last_press_time;
    uint8_t state;
} button_t;

typedef struct eixos{
	int16_t OUTX;
	int16_t OUTY;
	int16_t OUTZ;
}eixos;

//Declaração dos botões
button_t button_PC10 = {GPIO_Pin_10, GPIOC, 'I', 0, 0}; //Iniciar
button_t button_PC11 = {GPIO_Pin_11, GPIOC, 'R', 0, 0}; //Resume
button_t button_PC13 = {GPIO_Pin_13, GPIOC, 'P', 0, 0}; //Parar


typedef struct val{
	TickType_t tick;
	int32_t temp;
}val;

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupUSART2();
    prvUSART2Interrupt();
    prvSetupADC();
    prvSetupEXTI15_10();

    SPI();


    //Create a message queue with a size of 10 and element size of 1 byte
    xQueue = xQueueCreate(40, sizeof(eixos));

    //Check if the queue was created successfully
    if( xQueue == 0 ) {

    	/* Queue was not created and must not be used. */

    	}else
		{
    		/* Queue created successfully. */
		}


    button_queue = xQueueCreate(2, sizeof(button_queue));
    if( button_queue == 0 ) {

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

	xSemaphoreBinary = xSemaphoreCreateBinary();
	if( xSemaphoreBinary == NULL )
	{
		/*  Error creating the semaphore, it cannot be used. */
	}

	xSemaphoreBinary1 = xSemaphoreCreateBinary();
	if( xSemaphoreBinary1 == NULL )
	{
		/*  Error creating the semaphore, it cannot be used. */
	}
	xSemaphoreBinary2 = xSemaphoreCreateBinary();
	if( xSemaphoreBinary2 == NULL )
	{
		/*  Error creating the semaphore, it cannot be used. */
	}

	/* Create the tasks */
 	xTaskCreate( prvLcdTask, "Lcd", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY+1, &HandleTask1 );
 	xTaskCreate( prvFlashTask1, "Flash1", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY+2, &HandleTask2 );
    xTaskCreate( prvButtonTask, "Button",configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY+1, &HandleTask3 );
    xTaskCreate( prvTempTask, "Temp", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY+1, &HandleTask4 );

    xTaskCreate(prvBater, "Bater", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY+1, &HandleTask6 );

    xTaskCreate(prvEixos, "Eixos", configMINIMAL_STACK_SIZE+100, NULL, mainFLASH_TASK_PRIORITY, &HandleTask5 );

    /* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}

/*-----------------------------------------------------------*/

uint8_t SPI_Send(uint8_t data){

	 while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET );
	 SPI_I2S_SendData(SPI2, data);
	 SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);
	 while( SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET );
	 SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);

	return SPI_I2S_ReceiveData(SPI2);
}

void SPI(void){

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Full Duplex
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //idles high
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; // CPHA = 1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // NSS software
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);

	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);

	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_RESET);
	SPI_Send(0x20);    // CTRL_REG1
	SPI_Send(0b11000111);
	//SPI_Send(0b11000111); //Device on, decimate by 512 (40hz), Z,Y e X Axis enable
	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);


	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
	SPI_Send(0x21); // CTRL_REG2
	SPI_Send(0b10000000); // Full Scale = +-6g, Update continuo, DRDY enable, little endian (LSB do Eixo X_L = 28h)
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
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

static void prvBater(void *pvParameters){

	eixos eixos;
	char buf[30];
	char buffer[30];

	xSemaphoreGive(xSemaphoreBinary1);
	xSemaphoreGive(xSemaphoreBinary);

	for(;;){

		xQueueReceive(xQueue, &eixos, (TickType_t) 20);

		if(eixos.OUTX>300 || eixos.OUTX<-300){

			prvSendMessageUSART2(buf);
			vTaskSuspend(HandleTask5);
			sprintf(buf, "Bateu");
			lcd_draw_string(30,100,buf, 0xFFFF, 2);

		}else if(eixos.OUTY>300 || eixos.OUTY<-300){

			prvSendMessageUSART2(buf);
			vTaskSuspend(HandleTask5);
			sprintf(buf, "Bateu");
			lcd_draw_string(30,100,buf, 0xFFFF, 2);

		}else if(eixos.OUTZ>200 || eixos.OUTZ<-600){

			prvSendMessageUSART2(buf);
			vTaskSuspend(HandleTask5);
			sprintf(buf, "Bateu");
			lcd_draw_string(30,100,buf, 0xFFFF, 2);

		}
		prvSendMessageUSART2("----------------------------------------\r\n");
		prvSendMessageUSART2("Estado Filas de Mensagem: \r\n");
	    unsigned int qStatus_xQueue = uxQueueMessagesWaiting(xQueue );
	    sprintf(buffer, " Fila mensagem Eixos - %d \r\n ", qStatus_xQueue);
	    prvSendMessageUSART2(buffer);

        unsigned int qStatus_xQueue3 = uxQueueMessagesWaiting(xQueue3);
        sprintf(buffer, " Fila mensagem Temperatura - %d \r\n ", qStatus_xQueue3);
        prvSendMessageUSART2(buffer);

        unsigned int qStatus_button = uxQueueMessagesWaiting(button_queue);
        sprintf(buffer, " Fila mensagem Botões - %d \r\n ", qStatus_button);
        prvSendMessageUSART2(buffer);
        prvSendMessageUSART2("----------------------------------------\r\n");

		xSemaphoreGive(xSemaphoreBinary1); //ler eixos quando termina de verificar acidente
		xSemaphoreGive(xSemaphoreBinary); //sync com LCD
	}
}


void prvEixos(void *parameters){

	eixos eixos;

	for(;;){

	xSemaphoreTake( xSemaphoreBinary1, (TickType_t) portMAX_DELAY); //sinc com prvBater

	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET); //Eixo X
	OUTX_L = SPI_Send(0x28|0x80); //Register adress do OUTX_L
	OUTX_L = SPI_Send(0x00); //ler o output do OUTX_L
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
	OUTX_H = SPI_Send(0x29|0x80); //Register adress do OUTX_H
	OUTX_H = SPI_Send(0x00);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	eixos.OUTX = (OUTX_H << 8) + OUTX_L; //Junta os valores High do OUTX com os Low


	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET); //Eixo Y
	OUTY_L = SPI_Send(0x2A|0x80);
	OUTY_L = SPI_Send(0x00);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
	OUTY_H = SPI_Send(0x2B|0x80);
	OUTY_H = SPI_Send(0x00);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	eixos.OUTY = (OUTY_H << 8) + OUTY_L;


	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET); //Eixo Z
	OUTZ_L = SPI_Send(0x2C|0x80);
	OUTZ_L = SPI_Send(0x00);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
	OUTZ_H = SPI_Send(0x2D|0x80);
	OUTZ_H = SPI_Send(0x00);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);

	eixos.OUTZ = (OUTZ_H << 8) + OUTZ_L;


	xQueueSendToBack(xQueue, (void *) &eixos, (TickType_t) 20);

	}
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



static void prvLcdTask( void *pvParameters )
{
	lcd_init( );
	char buf[30];

	val valores;

	eixos eixos;

	xSemaphoreGive(xSemaphoreBinary2);
	for(;;)
	{

		xSemaphoreTake( xSemaphoreBinary, (TickType_t) portMAX_DELAY); //sync com prvBater, espera que este verifique se bateu
		xQueueReceive(xQueue3, &valores, (TickType_t) portMAX_DELAY);
		if(flag_parar == 0){
			xQueuePeek(xQueue, &eixos, (TickType_t) portMAX_DELAY); // recebe do prvEixos o X, Y e Z
		}
		prvSendMessageUSART2("----------------------------------------\r\n");
		prvSendMessageUSART2("valores:\r\n");

		sprintf(buf, "X: %ld ", eixos.OUTX);
		lcd_draw_string(40,20,buf, 0xFFFF, 1);
		prvSendMessageUSART2(buf);
		prvSendMessageUSART2("\r\n");

		sprintf(buf, "Y: %ld   ", eixos.OUTY);
		lcd_draw_string(40,30,buf, 0xFFFF, 1);
		prvSendMessageUSART2(buf);
		prvSendMessageUSART2("\r\n");

		sprintf(buf, "Z: %ld   ", eixos.OUTZ);
		lcd_draw_string(40,40,buf, 0xFFFF, 1);
		prvSendMessageUSART2(buf);
		prvSendMessageUSART2("\r\n");

		sprintf(buf, "Temp: %ld", valores.temp);
		lcd_draw_string(40,0,buf, 0xFFFF, 1);
		prvSendMessageUSART2(buf);
		prvSendMessageUSART2("\r\n");

		sprintf(buf, "Tick: %ld", valores.tick);
		lcd_draw_string(40,10,buf, 0xFFFF, 1);
		prvSendMessageUSART2(buf);
		prvSendMessageUSART2("\r\n");

		prvSendMessageUSART2("----------------------------------------\r\n");

		xSemaphoreGive(xSemaphoreBinary2); //sync com temp

	}
}
/*-----------------------------------------------------------*/



/* Temperature task - demo to read the ADC and get the temperature. */

static void prvTempTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;
    int ADC1ConvertedValue=0;
    int32_t temp;

    val valores;
    char buf[10];


    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{

    	xSemaphoreTake( xSemaphoreBinary2, (TickType_t) portMAX_DELAY);

    	//vTaskDelayUntil ( &xLastExecutionTime, mainTEMP_DELAY);
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

        valores.temp = temp;
        valores.tick = xTaskGetTickCount() - tickPrev;

        xQueueSendToBack(xQueue3, (void *) &valores, (TickType_t) 10);

    }
}


/*-----------------------------------------------------------*/

static void prvButtonTask( void *pvParameters ){


	char buf[30];
	char buf_tick[60];
    char button_char;
    char buffer[10];

    val valores;

    char character;


    for(;;){

		xQueueReceive(button_queue, (void *) &button_char, portMAX_DELAY);
		lcd_draw_char( 60, 60, button_char, 0xFFFF, 1 );
		if(button_char == 'I'){
			if(character != 'P'){
			//vTaskDelay( (TickType_t) 2000 / portTICK_PERIOD_MS);
			sprintf(buf, "Andar");
			lcd_draw_string(30,100,buf, 0xFFFF, 2);
			xQueueReset(xQueue);
			vTaskResume(HandleTask5);//Iniciar depois de bater
			}
		}else if(button_char == 'P'){
			sprintf(buf, "Parou");
			lcd_draw_string(30,100,buf, 0xFFFF, 2);
			vTaskSuspend(HandleTask5); //lcd
			flag_parar = 1;
			character = 'P';
		}else if(button_char == 'R'){ // so funciona quando arrancamos do parado
			if(character == 'P'){

				xQueueReset(xQueue);
				character = 'R';
				sprintf(buf, "Ligado");
				lcd_draw_string(30,100,buf, 0xFFFF, 2);
				tickPrev = xTaskGetTickCount();

				flag_parar = 0;
				vTaskResume(HandleTask5);
			}

		}
		prvSendMessageUSART2(buf);

		xQueueReset(button_queue);
		GPIO_WriteBit(GPIOB, GPIO_Pin_1, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)));


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
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE );

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* Enable GPIOC clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE );


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //Enable spi2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // SPI CS/SS

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //MISO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //MOSI
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //SCK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //RDY
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // CS2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}
/*-----------------------------------------------------------*/


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



void prvSetupEXTI15_10(void){

    /*NVIC configuration*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Configure Key Button EXTI Line to generate an interrupt on falling edge*/
    EXTI_InitTypeDef EXTI_InitStructure;

    /*Connect Key Button EXTI Line to Key Button GPIO Pin*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

    EXTI_InitStructure.EXTI_Line = EXTI_Line10 | EXTI_Line11 | EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//Rising Edge
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

}
