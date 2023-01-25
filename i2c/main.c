#include "stm32f10x.h"
#include <lcd.h>
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mma8452q.h>

USART_InitTypeDef USART_InitStructure;

int flag_play = 1;
int timer = 100;
int health_points;
int pos_x;
int pos_y;
int block_x;
int block_y;
int lower_limit_x = 0;
int lower_limit_y = 1;
int upper_limit_x = 11;
int upper_limit_y = 12;
int score=0;

char buffer[10];
uint8_t X ,Y ,Z;
int flag_game_over;

int delay(uint32_t a)
{
	for(int i =0; i<a; i++);
	return 0;
}

void lcd_clear()
{
	lcd_draw_fillrect(0,0,128,160,0x0000);
}

int random_pos_x(){
	int pos_x = rand() % 12;
	return pos_x;
}

int random_pos_y(){
	int pos_y = rand() % 11 + 1;
	return pos_y;
}

void RCC_Config_HSE_PLL_Max() // Clock a 72 MHz
{
	RCC_HSEConfig(RCC_HSE_ON);
	ErrorStatus HSEStartUpStatus;
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_SetLatency(FLASH_Latency_2);
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
		RCC_PLLCmd(ENABLE);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	else
	while(1);
}

void Init(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Inicialização do Led0

	GPIO_InitTypeDef GPIO_InitStructure_1;
	GPIO_InitStructure_1.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure_1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure_1.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure_1);

	//Inicialização do SW5

	GPIO_InitTypeDef GPIO_InitStructure_2;
	GPIO_InitStructure_2.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure_2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure_2.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure_2);

	//Inicialização TX

	GPIO_InitTypeDef GPIO_InitStructure_3;
	GPIO_InitStructure_3.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure_3.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure_3.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_3);

	//Inicialização RX

	GPIO_InitTypeDef GPIO_InitStructure_4;
	GPIO_InitStructure_4.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure_4.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure_4.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure_4);


	//Inicialização I2C
	GPIO_InitTypeDef GPIO_InitTypeDefStruct;
	GPIO_StructInit(&GPIO_InitTypeDefStruct);
	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // SCL, SDA, SA0
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);
}

void USART2_Config()
{
	//Configuração da USART
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);

   USART_Cmd(USART2, ENABLE);

   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

   NVIC_EnableIRQ(USART2_IRQn);
}

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

void send_char(char c)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, c);
}

int __io_putchar(int c)
{
	if (c=='\n')
	send_char('\r');
	send_char(c);
	return c;
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

void start_interrupt()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void read(){

	printf("Searching for accelerometer...\n");

	uint8_t who_am_i = I2C_read(MMA8452Q_ADDR, MMA8452Q_WHO_AM_I);

	if(who_am_i == 0x2a)
	{
		printf("MMA8452Q accelerometer was found\n");
		printf("Who am i value is: (0x%X)\n", who_am_i);
	}
	else
	{
		printf("Invalid device response (0x%X)\n", who_am_i);
	}

	uint8_t ctrl_reg1 = I2C_read(MMA8452Q_ADDR, MMA8452Q_CTRL_REG1);

	I2C_write(MMA8452Q_ADDR, MMA8452Q_CTRL_REG1, 0x38|0x11); // DR (0x20 = 25 Hz, 0x28 = 12,5 Hz, 0x38 = 1,56 Hz) | fast-read mode

	printf("Control_register 1 value is: (0x%X)\n", ctrl_reg1);
}

void values_I2C(){
	X = I2C_read(MMA8452Q_ADDR, MMA8452Q_OUT_X_MSB);
	Y = I2C_read(MMA8452Q_ADDR, MMA8452Q_OUT_Y_MSB);
	Z = I2C_read(MMA8452Q_ADDR, MMA8452Q_OUT_Z_MSB);
	//printf("X = %d Y = %d Z = %d\n", X, Y, Z);//print in terminal
}

void Send_USART(){
	sprintf(buffer,"|X=%d|Y=%d|Z=%d|", X, Y, Z);
	int i = 0;
	while(buffer[i] != '\0')
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		//USART_SendData(USART2, buffer[i]);
		USART_ClearFlag(USART2,USART_FLAG_TXE);
		i++;
	}
}

void TIM3_Config() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Period = 548;//auto-reload 0 ate 65535
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_Prescaler = 65535;//prescaler de 0 ate 65535
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM3, ENABLE);

    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //Pulso igual ao ARR, ou seja, quando o TIM3 bate no período,
    //o OC é capturado.
    TIM_OCInitStructure.TIM_Pulse = 548; //0 at ́e 65535
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}
void TIM4_Config() {
	//interligar o RCC ao TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//36 MHz -> PSC 36 -> 1 MHz -> Período de 1000 -> 1kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 36-1; //prescaler de 0 at´e 65535
	TIM_TimeBaseStructure.TIM_Period = 1000-1; //auto-reload 0 at´e 65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;

	//MODO PWM
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//Pulso: 1000/4 (ou seja ARR a dividir por 4 ou seja,
	//25% de duty cycle inicialmente
	TIM_OCInitStructure.TIM_Pulse = 1000/4;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	//Cada canal irá corresponder a uma das cores
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	TIM_Cmd(TIM4, ENABLE);

}
void TIM4_Setcounter(int counter) {
	counter = (counter*0.01)*1000;
	TIM_SetCompare1(TIM4,counter); //Update counter
}

//Display accelerometer values
void LCD(){
	lcd_init();
	lcd_draw_string( 10, 10, buffer, 0x07F0 , 1);
}

int begin(){
	lcd_init();
	lcd_draw_string(30,30,"MEGA",0x07F0, 3);
	lcd_draw_string(30,60,"JOGO",0x07F0 , 3);
	lcd_draw_string(30,90,"3000",0x07F0 , 3);
	lcd_draw_string(50,120,"(SW5)",0x07F0 , 1);
	start_interrupt();
}

void update_health(int health_points){
	if(health_points==3){
		lcd_draw_fillrect( 10, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 10, 10,5,5, 0x0000);

		lcd_draw_fillrect( 30, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 30, 10,5,5, 0x0000);

		lcd_draw_fillrect( 50, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 50, 10,5,5, 0x0000);

	}else if(health_points==2){
		lcd_draw_fillrect( 50, 10,10,10, 0x0000);
		lcd_draw_fillrect( 10, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 10, 10,5,5, 0x0000);

		lcd_draw_fillrect( 30, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 30, 10,5,5, 0x0000);

	}else if(health_points==1){
		lcd_draw_fillrect( 30, 10,10,10, 0x0000);
		lcd_draw_fillrect( 10, 10,10,10, 0x07F0);
		lcd_draw_fillrect( 10, 10,5,5, 0x0000);

	}else if(health_points==0){
		lcd_clear();
		flag_play = 0;
		lcd_draw_string( 30, 40, "GAME", 0x07F0 , 3);
		lcd_draw_string( 30, 70, "OVER", 0x07F0 , 3);
		lcd_draw_string( 20, 100, "Play Again(SW6)", 0x07F0 , 1);
}
}

void fill_square(int x, int y, int color){
	int x1 = x *10 + 10;
	int y1 = y * 10 + 30;
	lcd_draw_fillrect( x1, y1, 10, 10, color) ;
}

void move_square(int x, int y){
	int x1 = x *10 + 10;
	int y1 = y * 10 + 30;
	lcd_draw_fillrect( x1, y1, 10, 10, 0x0000) ;
}

int down(){
		if(pos_y >= upper_limit_y){
			return;
		}
		move_square(pos_x,pos_y);
		pos_y = pos_y+1;
		fill_square(pos_x,pos_y,0x07F0);
		//printf("x = %d\n",pos_x);
		//printf("y = %d\n",pos_y);
		return pos_y;
 }

int left(){
		if(pos_x <= lower_limit_x){
			return;
		}
		move_square(pos_x,pos_y);
		pos_x = pos_x-1;
		fill_square(pos_x,pos_y,0x07F0);
		//printf("x = %d\n",pos_x);
		//printf("y = %d\n",pos_y);
		return pos_x;
 }

int right(){
		if(pos_x >= upper_limit_x){
			return;
		}
		move_square(pos_x,pos_y);
		pos_x = pos_x+1;
		fill_square(pos_x,pos_y,0x07F0);
		//printf("x = %d\n",pos_x);
		//printf("y = %d\n",pos_y);
		return pos_x;
 }

int up(){
		if(pos_y <= lower_limit_y){
			return;
		}
		move_square(pos_x,pos_y);
		pos_y = pos_y-1;
		fill_square(pos_x,pos_y,0x07F0);
		//printf("x = %d\n",pos_x);
		//printf("y = %d\n",pos_y);
		return pos_y;
 }

void compare_pos(){
	if(pos_x == block_x && pos_y == block_y){
		new_block();
		score ++;
		update_score();
	}
}

//VER VALORES E INTERVALOS
void shift(){
	delay(1200000);
	 values_I2C();
	 if(X>210 && X<230){
		left();
		printf("\nL\n");
	}else if(X<40 && X>20){
		right();
		printf("\nR\n");
	}else if(Y<50 && Y>10){
		down();
		printf("\nD\n");
	 }else if(Y>220 && Y<240){
		up();
		printf("\nU\n");
	 }else{
		 return;
	 }
	 compare_pos();
 }

void new_block(){
	timer = 40;
	block_x = random_pos_x();
	if(block_x == pos_x){
		block_x = random_pos_x();
	}
	block_y = random_pos_y();
	if(block_y == pos_y){
		block_y = random_pos_y();
	}
	fill_square(block_x,block_y,10);
}

void check_timer(){
	timer -= 1;
	if(timer <= 0){
		health_points--;
		update_health(health_points);
		if(health_points == 0)
			return;
		move_square(block_x,block_y);
		new_block();
	}
}

void play(){
	while(flag_play == 1){
		shift();
		check_timer();
	}
}

void update_score(){
	char str[10];

	sprintf(str, "%d", score);
	lcd_draw_string( 110, 10,str, 0x0000, 1);
	lcd_draw_string( 70, 10,"score:", 0x07F0, 1);
	lcd_draw_string( 110, 10,str, 0x07F0, 1);
}

void game(){
	pos_x = random_pos_x();
	pos_y = random_pos_y();
	fill_square(pos_x, pos_y,0x07F0);
	flag_play = 1;
	new_block();
	update_score();
	health_points = 3;
	update_health(health_points);
	play();
}


int main(void)
{
	lcd_init();
	RCC_Config_HSE_PLL_Max();
	Init();
	USART2_Config();
	I2C2_Config();
	read();
	flag_game_over = 3;
	//game_over(flag_game_over);
	begin();

	for(;;){}
}
