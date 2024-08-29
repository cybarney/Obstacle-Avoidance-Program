/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    CSE325_MoorseCode.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include <ctype.h>
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int current_state = 0;
volatile unsigned int time_now_seconds = 0;
volatile unsigned int time_now_ms = 0;
volatile unsigned int time_now_us = 0;
volatile unsigned int t_start = 0;
volatile unsigned int t_end = 0;
volatile unsigned int t_total = 0;
volatile unsigned int count = 0;
volatile unsigned int distance = 0;
int trigger_flag = 0;
int left_num = 0;
int right_num = 0;

void left(void){
	TPM1->CONTROLS[0].CnV = 1; //Left
}

void straight(void){
	TPM1->CONTROLS[0].CnV = 12000; //Straight
}

void right(void){
	TPM1->CONTROLS[0].CnV = 1100; //Right
}

void turn_right(){
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<2);
	for(int i=0;i<2500000;i++){
        __asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void turn_left(){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR |= (1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);
	for(int i=0;i<2500000;i++){
        __asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void stop(void){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
	left(); //Left
    printf("Going left");
	for(int i=0;i!=1000000*5;i++){
        __asm volatile ("nop");
	   }
    right();
    for(int i=0;i!=1000000*5;i++){
        __asm volatile ("nop");
	   }
    if(right_num >= left_num){
    	turn_right();
    }else{
    	turn_left();
    }
}

void echo(){
//    	t_start |= (TPM0->CNT);
	//printf("ECHO\n");
	count = 0;
	while(!(GPIOA->PDIR & (1<<13))){

	}
	t_start = TPM0->CNT;
	while(GPIOA->PDIR & (1<<13)){
        __asm volatile ("nop");
	}
	t_end = TPM0->CNT;
	t_total = (t_end - t_start) + (count*65535);
	distance = t_total/58;
	if(t_total<=40){
		stop();
	}else{
		TPM2->CONTROLS[0].CnV = 7000;
		TPM2->CONTROLS[1].CnV = 7000;
		GPIOB->PDOR |= (1<<0);
		GPIOB->PDOR &= ~(1<<1);
		GPIOC->PDOR |= (1<<1);
		GPIOC->PDOR &= ~(1<<2);

	}
	printf("%d\n", t_total);
}

void trigger(void){
	GPIOD->PDOR |= (1<<2);
	for(int i = 0; i<480;i++){
       __asm volatile ("nop");
	}
	//delay_us(10);
	GPIOD->PCOR |= (1<<2);

	echo();
}




void PIT_IRQHandler(void){
	if(PIT->CHANNEL[0].TFLG){ // Timer 0 Triggered
		PIT->CHANNEL[0].TFLG = 1; // Reset
		time_now_seconds+=1;
		//printf("UPDATED TIME_NOW_SECONDS\n");
//		if(current_state == 0){
//			left();
//			current_state+=1;
//		}else if(current_state == 1){
//			straight();
//			current_state+=1;
//		}else{
//			right();
//			current_state = 0;
//		}
	}
	if(PIT->CHANNEL[1].TFLG){
		//printf("%d\n",time_now_us);
		PIT->CHANNEL[1].TFLG = 1;
		trigger();
	}
}


void init_pits() {
	SIM->SCGC6 |= (1<<23);
	PIT->MCR = 0x00;
	//PIT->CHANNEL[0].LDVAL = 24000000; // (1 seconds)
	PIT->CHANNEL[0].LDVAL = 24000000; // (1 seconds)
	//PIT->CHANNEL[1].LDVAL = 24; // microsecond
	PIT->CHANNEL[1].LDVAL = 1440000; // 60ms
	//PIT->CHANNEL[1].LDVAL = 14400000; // 600ms

	NVIC_EnableIRQ(22);
	PIT->CHANNEL[0].TCTRL = 0x3; // enable Timer 0 interrupts and start timer.
	PIT->CHANNEL[1].TCTRL = 0x3; // Enable Timer 1 Interrupts and start timer
}

void TPM0_IRQHandler(void){
	TPM0->SC |= (1 << 7); // Reset Timer Interrupt Flag
	count++; // What happens on overflow?
}

void timer(){
	SIM->SCGC6 |= (1<<24);//Enable Clock for TPM0
	SIM->SOPT2 |= (0x2 << 24); //Set TPMSRC to OSCERCLK
	TPM0->CONF |= (0x1 << 17); //Stop on Overflow
	TPM0->SC = (1<<7) | (0x07) | (1<<6) | (1<<3);//Reset Timer Overflow Flag, set prescaler 128
	//TPM0->SC = (1<<7) | (0x00) | (1<<6) | (1<<3);//Reset Timer Overflow Flag, set prescaler 128
	//TPM0->MOD = 2810;
	TPM0->MOD = 65535;
	NVIC_EnableIRQ(17);
	TPM0->SC |= 0x01 << 3; //Starts the clock
}


void PORTC_PORTD_IRQHandler(void) { // Location defined in startup/startup_MKL46Z4.c
	PORTC->PCR[3] |= (1 << 24); // Clear Interrupt Flag!
	time_now_seconds = 0;
//	printf("Time_now_seconds: %d\n", time_now_seconds);
//	for(int i=0;i!=96000000;i++){
//		__asm volatile ("nop");
//	}

	//	while(time_now_seconds <=2){
//
//	}
	TPM2->CONTROLS[0].CnV = 7000;
	TPM2->CONTROLS[1].CnV = 7000;
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);

}

void setup_PORTD_interrupt() {
	//printf("SETUP ORTD\n");
	SIM->SCGC5 |= (1<<11);  // Enable Port C Clock
	PORTC->PCR[3] &= ~0xF0703; // Clear First
	PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3 ); // Set MUX bits, enable pullups, interrupt on falling edge
	GPIOC->PDDR &= ~(1 << 3); // Setup Pin 3 Port C as input

    NVIC_SetPriority(31, 1);

	// Call Core API to Enable IRQ
	NVIC_EnableIRQ(31);
}


int main(void) {

    /* Init board hardware. */
    //hardware_init();
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
	init_pits();
	setup_PORTD_interrupt();
	timer();
	//init_echo();
    SIM->SCGC5 |= (1<<12) | (1<<13) | (1<<11) | (1<<10) | (1<<9);

    //SW1
//    PORTC->PCR[3] &= ~0x703;
//    PORTC->PCR[3] |= 0x703 & ((1<<8) | 0x03); //Set MUX bits, enable pullups.

    //PTA13
	PORTA->PCR[13] &= ~0x700;
	PORTA->PCR[13] |= 0x700 & (1<<8);
	GPIOA->PDDR &= ~(1<<13);

	//PTD2(Trigger)
	SIM->SCGC5 |= (1<<12);
	PORTD->PCR[2] &= ~0x700;
    PORTD->PCR[2] |= 0x700 & (1<<8);
	GPIOD->PDDR |= (1<<2);

	//Motor stuff
	 //Left Motor Setup
	PORTB->PCR[0] &= ~0x700; //In1
	PORTB->PCR[0] |= 0x700 & (1<<8);

	PORTB->PCR[1] &= ~0x700;//In2
	PORTB->PCR[1] |= 0x700 & (1<<8);

	//Right Motor Setup
	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1<<8);

	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[2] |= 0x700 & (1<<8);

	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x300;//Drive as PWM

	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x300;//Drive as PWM

	GPIOB->PDDR |= (1<<0) | (1<<1);
	GPIOC->PDDR |= (1<<1)| (1<<2);
	GPIOC->PDDR &= ~(1<<3);
	GPIOC->PDDR &= ~(1<<12);

	//Setup for PWM on PTB2 (PWMA) Left Motor
	SIM->SCGC6 |= (1<<26);
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	//Setup for PWM on PTB3 (PWMB) Right Motor
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	TPM2->MOD = 7999;
	TPM2->CONTROLS[0].CnV = 1;
	TPM2->CONTROLS[1].CnV = 1;

	TPM2->SC |= 0x01 << 3;




    SIM->SCGC5 |= (1<<9); //Enable clock gating
	PORTA->PCR[12] &= ~0x700;
    PORTA->PCR[12] |= 0x300;
    GPIOA->PDDR &= ~(1<<12);

	SIM->SCGC6 |= (1 << 25); // Clock Enable TPM1
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Edge PWM
	TPM1->MOD = 99999;
	TPM1->SC |= 0x01 << 3; // Start the clock!
    TPM1->CONTROLS[0].CnV = 1;
    straight();
    int flag_straight = 0;
    while(1) {
    	if(flag_straight == 0) {
    		straight();
    		}
    }


    return 0 ;
    }

