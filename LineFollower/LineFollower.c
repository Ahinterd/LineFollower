/*
 * LineFollower.c
 *
 * Created: 08.06.2017 11:03:30
 *  Author: Andi
 */ 

/*-----------------------------------------------------------------------------------
*PIN MAPPING
*------------------------------------------------------------------------------------
	02: Ultrasonic Echo
	20: Nunchuck SDA
	21: Nunchuck SCL
	22: Ultrasonic Trigger


*/

/*------------------------------------
Timers used
--------------------------------------
TIMER0 for PWM of Motor1
TIMER2 for Ultrasonic Sensor
*/

//

#include <avr/io.h>
#include "USART.h"
#include <stdio.h>
#include "wiinunchuck.h"
#include <avr/interrupt.h>
#include <util/delay.h>


#define DEBUG_ENABLED 0		//enable or disable debug output
#define CRCCHECK_ENABLED 1	//enable or disable crc check
#define IRRefValue 100		//for comparison with IR Sensors


//Global variables for main motor
uint8_t motorSpeed = 0;
uint8_t motorDir = 0;		//1 = forward, 0 = reverse

//Global variables for steering stepper motor
int stepperPos = 0;			//Goal Positon of Stepper Motor		+ = right  /  - = left
int stepperPosCur = 0;		//Current Position of Stepper Motor

uint8_t automatic = 0;

//Global variables for nunchuck
uint8_t buttonZ;
uint8_t buttonC;
int joyX;
int joyY;
int angleX;
int angleY;
int angleZ;

//Definitions for ultrasonic sensor

#define trigger PA0
#define triggerPort PORTA
#define echo PE4
#define echoPort PORTE
#define speedOfSound 343.421

//Global variable for ultrasonic sensor

volatile int ultrasonic_rising_edge = 0;
volatile int ultrasonic_gotSignal = 0;
uint16_t ultrasonic_timerOVFValue;
uint8_t ultrasonic_error;
volatile int ultrasonic_working = 0;


//Initialize USART, may only be used for debugging
void initUSART(){
	USARTInitSTDIO(0);
	USARTInit(0, 9600, 1, 0, 1, 0);
}
//Initialize Nunchuck, uses external function for initialization
void initNunchuck(){
	wiinunchuck_init();	
}
void initUltrasonic(){
	//Init external Interrupt
	EICRB |= (1 << ISC40); //Any logical change on INT4 triggers interrupt
	EIMSK |= (1 << INT4); //Enable INT4 = PE4 = Pin2
	//configure Pins used for ultrasonic sensor
	DDRA |=  (1 << trigger);
	DDRE &=~ (1 << echo);
	ultrasonic_working = 0;
}
void initIRSensor(){
	
}
void ultrasonic_init_timer2(){
	//timer 2 is a 8 bit timer --> overflow after 255 ticks
	TCCR2B = (1<<CS20);	//no prescaling
	//timer counts with F_CPU = 16000000 Hz
	//timer overflow every 255/16000 ms
	//timer overflow after 0.0159375 ms
	//activate timeroverflow (TIMSK = Timer Interrupt Mask Register, TOIE = Timer Overflow Interrupt Enable)
	TIMSK2 = (1<<TOIE2);
	
	TCNT2 = 0;	//set timer value 0
	ultrasonic_timerOVFValue = 0;
}
void ultrasonic_stop_timer2(){
	TCCR2B = 0; //writes zero to CS20,21,22 which stops the counter
}
void nunchuck_getData(){
	
	//Update data
	wiinunchuck_update();
	//Save data for usage
	buttonC = wiinunchuck_getbuttonC();
	buttonZ = wiinunchuck_getbuttonZ();
	joyX = wiinunchuck_getjoyX();
	joyY = wiinunchuck_getjoyY();
	angleX = wiinunchuck_getangleX();
	angleY = wiinunchuck_getangleY();
	angleZ = wiinunchuck_getangleZ();
}
int ultrasonicCheckDist(){
	/*
	uint16_t distance_cm = 0;
	//Send signal
	if(ultrasonic_working == 0) //Be sure that conversation is finished
	{
		_delay_ms(50);		//Restart HC-SR04
		triggerPort &=~ (1 << trigger);
		_delay_us(1);
		triggerPort |= (1 << trigger); //Send 10us second pulse
		_delay_us(10);
		triggerPort &=~ (1 << trigger);
		ultrasonic_working = 1;	//To make sure that it is ready
		ultrasonic_error = 0;		//Clean errors
	}//end of sending signal
	
	while(ultrasonic_working){}
					
	if(ultrasonic_error)
	{
		printf("\nerror\n");
	}else if(ultrasonic_gotSignal){	//calculate distance
		float distance_ms;
		printf("%i\n", ultrasonic_timerOVFValue);
		distance_ms = ultrasonic_timerOVFValue* 0.0159375; // * 255 / 16000000 * 1000 = 0.0159
		uint16_t distance_cm = distance_ms * speedOfSound / 10 / 2;// / *100 in cm /1000 in m --> /10, /2 da der Schall die Strecke 2mal zurücklegt
		printf("Errechnete Distanz: %i cm\n",distance_cm);
		ultrasonic_gotSignal = 0;	//reset
	}
	return distance_cm;
	*/
	return 21;
}
int IRSensorLeft(){
	return 0;
}
int IRSensorRight(){
	return 0;
}
int IRSensorMid(){
	return 0;
}
//Service Routine for Timer 2 (used for ultrasonic sensor)
ISR (TIMER2_OVF_vect)
{	
	ultrasonic_timerOVFValue++;
	//check if Object out of range
	if(ultrasonic_timerOVFValue>1460){
		ultrasonic_error = 1;
	}
}
//Service Routine for Interrupt 4 which is used for ultrasonic sensor (echo)
ISR (INT4_vect)
{
	printf("interrupt4");
	if(ultrasonic_working==1)			//check if signal was sent before echo was incoming
	{
		if(ultrasonic_rising_edge==0)	//Check if echo is high
		{
			ultrasonic_init_timer2();	//start timer
			ultrasonic_rising_edge=1;	
		}
		else //Check if echo turned low, calculate distance
		{
			ultrasonic_stop_timer2();	//stop timer
			ultrasonic_rising_edge = 0;			//reset values
			ultrasonic_working = 0;
			ultrasonic_gotSignal = 1;				//confirm success
		}
	}
}
int main(void)
{
	
	sei();
	initUSART();
	printf("Starting\n");
	initNunchuck();
	initUltrasonic();
	initIRSensor();
	printf("Initialization complete!\n");
	
    while(1)
    {
		if(automatic == 0)												//Check if Automatic Mode not enabled
		{
			nunchuck_getData();			
			if(buttonC)													//When C pressed toggle Automatic Mode
			{
				automatic = 1;
			}
			
		}else  															//In Automatic Mode
		{
			printf("Auto mode!\n\n");
										
			if(ultrasonicCheckDist()<20)								//Check Distance to obstacle
			{															//In Automatic Mode, obstacle in range
				printf("Obstacle in Range\n");			
			
			}else                       								//In Automatic Mode, no obstacle in range
			{	
				printf("NO OBSTACLE\n");														
				if(IRSensorLeft()<IRRefValue)
				{
					
				}else if(IRSensorRight()<IRRefValue)
				{
					
				}else if(IRSensorMid()<IRRefValue)
				{
					
					}else{
					
				}
			}	
		
		}
	}
}