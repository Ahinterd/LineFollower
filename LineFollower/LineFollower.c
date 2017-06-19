/*
 * LineFollower.c
 *
 * Created: 08.06.2017 11:03:30
 *  Author: Andreas Hinterdorfer, Severin Bergsmann
 
 */ 

/*-----------------------------------------------------------------------------------
*PIN MAPPING
*------------------------------------------------------------------------------------
	A0: IRSensor0
	A1: IRSensor1
	A2: IRSensor2	
	02: Ultrasonic Echo
	13: Motor Analog
	20: Nunchuck SDA
	21: Nunchuck SCL
	22: Ultrasonic Trigger
	23: Stepper In1
	24: Motor IN1
	25: Stepper In2
	26: Motor IN2
	27: Stepper In3
	29: Stepper In4
	


*/

/*------------------------------------
Timers used
--------------------------------------
TIMER0 for PWM of Motor1
TIMER2 for Ultrasonic Sensor
*/

/*

Working atm:
	Nunchuck
	Manual Mode(Steering+Acceleration)
	IRSensor
*/
/*Todo
	Ultrasonic Sensor
	Code for Automatic Mode
	Max Wheel Postion for Steering
*/

#include <avr/io.h>
#include "USART.h"
#include <stdio.h>
#include "wiinunchuck.h"
#include <avr/interrupt.h>
#include <util/delay.h>


#define DEBUG_ENABLED 0		//enable or disable debug output
#define CRCCHECK_ENABLED 1	//enable or disable crc check



//Global variables for main motor
uint8_t motorSpeed = 0;
uint8_t motorDir = 0;		//1 = forward, 0 = reverse

//Definitions for main motor
#define motorIn1 PA2
#define motorIn2 PA4
#define motorSpeedSlow 128
#define motorSpeedFast 255

//Global variables for nunchuck
uint8_t buttonZ;
uint8_t buttonC;
int joyX;
int joyY;
int angleX;
int angleY;
int angleZ;
uint8_t automatic = 0;

//Definitions for nunchuck
#define joyYMax 103

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

//Definitions for stepper motor
#define stepA PA1
#define stepB PA3
#define stepC PA5
#define stepD PA7
#define stepDelay 1600

//Global variables for steering stepper motor
int stepperPos = 0;			//Goal Positon of Stepper Motor		+ = right  /  - = left
int stepperPosCur = 0;		//Current Position of Stepper Motor


//Definitions for IRSensors
#define IRRefValue 100		//for comparison with IR Sensors
#define IRLeft 0
#define IRRight 1
#define IRMid 2


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
	//Referenzspannung wählen
	ADMUX = (1<<REFS0) | (1<<REFS1);	//2.56V
	//Standard single conversion
	//ADFR=1 damit dauerbetrieb
	
	//Vorteiler wählen, sodass Frequenz zwischen 50kHz-200kHz --> 128 --> 125kHz
	ADCSRA = (1<<ADPS0)| (1<<ADPS1) | (1<<ADPS2); //AnalogDigitalPreScalser = 128
	
	//ADC aktivieren
	ADCSRA |= (1<<ADEN);
	
	ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung 
	while (ADCSRA & (1<<ADSC) ) {}         // auf Abschluss der Konvertierung warten
	
	/* ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten
     Wandlung nicht übernommen. */
	(void) ADCW;
}

void initMotor(){
	// initialize timer0 in PWM mode
	TCCR0A |= (1<<WGM00)|(1<<COM0A1);
	TCCR0B |= (1<<CS00);
	
	// make sure to make OC0 pin  as output pin
	DDRB |= (1<<PB7);
	
	//set motorIn as output
	DDRA |= (1<<motorIn1)|(1<<motorIn2);
	
	OCR0A = 0; //duty cycle... 0 = 0% = 0V, 255 = 100% = 5V
}
void initStepper(){
	DDRA |= (1<<stepA)|(1<<stepB)|(1<<stepC)|(1<<stepD);	//set Stepper pins as output
}
void step1(){
	PORTA |= (1<<stepB)|(1<<stepC);		//set pins high
	PORTA &=~((1<<stepA)|(1<<stepD));	//set pins low
	_delay_us(2500);
}
void step2(){
	PORTA |= (1<<stepB)|(1<<stepD);
	PORTA &=~((1<<stepA)|(1<<stepC));
	_delay_us(2500);
}
void step3(){
	PORTA |= (1<<stepA)|(1<<stepD);
	PORTA &=~((1<<stepB)|(1<<stepC));
	_delay_us(2500);
}
void step4(){
	PORTA |= (1<<stepA)|(1<<stepC);
	PORTA &=~((1<<stepB)|(1<<stepD));
	_delay_us(2500);
}
void stepLeft(int cycles){
	printf("Step Left\n");
	for(int i = 0; i<cycles; i++){							//for debugging
		step4();
		step3();
		step2();
		step1();
	}
	PORTA &=~((1<<stepA)|(1<<stepB)|(1<<stepC)|(1<<stepD));
}
void stepRight(int cycles){
	printf("Step Right\n");
	for(int i = 0; i<cycles; i++){							//for debugging
		step1();
		step2();
		step3();
		step4();
	}
	PORTA &=~((1<<stepA)|(1<<stepB)|(1<<stepC)|(1<<stepD));
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
void nunchuckCheckY(){
	if(joyY>=5){												//Joystick Y in positive position
		OCR0A = joyY*255/joyYMax;								//set duty cycle relative to joystick position
		PORTA |= (1<<motorIn1);									//set In1 1 and In2 0 --> Motor turns forward
		PORTA &=~ (1 << motorIn2);
		printf("%d\n", OCR0A);
		}else if(joyY<=-5){										//Joystick Y in negative position
		OCR0A = joyY*(-1)*255/joyYMax*3/4;								//set duty cycle relative to joystick position and only drive at 3/4 speed
		PORTA |= (1<<motorIn2);									//set In2 1 and In1 0 --> Motor turns backwards
		PORTA &=~ (1 << motorIn1);
		printf("%d\n", OCR0A);
		}else{													//Joystick Y in 0 position
		OCR0A = 0;												//set all three values 0 --> Motor stops
		PORTA &=~ (1 << motorIn1);
		PORTA &=~ (1 << motorIn2);
		printf("Stop\n");
	}
}
void nunchuckCheckX(){
	if(joyX>10){												//Joystick position right
		stepRight(1);
	}else if(joyX<-10){											//Joystick postion left
		stepLeft(1);
	}else{														//Joystick X in 0 position
		
	}
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
uint16_t ADC_Read( uint8_t channel )
{
	// Kanal waehlen, ohne andere Bits zu beeinflußen
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
	while (ADCSRA & (1<<ADSC) ) {   // auf Abschluss der Konvertierung warten
	}
	return ADCW;                    // ADC auslesen und zurückgeben
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
	initMotor();
	initStepper();
	printf("Initialization complete!\n");
	
	

    while(1)
    {
		if(automatic == 0)												//Check if Automatic Mode not enabled
		{

			//printf("\nZ: %i\nC: %i\nX: %i\nY: %i\n", buttonZ, buttonC, joyX, joyY);
			//_delay_ms(500);
			

			nunchuck_getData();			
			if(buttonC)													//When C pressed toggle Automatic Mode
			{
				automatic = 1;
				_delay_ms(500);
			}
			nunchuckCheckY();
			nunchuckCheckX();
			
			
		}else  															//In Automatic Mode
		{
			printf("Auto mode!\n\n");
			if(buttonC){
				automatic = 0;
				_delay_ms(500);
			}
										
			if(ultrasonicCheckDist()<20)								//Check Distance to obstacle
			{															//In Automatic Mode, obstacle in range
				printf("Obstacle in Range\n");			
			
			}else                       								//In Automatic Mode, no obstacle in range
			{	
				printf("NO OBSTACLE\n");														
				if(ADC_Read(IRLeft)<IRRefValue)							//IR Left returns black
				{
					stepLeft(1);										//steer left
					OCR0A = motorSpeedSlow;								//set speed slow by changing duty cycle of PWM
				}else if(ADC_Read(IRRight)<IRRefValue)					//IR Right returns black
					{
						stepRight(1);									//steer right
						OCR0A = motorSpeedSlow;							//set speed slow by changing duty cycle of PWM
					}else if(ADC_Read(IRMid)<IRRefValue)				//IR Mid returns black
						{
							OCR0A = motorSpeedFast;						//set speed fast by changing duty cycle of PWM
					}else          										//No sensor returned black--error condition
					{
						OCR0A = 0;
				}
			}			
		}
	}
}