/*
? * Team Id: <eYRC#1221>
? * Author List: Shubhankar Handa,
? * (Comma separated eg. Name1, Name2)>
? * Filename: <Filename>
? * Theme: <Theme name -- Specific to eYRC>
? * Functions: <Comma separated list of Functions defined in this file>
? * Global Variables: <List of global variables defined in this file, None if no global
? * variables>
? */

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
unsigned char ADC_Conversion(unsigned char);

///////*******************you cannot initiallize a float here put value of float in the code below aswell ********************
float ls=0,rs=0,ms=0;
float LMS=0,RMS=0;
float err=0,perr=0;
int var=0;
unsigned int p,d;
int flag2=1;
 int flag=0,flag1=0,flag3=0,flag4=0;
 int match=0;
 
 char done[30]={'R','R','l','L','P','S','S','R','S','R','S','S','S','R','S','R','S','S','l','S','S','L','S','L','S','p'};
//Function to configure LCD port
//ADC pin configuration

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned int PostionLeft=10000,PostionRight=10000;
//Function to configure ports to enable robot's motion



//lcd ................


void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//lcd ..............


//adc >>>>>>>>>>>>>>>>>>>>>>
void adc_pin_config (void)
{
  DDRF = 0x00; //set PORTF direction as input
  PORTF = 0x00; //set PORTF pins floating
  DDRK = 0x00; //set PORTK direction as input
  PORTK = 0x00; //set PORTK pins floating
}

//adc>>>>>>>>>>>>>>>>>>>>>>>>>

//uart>>>>>>>>>>>>>>>>>>>>>>>
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}
void uart_transmit (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}
unsigned char uart_recieve (void)
{
	while(!(UCSR0A) & (1<<RXC0));                   // wait while data is being received
	return UDR0;                             // return 8-bit data
}

//uart >>>>>>>>>>>>>>>>>>>>>>>>

//pwm>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void timer5_init()
{
  TCCR5B = 0x00;  //Stop
  TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;  //Output compare register high value for Left Motor
  OCR5AL = 0xFF;  //Output compare register low value for Left Motor
  OCR5BH = 0x00;  //Output compare register high value for Right Motor
  OCR5BL = 0xFF;  //Output compare register low value for Right Motor
  OCR5CH = 0x00;  //Output compare register high value for Motor C1
  OCR5CL = 0xFF;  //Output compare register low value for Motor C1
  TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
            For Overriding normal port functionality to OCRnA outputs.
              {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
  
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}
//pwm>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Function to Initialize PORTS

//Function to Initialize ADC
void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
  unsigned char a;
  if(Ch>7)
  {
    ADCSRB = 0x08;
  }
  Ch = Ch & 0x07;
  ADMUX= 0xE0| Ch;
  ADCSRA = ADCSRA | 0x40;   //Set start conversion bit
  while((ADCSRA&0x10)==0);  //Wait for ADC conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}
//servo************

void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}
//servo end***************

//encoder ********************************


void encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF0;  //Set the direction of the PORTE 4 pin as input
 PORTD = PORTD | 0x0F; //Enable internal pull-up for PORTE 4 pin *****INTERESTING*****
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void port_init()
{
	adc_pin_config();
	encoder_pin_config();
		servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
		servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
}


//Function to initialize ports

void encoder_interrupt_init (void) //Interrupt 4 enable
{
 //cli(); //Clears the global interrupt
 EICRA = EICRA | 0xAA; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x0F; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

//ISR for right position encoder

ISR(INT3_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
 /*if(ShaftCountRight>ShaftCountLeft)
 {
  Postion++;
 }*/
  if(PIND &(1<<PD2))
 {
  PostionLeft--;
 }  
}
//ISR for left position encoder
ISR(INT2_vect)
{
 ShaftCountLeft++;  //increment left shaft position count

  /*if(ShaftCountRight<ShaftCountLeft)
 {
  Postion--;
 }*/
  if(PIND &(1<<PD3))
 {
  PostionLeft++;
 } 
}
ISR(INT1_vect)  
{

 ShaftCountRight++;  //increment right shaft position count
 /*if(ShaftCountRight>ShaftCountLeft)
 {
  Postion++;
 }*/
  if(PIND &(1<<PD0))
 {
  PostionRight++;
 }  
}

//ISR for left position encoder
ISR(INT0_vect)
{

 ShaftCountLeft++;  //increment left shaft position count

  /*if(ShaftCountRight<ShaftCountLeft)
 {
  Postion--;
 }*/
  if(PIND &(1<<PD1))
 {
  PostionRight--;
 } 
 
}
void init_devices()
{
 cli(); //Clears the global interrupt
 	timer1_init();
 lcd_port_config();
 port_init();  //Initializes all the ports
 encoder_interrupt_init();
   	lcd_set_4bit();
   	lcd_init();
	     port_init();
	     adc_init();
 sei();   // Enables the global interrupt 
}





//encoder end ****************************

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.



void sensorvalue()
{
    ls = ADC_Conversion(1); /// was ls
    ms = ADC_Conversion(2);
    rs = ADC_Conversion(3); // was rs 
}

void errorgen()
{	if((ls<50)&&(ms<50)&&(rs<50))
	{err=perr;}
	else
	{err=(ls-rs);}		//    err=(ls-rs);
	LMS=250-err*p-(err-perr)*d;
	RMS=250+err*p+(err-perr)*d;
}
void nodeavoid()
{
	PORTB=0b00000101;
	OCR5AL=200;
	OCR5BL=200;
	_delay_ms(150); //just to avoid node then follow line to midpoint of the wheels
}


void straight()
{
	PORTB=0b00000101;
	OCR5AL=250;
	OCR5BL=250; //just to avoid node then follow line to midpoint of the wheels
}
void right()
{
	PORTB=0b00000110;
		OCR5AL=200;
		OCR5BL=200;
		_delay_ms(300);
		sensorvalue();
		while(!((ls<50)&&(ms>100)&&(rs<50))) //		while(!((ls<50)&&(ms>100)&&(rs<50)))
		{sensorvalue();
			OCR5AL=200;
			OCR5BL=200;
		} //just to avoid node then follow line to midpoint of the wheels
		 	PORTB=0b00001001;//if error then this was forward
		 	OCR5AL=200;      //changed to impulse of left
		 	OCR5BL=200;
		 	_delay_ms(50);
}
void rightbl()
{
	PORTB=0b00000110;
	OCR5AL=200;
	OCR5BL=200;
	_delay_ms(300);
	sensorvalue();
	while(!((ls>50)&&(ms<100)&&(rs>50))) //		while(!((ls<50)&&(ms>100)&&(rs<50)))
	{sensorvalue();
		OCR5AL=200;
		OCR5BL=200;
	} //just to avoid node then follow line to midpoint of the wheels
	PORTB=0b00001001;//if error then this was forward
	OCR5AL=200;      //changed to impulse of left
	OCR5BL=200;
	_delay_ms(50);
}
void left()
{
		PORTB=0b00001001;
		OCR5AL=200;
		OCR5BL=200;
		_delay_ms(300);
		sensorvalue();
		while(!((ls<50)&&(ms>100)&&(rs<50)))	//		while(!((ls<50)&&(ms>100)&&(rs<50)))
		{sensorvalue();
					OCR5AL=200;
					OCR5BL=200;
		}
		 //just to avoid node then follow line to midpoint of the wheels
		 	PORTB=0b00000110;//if error then this was forward
			 		OCR5AL=200;
			 		OCR5BL=200;
					 _delay_ms(50);
			 
}
void leftbl()
{
	PORTB=0b00001001;
	OCR5AL=200;
	OCR5BL=200;
	_delay_ms(300);
	sensorvalue();
	while(!((ls>50)&&(ms<100)&&(rs>50)))	//		while(!((ls<50)&&(ms>100)&&(rs<50)))
	{sensorvalue();
		OCR5AL=200;
		OCR5BL=200;
	}
	//just to avoid node then follow line to midpoint of the wheels
	PORTB=0b00000110;//if error then this was forward
	OCR5AL=200;
	OCR5BL=200;
	_delay_ms(50);
	
}
void linefollowing()
{
		PORTB=0b00000101;
    sensorvalue();
    errorgen();
if(LMS>255)
LMS=255;
else if(RMS>255)
RMS=255;
if(LMS<0)
LMS=0;
else if(RMS<0)
RMS=0;

	OCR5AL=RMS;
	OCR5BL=LMS;
	perr=err;
}
void linefollowingbl()
{
			PORTB=0b00000101;
    if((ls>50)&&(ms<150)&&(rs>50))
    {
	    	OCR5AL=250;
	    	OCR5BL=250;
    }
	    if((ls<150)&&(ms>50)&&(rs>50))
	    {
		    	OCR5AL=250;
		    	OCR5BL=120;
	    }
			    if((ls>50)&&(ms>50)&&(rs<150))
			    {
				    	OCR5AL=120;
				    	OCR5BL=250;
			    }

}
void revlinefollowing()//changes
{
	PORTB=0b00001010;
	sensorvalue();
	errorgen();
	if(LMS>255)
	LMS=255;
	else if(RMS>255)
	RMS=255;
	if(LMS<0)
	LMS=0;
	else if(RMS<0)
	RMS=0;

	OCR5AL=LMS;
	OCR5BL=RMS;
	perr=err;
}
void movestraight(unsigned int v)
{		linefollowing();
	PostionLeft=1100;
	PostionRight=1100;
		//PORTB=0b00000101;
	unsigned int temp_pos_left,temp_pos_right;
	temp_pos_left=1000;
	temp_pos_right=1000;
	linefollowing();
	while(((PostionLeft-temp_pos_left)<=v)&&((PostionRight-temp_pos_right)<=v))
	{
		linefollowing();
		_delay_ms(1);
					//lcd_print(2,2,(PostionLeft-temp_pos_left),5);//1000
					//lcd_print(1,2,(PostionRight-temp_pos_right),5);//990
	}
	PostionLeft=1000;
	PostionRight=1000;//*************************error due to saturation of int variable due to many encoder values
	
			PORTB=0b00001010;
				OCR5AL=RMS;
				OCR5BL=LMS;
				_delay_ms(30);
		//try using the force stop condition 00001111 and ocr 200
}

void movest(unsigned int v)
{		linefollowing();
	PostionLeft=1100;
	PostionRight=1100;
		//PORTB=0b00000101;
	unsigned int temp_pos_left,temp_pos_right;
	temp_pos_left=1000;
	temp_pos_right=1000;
	linefollowing();
	while(((PostionLeft-temp_pos_left)<=v)&&((PostionRight-temp_pos_right)<=v))
	{
		linefollowing();
		_delay_ms(1);
					//lcd_print(2,2,(PostionLeft-temp_pos_left),5);//1000
					//lcd_print(1,2,(PostionRight-temp_pos_right),5);//990
	}
	PostionLeft=1000;
	PostionRight=1000;//*************************error due to saturation of int variable due to many encoder values

		//try using the force stop condition 00001111 and ocr 200
}

void moveback(unsigned int v)
{		revlinefollowing();
	PostionLeft=1000;
	PostionRight=1000;
		//PORTB=0b00000101;
	unsigned int temp_pos_left,temp_pos_right;
	temp_pos_left=1100;
	temp_pos_right=1100;
	linefollowing();
	while(((-PostionLeft+temp_pos_left)<=v)&&((-PostionRight+temp_pos_right)<=v))
	{
		revlinefollowing();
		_delay_ms(2);
					//lcd_print(2,2,(PostionLeft-temp_pos_left),5);//1000
					//lcd_print(1,2,(PostionRight-temp_pos_right),5);//990
	}
	PostionLeft=1000;
	PostionRight=1000;//*************************error due to saturation of int variable due to many encoder values
	
		//try using the force stop condition 00001111 and ocr 200
}

void turnright(unsigned int v)
{
	PORTB=0b00000110;
	unsigned int temp_pos_left,temp_pos_right;
	temp_pos_left=PostionLeft;
	temp_pos_right=PostionRight;
	
	while(((PostionLeft-temp_pos_left)<=v)&&((PostionRight-temp_pos_right)<=v))
	{
	PORTB=0b00000110;
	}
	PostionLeft=10000;
	PostionRight=10000;//*************************error due to saturation of int variable due to many encoder values
	PORTB=0b00000101;
}

void turnleft(unsigned int b)
{
		PostionLeft=10000;
		PostionRight=10000;
	OCR5AL=200;
	OCR5BL=200;
	PORTB=0b00001111;
	_delay_ms(50);
	unsigned int temp_pos_left,temp_pos_right;
	temp_pos_left=10300;
	temp_pos_right=9700; //changed to 10000 istested of 1000

	while(((temp_pos_left-PostionLeft)<=b)&&((PostionRight-temp_pos_right)<=b))//as unsigned hence -ve no is 65578
    {	
		PORTB=0b00001001;
			OCR5AL=200;
			OCR5BL=200;
			lcd_print(2,2,PostionLeft,5);//1000 990 
						lcd_print(1,2,PostionRight,5);//990 1030
	}
	PostionLeft=1000;
	PostionRight=1000;//*************************error due to saturation of int variable due to many encoder values
	PORTB=0b00001111;
		OCR5AL=250;
		OCR5BL=250;
		_delay_ms(50);
}
void flaggen()
{		sensorvalue();
		if(((ls<50)&&(ms>100)&&(rs<50))||((ls>50)&&(ms<50)&&(rs<50))||((ls<50)&&(ms<50)&&(rs>50)))
		{flag2=1;}
		else if(((ls>50)&&(ms<100)&&(rs>50))||((ls<100)&&(ms>50)&&(rs>50))||((ls>50)&&(ms>50)&&(rs<100)))
		{flag2=0;}
		else if(((ls<50)&&(ms<50)&&(rs<50)))
		{flag2=2;}
		else if(((ls>50)&&(ms>50)&&(rs>50)))
		{flag2=5;}
		else
		{flag2=1;}
		
		if(((ls>50)&&(ms>50))||((ms>50)&&(rs>50)))
		{
			flag4=1;
		}
		if(((ls<50)&&(ms<50))||((ms<50)&&(rs<50)))
		{
			flag4=0;
		}
}

void check()
{
	flaggen();
	if(flag2==5||flag2==0)
	{match=1;}
	else
	{match=0;}
	
}
void checkbl()
{
	flaggen();
	if(flag2==2||flag2==1)
	{match=1;}
	else
	{match=0;}
	
}
void turn()
{	lcd_wr_char('a');
		if (done[var]=='L')
		{			
			movestraight(190);//change from 220 to 240
			check();
			if(match==0){
			left();
			var++;
			}
			else 
			{match=0;
			flag3=2;}
		}
		else if (done[var]=='R')
		{		
			movestraight(190);
			check();
			if(match==0){
			right();
			var++;
			}
			else
			{match=0;
			flag3=2;}
		}
		else if (done[var]=='S')
		{					
			movest(190);
			check();
			if(match==0){
			var++;
			}
			else
			{match=0;
			flag3=2;}

		}
		else if (done[var]=='l')
		{
					movestraight(160);
						check();
						if(match==0){
						servo_2(90);
						servo_1(90);
						PORTB=0x00;
						//_delay_ms(3000);
						//turnleft(800);
						left();
						//movestraight(150);//changed to 150
							PORTB=0b00001111;
							OCR5AL=250;
							OCR5BL=250;
							_delay_ms(50);

						
						PORTB=0b00000000;
						servo_2(25);
						_delay_ms(600);
										
						servo_1(155);
						_delay_ms(1000);
										
						servo_2(90);
						_delay_ms(600);
							
						moveback(150);			
								//_delay_ms(1000);	
			var++;
			if(done[var]=='R')
			{left();
						var++;}
			if(done[var]=='L')
			{right();
						var++;}
			}
			else
			{match=0;
			flag3=2;}
			
		}
		else if (done[var]=='r')
		{
					movestraight(160);
					check();
					if(match==0){
					servo_2(90);
					servo_1(90);
					PORTB=0x00;
					//_delay_ms(3000);
					//turnleft(800);
					right();
					//movestraight(150);//changed to 150
					PORTB=0b00001111;
					OCR5AL=250;
					OCR5BL=250;
					_delay_ms(50);

					
					PORTB=0b00000000;
					servo_2(25);
					_delay_ms(600);
					
					servo_1(155);
					_delay_ms(1000);
					
					servo_2(90);
					_delay_ms(600);
					
					moveback(150);
					//_delay_ms(1000);
					var++;
			if(done[var]=='R')
			{left();
						var++;}
			if(done[var]=='L')
			{right();
						var++;}
			}
			else
			{match=0;
			flag3=2;}
		}
		else if (done[var]=='P')
				{
								movestraight(160);
					right();
					//movestraight(150);
												PORTB=0b00001111;
												OCR5AL=250;
												OCR5BL=250;
												_delay_ms(50);
					servo_2(80);
					_delay_ms(100);
					
					servo_1(10);
					_delay_ms(100);
					
					servo_2(90);
					_delay_ms(100);
					
					//movestraight(230);
					//_delay_ms(1000);
					//moveback(150); //done to 50
					var++;
			if(done[var]=='R')
			{left();
			var++;}
			if(done[var]=='L')
			{right();
			var++;}
				}
		else if (done[var]=='p')
				{
								movestraight(160);
								left();
								//movestraight(150);
								PORTB=0b00001111;
								OCR5AL=250;
								OCR5BL=250;
								_delay_ms(50);
								servo_2(80);
								_delay_ms(100);
								
								servo_1(10);
								_delay_ms(100);
								
								servo_2(90);
								_delay_ms(100);
								
								//movestraight(230);
								//_delay_ms(1000);
								//moveback(150); //done to 50
								var++;
								if(done[var]=='R')
								{left();
								var++;}
								if(done[var]=='L')
								{right();
								var++;}
				}
						lcd_wr_char('1');
					
}
void turnbl()
{	lcd_wr_char('b');
		if (done[var]=='L')
		{			
			movestraight(190);//change from 220 to 240
			checkbl();
			if(match==0){
			leftbl();
			var++;
			}
			else 
			{match=0;
			flag3=0;}
		}
		else if (done[var]=='R')
		{		
			movestraight(190);
			checkbl();
			if(match==0){
			rightbl();
			var++;
			}
			else
			{match=0;
			flag3=0;}
			
		}
		else if (done[var]=='S')
		{					
			movest(190);
			checkbl();
			if(match==0){
			var++;
			}
			else
			{match=0;
			flag3=0;}
		}
		else if (done[var]=='l')
		{
					movestraight(180);
						servo_2(90);
						servo_1(0);
						PORTB=0x00;
						//_delay_ms(3000);
						//turnleft(800);
						left();
						//movestraight(150);//changed to 150
							PORTB=0b00001111;
							OCR5AL=250;
							OCR5BL=250;
							_delay_ms(50);
							

						PORTB=0b00000000;
						servo_2(25);
						_delay_ms(600);
										
						servo_1(155);
						_delay_ms(1000);
										
						servo_2(90);
						_delay_ms(600);
							
						moveback(150);			
								//_delay_ms(1000);	
						left();
			var++;
			
		}
		else if (done[var]=='r')
		{
					movestraight(180);
						servo_2(90);
						servo_1(0);
						right();
						movestraight(100);
						_delay_ms(50);
						servo_2(15);
						_delay_ms(100);
						
						servo_1(135);
						_delay_ms(100);
						
						servo_2(90);
						_delay_ms(100);
						left();
						var++;
						
		}
		else if (done[var]=='P')
				{
								movestraight(180);
					right();
					//movestraight(150);
												PORTB=0b00001111;
												OCR5AL=250;
												OCR5BL=250;
												_delay_ms(50);
					servo_2(80);
					_delay_ms(100);
					
					servo_1(10);
					_delay_ms(100);
					
					servo_2(90);
					_delay_ms(100);
					
					//movestraight(230);
					//_delay_ms(1000);
					//moveback(150); //done to 50
					right();
					var++;
					
				}
		else if (done[var]=='p')
				{
								movestraight(180);
					right();
					//movestraight(150);
												PORTB=0b00001111;
												OCR5AL=250;
												OCR5BL=250;
												_delay_ms(50);
					servo_2(80);
					_delay_ms(600);
					
					servo_1(10);
					_delay_ms(600);
					
					servo_2(90);
					_delay_ms(600);
					PORTE=0x00;
																	PORTB=0b00001111;
																	OCR5AL=250;
																	OCR5BL=250;
																	_delay_ms(5000);
					PORTE=0xFF;
																						PORTB=0b00001111;
																						OCR5AL=250;
																						OCR5BL=250;
																						_delay_ms(10000);
					var++;
					
				}
}

//Main Function
int main(void)
{	

	PostionRight=1000;  
	PostionLeft=1000;  
	encoder_pin_config(); 
	lcd_port_config();
	init_devices();
	lcd_set_4bit();
	lcd_init();
  uart0_init();
  init_devices();
  DDRE=0xFF;
  PORTE=0xFF;
  DDRL=0xFF;  //for pwm
  PORTL=0xFF; //for pwm
  DDRB=0xFF;
  PORTB=0b00000101;
  timer5_init();
  p=30; //p=40 d=8
  d=0;
	  /*
  done[0]='R';done[1]='R';done[2]='l';done[3]='R';  done[4]='L';done[5]='S';done[6]='L';done[7]='S';  done[8]='S';done[9]='S';done[10]='P'; 
  done[11]='S';  done[12]='S';  done[13]='S';  done[14]='R';  done[15]='S';  done[16]='R';  done[17]='S';  done[18]='S';
  done[19]='l';  done[20]='R';  done[21]='S';  done[22]='S';  done[23]='L';  done[24]='S';  done[25]='L';  done[26]='S';done[27]='P';
  */
  var=0;
  							//turnleft(900);
  	 servo_2(90);
  
  while(1)
  {	  
	  //lcd_print(1,1,flag2,1);
	  		//lcd_print(2,3,PostionLeft,4);
	  		//lcd_print(1,3,PostionRight,4);
	flaggen();	 
			 
	if((flag2==0||flag2==5||flag4==1)&&(flag3!=2)) // add a check condition for flag3!=2 because if it is equal to 2 the but is in white line zone hence it dosent need to dettect the node which is in th black line, 
	{	
				_delay_ms(20);				   // similarly for the black line then flag3==0 then it dosent need to check for the white line node 
				flaggen();
		if((flag2==0||flag2==5||flag4==1)) 
		{				   
	while(match==0)
		{
		//linefollowing();
		flaggen();
		if((flag2==1||flag4==1||flag2==5)&&flag2!=2)
		{	
			flag3=1;
			flag4=0;
			break;
		}
		
		}
	
		}
	}
	/*
	
	to detect node on white line the condition for flag2 == 2 and 1 and flag4==0
	then go forward and check if the line is flag2==1 or is flag2==2
	
			sensorvalue();
			if(((ls<50)&&(ms>100)&&(rs<50))||((ls>50)&&(ms<50)&&(rs<50))||((ls<50)&&(ms<50)&&(rs>50)))
			{flag2=1;}
			else if(((ls>50)&&(ms<100)&&(rs>50))||((ls<100)&&(ms>50)&&(rs>50))||((ls>50)&&(ms>50)&&(rs<100)))
			{flag2=0;}
			else if(((ls<50)&&(ms<50)&&(rs<50)))
			{flag2=2;}
			else if(((ls>50)&&(ms>50)&&(rs>50)))
			{flag2=5;}
			else
			{flag2=1;}
			
			if(((ls>50)&&(ms>50))||((ms>50)&&(rs>50)))
			{
				flag4=1;
			}
	*/
		else if((flag2==1||flag2==2||flag4==0)&&(flag3==2)) // add a check condition for flag3!=2 because if it is equal to 2 the but is in white line zone hence it dosent need to dettect the node which is in th black line,
		{	linefollowingbl();					   // similarly for the black line then flag3==0 then it dosent need to check for the white line node
			_delay_ms(15);
			while(match==0)
			{
				//linefollowing();
				flaggen();
				if(flag2==2||flag4==0)
				{
					flag3=3;
					flag4=1;
					break;
					lcd_wr_char('k');
				}
				
			}
			
		}
	
	
	

	/*
	else if(flag2==2)
	{	
		sensorvalue();
		while((ls<15)&&(ms<15)&&(rs<15))
		{
			sensorvalue();//change with line following is needed
			OCR5AL=250;
			OCR5BL=0;
			flag++;
			
			if(flag>50)
			{
				PORTB=0b00001010;
				OCR5AL=RMS;
				OCR5BL=LMS;
				_delay_ms(30);
				for(int n=0;n<400;n++)
				{
					PORTB=0b00000110;
					OCR5AL=100;
					OCR5BL=100;
					_delay_ms(1);
					sensorvalue();
					if((ls<50)&&(ms>100)&&(rs<50))
					{
						flag1=1;
						linefollowing();
						break;
					}
				}
				if(flag1==0)
				{
					for(int n=0;n<1000;n++)
					{
						PORTB=0b00001001;
						OCR5AL=100;
						OCR5BL=100;
						_delay_ms(1);
						sensorvalue();
						if(((ls<50)&&(ms>100)&&(rs<50)))
						{
							linefollowing();
							break;
						}
					}
				}
				flag=0;
				flag1=0;
				break;
			}
		}
		flag=0;
	}
	*/
	
	if(flag3==0)
	{
		linefollowing();
	}
	
	if(flag3==1)
	{

		flag3=0;
		//lcd_wr_char('a');
		turn();
	}
	
	else if(flag3==2)
	{
		//lcd_wr_char('w');
		linefollowingbl();
		
	}
	if(flag3==3)
	{

		flag3=0;
		//lcd_wr_char('a');
		turnbl();
	}
	
	/*

	*/
	//change 8/02/2020
	
	
  }
  }

