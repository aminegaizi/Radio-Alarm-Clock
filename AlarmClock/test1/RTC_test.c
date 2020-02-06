#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"


//***********************************************************************
//                    Global variables
//***********************************************************************
volatile uint8_t second = 0; 
volatile uint8_t minute = 0;
volatile uint8_t hour = 0; 
uint8_t dummy_counter = 0; 
uint8_t second_flag = 0; 
uint8_t segment_data[5]; 
//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};


uint8_t  i;              //dummy variable
uint16_t adc_result;     //holds adc result 
char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd  
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 
//***********************************************************************
//                     Function prototypes
//***********************************************************************
void TIM0_RTC_init();
void TimeProcessing();
void InitLEDdisplay();
void SelectDigit(int DigitNumber);
void SeperateDigits();
void PWM_init(void);



void spi_init(void)
{
 /* Run this code before attempting to write to the LCD.*/
 DDRF  |= 0x08;  //port F bit 3 is enable for LCD
 PORTF &= 0xF7;  //port F bit 3 is initially low

 DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
 PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
//see: /$install_path/avr/include/avr/iom128.h for bit definitions   

 //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
 SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
 SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
 }

void TimeProcessing() 
{
	if(dummy_counter == 128)
	{
		second++;
		dummy_counter = 0;
	}

/*	if(dummy_counter == 2)
	{
		second++;
		dummy_counter == 0;
	}
*/
	//second++;
	//if(second_flag == 1)
	if(dummy_counter == 127)
	{second_flag = 0;}
	//if(second%2 == 0)
	if(dummy_counter == 63)
	{second_flag = 1;}

	if(second >= 60)
	{	minute++; 
		second = 0;}
	if(minute == 60)
	{	hour++;
		minute = 0;}
	if(hour == 24)
	{	hour = 0;} 
	dummy_counter++; 
	//PORTB = ~PORTB; 
}

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************

ISR( TIMER0_OVF_vect ) 
{
 	TimeProcessing();
 	//SeperateDigits();
 	return;	
}

//***********************************************************************
//                     Timer 0 RTC mode initialization
//***********************************************************************
void TIM0_RTC_init()
{
	ASSR |= (1<<AS0); //Timer 0 clocked from oscillator clkTOS = 32768Hz
	TCNT0 = 0x00; //Reset start value for the timer
	TCCR0 = (0<<CS2) | (0<<CS1) | (1<<CS0); //Select prescaler value of 1-> 128 interrupt every second
	while(!bit_is_clear(ASSR, TCN0UB)) //Wait for ASSR to be updated
	{}
	TIMSK |= (1<<TOIE0); //Enable Timer 0 Overflow interrupt
	sei(); //Enable global interrupt
}

void TIM3_INT()
{
	//TCCR3A
	TCCR3B |= 0x1; //No prescaling  
}

//***********************************************************************
//                     ADC single ended mode initialization
//***********************************************************************
void ADC_init()
{
	//Initalize ADC and its ports
DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

ADMUX = 0x47; //writes 00111 to ADMUX (4:0) for single-ended, input PORTF bit 7,
// right adjusted, 10 bits

ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
 //ADC enabled, don't start yet, single shot mode 
}

//void InitLEDdisplay()
//This function initializes the IO to use the 4Digit LED display
void InitLEDdisplay()
{
	DDRA = 0xFF; //Set PORTA to all output
	DDRB |= 0XF0; //Set PORTB pins 4 to 7 to output
	asm("nop"); //IO Synchronization delay
	asm("nop"); //IO Synchronization delay
	PORTB |= 0b01100000; //Set PB5 and PB6 to 1 (no digit selected on Display)
	PORTB &= 0b01101111; //Set PB4 and PB7 to 0 (transistor saturated) 
}

//This function take in parameter the DigitNumber to select
//It chooses the correct parameters to turn on that digit
void SelectDigit(int DigitNumber)
{
	switch (DigitNumber)
        {
                case 0: //Choose digit number 1 (from the right)
                        PORTB = (0<<PORTB4) | (0<<PORTB5) | (0<<PORTB6);
                        break;
                case 1:  //Choose digit number 2 
                        PORTB = (1<<PORTB4) | (0<<PORTB5) | (0<<PORTB6);
                        break;
                case 2: //Choose the colon 
                		PORTB = (0<<PORTB4) | (1<<PORTB5) | (0<<PORTB7);
                		break; 
                case 3:  //Choose digit number 3
                        PORTB = (1<<PORTB4) | (1<<PORTB5) | (0<<PORTB6);
                        break;
                case 4: //Choose digit number 4
						PORTB = (0<<PORTB4) | (0<<PORTB5) | (1<<PORTB6);
                        break;
                default:
                        break;
        }
}

/*void SeperateDigits(int counter)
*This function separates the counter value into 4 different digits to be displayed
*The value of the number to be displayed by each digit is a modulo of 10
*/
void SeperateDigits()
{
	segment_data[0] = dec_to_7seg[minute%10]; //The ones for minutes
    segment_data[1] = dec_to_7seg[(minute/10)%10]; //The tens for minutes
    segment_data[2] = dec_to_7seg[hour%10]; //The ones for hours
    segment_data[3] = dec_to_7seg[(hour/10)%10]; //The tens for hour
    segment_data[4] = 0xFC; //The tens for hour

    uint8_t colon = 0xFC; 

    SelectDigit(0); //Select digit 0
    PORTA = segment_data[0]; //Display the "ones" value
    _delay_ms(1); 

    SelectDigit(1); //Select Digit 1
    PORTA = segment_data[1]; //Dispay "tens" value
    _delay_ms(1);

    SelectDigit(2); //Select Digit 2: colon
    if(second_flag == 0)
    	PORTA = 0xFF;
    if(second_flag == 1)
    {	PORTA = 0xFC;
   	}
   	_delay_ms(1);

    SelectDigit(3); //Select Digit 3 
    PORTA = segment_data[2]; //Display hundreds valus
    _delay_ms(1);

    SelectDigit(4); //Select Digit 4
    PORTA = segment_data[3]; //Display thousands value
    _delay_ms(1);

}

void PWM_init(void)
{
    /* Enable non inverting 8-Bit PWM */    
    //TCCR2A = (1<<COM2A1)|(1<<WGM21)|(1<<WGM20);    
    /* Timer clock = I/O clock */    
    //TCCR2B = (1<<CS20);   
    TCCR2 = 0x69; 
    /* Set the compare value to control duty cycle */    
    OCR2  = 0x80;    
    /* Enable Timer 2 Output Compare Match Interrupt */    
    TIMSK = 0x10;   
    /* Set OC2A pin as output */
    //DDRB |= (1 << PB7);
    DDRB |= 0b10000000;
}


//**********************************************************************
//                                main                                 
//**********************************************************************
int main(){     

	InitLEDdisplay();
	_delay_ms(2);
	//DDRB = 0xFF; 
	_delay_ms(1);
	//PORTB = 0xFF;
	PWM_init();	
 	TIM0_RTC_init();

	//initalize the SPI port then the LCD
	spi_init();
	lcd_init(); 
	clear_display();
	ADC_init();

	while(1)
    {
    	SeperateDigits();
    	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion

 		while(bit_is_clear(ADCSRA,ADIF)){}; //spin while interrupt flag not set

		ADCSRA |= (1<<ADIF); //its done, clear flag by writing a one 
    	adc_result = ADC;                      //read the ADC output as 16 bits

 
		//now determine Vin, where Vin = (adc_result/204.8)
    	fp_adc_result = div(adc_result, 205);              //do division by 205 (204.8 to be exact)
    	itoa(fp_adc_result.quot, lcd_str_h, 10);           //convert non-fractional part to ascii string
    	fp_low_result = div((fp_adc_result.rem*100), 205); //get the decimal fraction into non-fractional form 
    	itoa(fp_low_result.quot, lcd_str_l, 10);           //convert fractional part to ascii string

    	//send string to LCD
    	string2lcd(lcd_str_h);  //write upper half
    	char2lcd('.');          //write decimal point
    	string2lcd(lcd_str_l);  //write lower half

    	for(i=0;i<=10;i++){ _delay_ms(50);}  //delay 0.5 sec
    	clear_display();
    	cursor_home();
 	} //while(1)
} //main