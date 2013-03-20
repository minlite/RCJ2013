/*****************************************************
Project : AMOS Robocup Junior Soccer 2013 
Version : 3
Date    : 3/17/2013
Author  : Miro Markarian and AMOS team
Company : AMOS
Comments: 


Chip type               : ATmega16
Program type            : Application
AVR Core Clock frequency: 10.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*****************************************************/

#include <mega16.h>
#include <stdio.h>
// I2C Bus functions
#asm
   .equ __i2c_port=0x18 ;PORTB
   .equ __sda_bit=0
   .equ __scl_bit=1
#endasm
#include <i2c.h>
#include <delay.h>
#include <stdlib.h>

// Alphanumeric LCD Module functions
#include <alcd.h>

// Declare your global variables here


/* MUX ADDRESS */
#define MUXA PORTA.6
#define MUXB PORTA.7
#define MUXC PORTD.6
#define MUXD PORTD.7

/* MUX OUT */
#define MUXOA PINA.4
#define MUXOB PINA.5


/* MUX PINS*/
#define LCD 15
#define STRICT 14

/* Switches varibles */
int lcd_enabled = 0;
//int strict_set = 0;

/* Define function prototypes so we can use this functions globally */
int init_robot();
void write_int(int x, int y, int value);
void set_mux(int pin);
int init_sensors();
void init_switches();
void bug(int error);
void checksensors();
float getMovement();

void lcd_writeint(int x, int y, int value);

/* Define global variables */

int rc; // Return Condition
int workingSensors[18] = {1}; // Working Sensors Array
int sensors[18] = {1}; // Sensor Values Array
int i; // For loop iterator
float move;
void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Func7=Out Func6=Out Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=0 State6=0 State5=P State4=P State3=T State2=T State1=T State0=T 
PORTA=0x30;
DDRA=0xC0;

// Port B initialization
// Func7=In Func6=In Func5=In Func4=In Func3=Out Func2=Out Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=0 State2=0 State1=T State0=T 
PORTB=0x00;
DDRB=0x0C;

// Port C initialization
// Func7=In Func6=In Func5=In Func4=In Func3=Out Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=1 State2=T State1=T State0=T 
PORTC=0x08;
DDRC=0x08;

// Port D initialization
// Func7=Out Func6=Out Func5=Out Func4=Out Func3=Out Func2=Out Func1=In Func0=In 
// State7=0 State6=0 State5=0 State4=0 State3=0 State2=0 State1=T State0=T 
PORTD=0x00;
DDRD=0xFC;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 1250.000 kHz
// Mode: Fast PWM top=0xFF
// OC0 output: Non-Inverted PWM
TCCR0=0x6A;
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1250.000 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inv.
// OC1B output: Non-Inv.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0xA1;
TCCR1B=0x0A;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
ASSR=0x00;
TCCR2=0x00;
TCNT2=0x00;
OCR2=0x00;

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
MCUCR=0x00;
MCUCSR=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x00;

// USART initialization
// USART disabled
UCSRB=0x00;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// ADC initialization
// ADC disabled
ADCSRA=0x00;

// SPI initialization
// SPI disabled
SPCR=0x00;

// TWI initialization
// TWI disabled
TWCR=0x00;

// I2C Bus initialization
i2c_init();

// Alphanumeric LCD initialization
// Connections specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTC Bit 0
// RD - PORTC Bit 1
// EN - PORTC Bit 2
// D4 - PORTC Bit 4
// D5 - PORTC Bit 5
// D6 - PORTC Bit 6
// D7 - PORTC Bit 7
// Characters/line: 16
//rc = init_robot();
//if(rc) bug(rc);
lcd_init(16);
for(i=0;i<18;i++) sensors[i] = 1;
while (1)
      {
      }
}
void write_int(int x, int y, int value) {
    char s[4];
    lcd_gotoxy(x,y);
    sprintf(s, "%d", value);
    lcd_puts(s); 
}
void lcd_writeint(int x, int y, int value) {
    lcd_gotoxy(x,y);
    lcd_putchar('0' + value/1000);
    lcd_putchar('0' + (value/100)%10);
    lcd_putchar('0' + (value/10)%10);
    lcd_putchar('0' + value%10);
}
int init_robot() {
    init_switches();
    if(lcd_enabled) lcd_init(16);
    rc = init_sensors();
    if(rc) return rc;
    return 0;
}
void set_mux(int pin) {
    MUXD = (pin/8);
    MUXC = (pin>>3) & 1;
    MUXB = (pin>>2) & 1;
    MUXA = (pin>>1) & 1;
}
void init_switches(){
    set_mux(LCD);
    lcd_enabled = MUXOB;
    //set_mux(STRICT);
    //strict_set = MUXOB;
}
int init_sensors() {
    char sensorHolder; 
    char swtch;
    for(i=0;i<18;i++) {
        sensorHolder = i;           
        if(i>15) {swtch = 1; sensorHolder = sensorHolder-16;} else {swtch = 0;}
        set_mux(sensorHolder);
        if((swtch ? MUXOB : MUXOA) == 0) workingSensors[i] = 1;
    }
    return 0;
}
void bug(int error) {
    if(lcd_enabled) {
        lcd_puts("BUG on: ");
        lcd_putchar('0' + error);
    } else {
        while(1);}   
}
void checksensors() {
    char sensorHolder; 
    char swtch;
    for(i=0;i<18;i++) {
        if(workingSensors[i] == 0) sensors[i] = 1; else {
            sensorHolder = i;           
            if(i>15) {swtch = 1; sensorHolder = sensorHolder-16;} else {swtch = 0;}
            set_mux(sensorHolder);
            sensors[i] = (swtch ? MUXOB : MUXOA); 
        }
    }
    
}
float getMovement() {
    unsigned char left = 0, right = 0, sum = 0, n = 0;
    float ret = 0;
    for(i=0;i<18;i++) {
        if(sensors[i] == 0) { 
            sum+= i;
            n++;
            if(i<5) left++;
            if(i>13) right++;
        }
    }
    if(left>0 && right >0) sum += (left * 18);
    ret = (float) sum/n;
    if(ret>18) ret -= 18;
    return ret; 
} 