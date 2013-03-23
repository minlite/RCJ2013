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

#define cmpbus 0xC0
#define RDC_FOR_MAXON 0.6

/* MUX ADDRESS */
#define MUXA PORTA.6
#define MUXB PORTA.7
#define MUXC PORTD.6
#define MUXD PORTD.7

/* MUX OUT */
#define MUXOA PINA.4
#define MUXOB PINA.5


/* MUX PINS*/
#define LCD 9
#define STRICT 10

/* Switches varibles */
int lcd_enabled = 0;
//int strict_set = 0;

/* Define function prototypes so we can use this functions globally */
int init_robot();
void write_int(int x, int y, int value);
int init_sensors();
void init_switches();
void bug(int error);
void checksensors();
float getMovement();
void set_muxs(int pin);
void goforIt(int m1, int m2, int m3);
void spinSpeed(int devidedValue, int addedValue, int correction);
void init_compass();
int calc_degree(int a);
void compass_calib();
unsigned char readcmp();
void motor(int a, int b, int c);

/* Define global variables */

int rc; // Return Condition
int workingSensors[18] = {1}; // Working Sensors Array
int sensors[18] = {1}; // Sensor Values Array
int i; // For loop iterator
char str[4];
float move,sss;
int motorSpeed;
int motorSpeed1;
int motorSpeed2;
int motorSpeed3;
int compass;
eeprom int start_point;

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
rc = init_robot();
if(rc) bug(rc);
lcd_init(16);
for(i=0;i<18;i++) sensors[i] = 1;
for(i=0;i<18;i++) workingSensors[i] = 1;
while (1)
      { 
        //set_muxs(15);
        //delay_us(1);
        //if(MUXOB == 0) compass_calib(); 
        //init_compass();
        checksensors();
//        move = getMovement();
//        if(move == 0) goforIt(-255,0,255); else
//        if(move == 0.5) goforIt(-208,-46,255); else
//        if(move == 1) goforIt(-165,-88,255); else
//        if(move == 1.5) goforIt(-127,-127,255); else
//        if(move == 2) goforIt(-88,-165,255); else
//        if(move == 2.5) goforIt(-46,-208,255); else
//        if(move == 3) goforIt(0,-255,255); else
//        if(move == 3.5) goforIt(46,-255,208); else
//        if(move == 4) goforIt(88,-255,165); else
//        if(move == 4.5) goforIt(127,-255,127); else
//        if(move == 5) goforIt(165,-255,88); else
//        if(move == 5.5) goforIt(208,-255,46); else
//        if(move == 6) goforIt(255,-255,0); else
//        if(move == 6.5) goforIt(255,-208,-46); else
//        if(move == 7) goforIt(255,-165,-88); else
//        if(move == 7.5) goforIt(255,-127,-127); else
//        if(move == 8) goforIt(255,-88,-165); else
//        if(move == 8.5) goforIt(255,-46,-208); else
//        if(move == 9) goforIt(255,0,-255); else
//        if(move == 9.5) goforIt(208,46,-255); else
//        if(move == 10) goforIt(165,88,-255); else
//        if(move == 10.5) goforIt(127,127,-255); else
//        if(move == 11) goforIt(88,165,-255); else
//        if(move == 11.5) goforIt(46,208,-255); else
//        if(move == 12) goforIt(0,255,-255); else
//        if(move == 12.5) goforIt(-46,255,-208); else
//        if(move == 13) goforIt(-88,255,-165); else
//        if(move == 13.5) goforIt(-127,255,-127); else
//        if(move == 14) goforIt(-165,255,-88); else
//        if(move == 14.5) goforIt(-208,255,-46); else
//        if(move == 15) goforIt(-255,255,0); else
//        if(move == 15.5) goforIt(-255,208,46); else
//        if(move == 16) goforIt(-255,165,88); else
//        if(move == 16.5) goforIt(-255,127,127); else
//        if(move == 17) goforIt(-255,88,165); else
//        if(move == 17.5) goforIt(-255,46,208);
//        motor(-motorSpeed1,-motorSpeed2,-motorSpeed3);         
      }
}
void write_int(int x, int y, int value) {
    char s[4];
    lcd_gotoxy(x,y);
    sprintf(s, "%d", value);
    lcd_puts(s); 
}
int init_robot() {
    init_switches();
    if(lcd_enabled) lcd_init(16);
    rc = init_sensors();
    if(rc) return rc;
    return 0;
}
void set_muxs(int pin) {
    #asm
    LD R30,y
    LD R31,Y
    ROR R30
    ROR R30
    ROR R30
    EOR R30,R31
    ROR R30
    BRCS first
    ; 0000 006D PORTA.6=0;
        CBI  0x1B,6
        JMP next
    first:
    ; 0000 006D PORTA.6=1;
        SBI  0x1B,6
    next:
    MOV R30,R31
    ROR R30
    ROR R30
    EOR R30,R31
    ROR R30
    ROR R30
    BRCS second
    ; 0000 006D PORTA.7=0;
        CBI  0x1B,7
        JMP nextt
    second:
    ; 0000 006D PORTA.7=1;
        SBI  0x1B,7
    nextt:
    MOV R30,R31
    ROR R30
    EOR R30,R31
    ROR R30
    ROR R30
    ROR R30
    BRCS third
    ; 0000 006D PORTD.6=0;
        CBI  0x12,6
        JMP nexttt
    third:
    ; 0000 006D PORTD.6=1;
        SBI  0x12,6
    nexttt:
    MOV R30,R31
    ROR R30
    ROR R30
    ROR R30
    ROR R30
    BRCS fourth
    ; 0000 006D PORTD.7=0;
        CBI  0x12,7
        JMP endzz
    fourth:
    ; 0000 006D PORTD.7=1;
        SBI  0x12,7
    endzz:
    #endasm
    
}
void init_switches(){
    set_muxs(LCD);
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
        set_muxs(sensorHolder);
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
    lcd_gotoxy(0,0);
    for(i=0;i<18;i++) {
        if(workingSensors[i] == 0) sensors[i] = 1; else {
            sensorHolder = i;           
            if(i>15) {swtch = 1; sensorHolder = sensorHolder-16;} else {swtch = 0;}
            set_muxs(sensorHolder);
            sensors[i] = (swtch ? MUXOB : MUXOA);
            lcd_putchar('0' + sensors[i]);
        }
    }
    
}
float getMovement() {
    unsigned char left = 0, right = 0, sum = 0, n = 0;
    float ret = 0.0;
    for(i=0;i<18;i++) {
        if(sensors[i] == 0) { 
            sum+= i;
            n++;
            if(i<5) left++;
            if(i>13) right++;
        }
    }
    if(left>0 && right >0) sum += (left * 18);
    if (n)
        ret = (float) sum/n;
    else ret = 100.0;    
    
    if(ret>=18) ret -= 18.0;
    sss = 10;
    ftoa (ret,1,str);
    lcd_gotoxy(5,1);
    lcd_puts(str);
    return ret; 
}
void goforIt(int m1, int m2, int m3) {
    int motors[3];
    int maxmotor;
    float maxvalue;
    motors[0] = m1;
    motors[1] = m2;
    motors[2] = m3;
    spinSpeed(3,15,10);
    motors[0] = motors[0] + motorSpeed;
    motors[1] = motors[1] + motorSpeed;
    motors[2] = motors[2] + motorSpeed;
    maxmotor=0;
    if(abs(motors[maxmotor])<abs(motors[1]))maxmotor=1;
    if(abs(motors[maxmotor])<abs(motors[2]))maxmotor=2;
    maxvalue = (float)255/abs(motors[maxmotor]);
    motors[0] = motors[0] * maxvalue ;
    motors[1] = motors[1] * maxvalue; 
    motors[2] = motors[2] * maxvalue; 
    motorSpeed1 = motors[0];
    motorSpeed2 = motors[1];        
    motorSpeed3 = motors[2];
}
void spinSpeed(int devidedValue, int addedValue, int correction) {
    if(compass<correction && compass>-correction) {
        motorSpeed = 0;
    } else {
        if(compass > 0) {
            motorSpeed = (((compass)/(devidedValue) + (addedValue)));
        } else {
            motorSpeed = (((compass)/(devidedValue) - (addedValue)));
        }
    }
}
void init_compass() {
    compass = readcmp();
    compass = calc_degree(compass);
}
int calc_degree(int a) {
   // write_int(0,1,a);
    a = a-start_point;
    if(a > 128) {
        a = a - 256;
    } else if(a < -128) {
       a = a + 256;
    }
    return a;
}
unsigned char readcmp() {
    unsigned char data;
    i2c_start();
    i2c_write(cmpbus);
    i2c_write(1);
    i2c_start();
    i2c_write(cmpbus | 1);
    data=i2c_read(0);
    i2c_stop();
    return data;
}
void compass_calib(void){
    set_muxs(15);
    while (MUXOB == 1);
    delay_ms(100);
    i2c_start();
    i2c_write(cmpbus);
    i2c_write(15);
    i2c_write(0xff);
    i2c_stop();
    lcd_clear();
    lcd_putsf("done");
    delay_ms(500);
    lcd_clear();
    set_muxs(15); 
    while (MUXOB == 1);
    delay_ms(100);
    i2c_start();
    i2c_write(cmpbus);
    i2c_write(15);
    i2c_write(0xff);
    i2c_stop();
    lcd_clear();
    lcd_putsf("done");
    delay_ms(500);
    lcd_clear();
    set_muxs(15); 
    while (MUXOB == 1);
    delay_ms(100);
    i2c_start();
    i2c_write(cmpbus);
    i2c_write(15);
    i2c_write(0xff);
    i2c_stop();
    lcd_clear();
    lcd_putsf("done");
    delay_ms(500);
    lcd_clear();
    set_muxs(15); 
    while (MUXOB == 1);
    delay_ms(100);
    i2c_start();
    i2c_write(cmpbus);
    i2c_write(15);
    i2c_write(0xff);
    i2c_stop();
    lcd_clear();
    lcd_putsf("done");
    delay_ms(500);
    lcd_clear();
}
///////////--------------------------- Motor Function ------------------------------/////////// 
void motor(int a, int b, int c) {
    a = a*RDC_FOR_MAXON;
    b = b*RDC_FOR_MAXON;
    c = c*RDC_FOR_MAXON;
    if(a>0) {
        PORTB.2 = 0;
        OCR0 = a;
    } else {
        PORTB.2 = 1;
        a = a + 255;
        OCR0 = a;
    }
    if(c>0) {
        PORTD.2 = 0;
        OCR1B = c;
    } else {
        PORTD.2 = 1;
        c = c + 255;
        OCR1B = c;                                                                    
    }
    if(b>0) {
        PORTD.3 = 0;
        OCR1A = b;
    } else {
        PORTD.3 = 1;
        b = b + 255;
        OCR1A = b;
    }
} 