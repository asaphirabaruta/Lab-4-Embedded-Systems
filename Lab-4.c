/*
 * File:   Lab-4.c
 * Author: asaph
 *
 * Created on March 14, 2023, 10:03 PM
 */

#include <stdio.h>
#include <xc.h>
#include <pic16f877a.h>

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7
#define DHT11_PIN RA2;
#define _XTAL_FREQ 20000000

// Define the potentiometer pin and its resistance
#define POT_PIN 0 // AN0 analog input pin
#define POT_RESISTANCE 10000 // 10 kohm potentiometer resistance
#define SPEED_CONSTANT 1.0 // Proportional constant for the speed calculation

// Global variables
float voltage;
float speed;
unsigned int timer_count = 0; // variable to hold timer count value
float distance = 0; // variable to hold distance covered

void timer1_init(void);
void adc_init(void);
void timerISR();

void LCD_SETBIT(char data_bit)
{
    if(data_bit& 1) 
        D4 = 1;
    else
        D4 = 0;

    if(data_bit& 2)
        D5 = 1;
    else
        D5 = 0;

    if(data_bit& 4)
        D6 = 1;
    else
        D6 = 0;

    if(data_bit& 8) 
        D7 = 1;
    else
        D7 = 0;
}

void LCD_CMD(char a)
{
    RS = 0;           
    LCD_SETBIT(a); //Incoming Hex value
    EN  = 1;         
        __delay_ms(4);
        EN  = 0;         
}

LCD_CLEAR()
{
    LCD_CMD(0); //Clear the LCD
    LCD_CMD(1); //Move the cursor to first position
}

void LCD_SET_CURSOR(char a, char b)
{
    char temp,z,y;
    if(a== 1)
    {
      temp = 0x80 + b - 1; //80H is used to move the cursor
        z = temp>>4; //Lower 8-bits
        y = temp & 0x0F; //Upper 8-bits
        LCD_CMD(z); //Set Row
        LCD_CMD(y); //Set Column
    }
    else if(a== 2)
    {
        temp = 0xC0 + b - 1;
        z = temp>>4; //Lower 8-bits
        y = temp & 0x0F; //Upper 8-bits
        LCD_CMD(z); //Set Row
        LCD_CMD(y); //Set Column
    }
}

void LCD_START()
{
  LCD_SETBIT(0x00);
  for(int i=1065244; i<=0; i--)  NOP();  
  LCD_CMD(0x03);
    __delay_ms(5);
  LCD_CMD(0x03);
    __delay_ms(11);
  LCD_CMD(0x03); 
  LCD_CMD(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  LCD_CMD(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  LCD_CMD(0x08); //Select Row 1
  LCD_CMD(0x00); //Clear Row 1 Display
  LCD_CMD(0x0C); //Select Row 2
  LCD_CMD(0x00); //Clear Row 2 Display
  LCD_CMD(0x06);
}

void LCD_PRINT_CHAR(char data) 
{
   char Lower_Nibble,Upper_Nibble;
   Lower_Nibble = data&0x0F;
   Upper_Nibble = data&0xF0;
   RS = 1;             // => RS = 1
   LCD_SETBIT(Upper_Nibble>>4);             
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP(); 
   EN = 0;
   LCD_SETBIT(Lower_Nibble); //Send Lower half
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP();
   EN = 0;
}

void LCD_PRINT_STRING(char *a)
{
    int i;
    for(i=0;a[i]!='\0';i++)
       LCD_PRINT_CHAR(a[i]); 
}

void ISR()
{
        if (INTF == 1) //External Interrupt detected
        { 
            LCD_CLEAR();
            LCD_SET_CURSOR(1,1);
            LCD_PRINT_STRING("Ext. ISR");

            LCD_SET_CURSOR(2,1);
            LCD_PRINT_STRING("Drive Safely");
            INTF = 0;          // clear the interrupt flag after done with it

            __delay_ms(5000);
            LCD_CLEAR();
        }

}

void SETUP() 
{
    ADCON1 = 0x80; // Configure AN0 pin as analog input
    TRISA = 0xff; // Set all PORTA pins as inputs
}

void timer1_init(void) 
{
    T1CON = 0b00000001; // prescaler 1:1, timer1 on
    TMR1IF = 0; // clear timer1 interrupt flag
    TMR1IE = 1; // enable timer1 interrupt
    PEIE = 1; // enable peripheral interrupts
    GIE = 1; // enable global interrupts
}

void timerISR() 
{
    if (T0IF) {
        
        timer_count++;        
        // Reset timer0
        TMR0 = 6;        
        // Clear timer0 interrupt flag
        T0IF = 0;
    }
}

void Speed_Function() 
{
    
    unsigned int adc_value;
    ADCON0 = 0x01; // Turn on ADC and select AN0 channel
    __delay_us(20); // Wait for acquisition time
    ADCON0bits.GO = 1; // Start conversion
    
        
   while (ADCON0bits.GO); // Wait for conversion to complete
   
    adc_value = (ADRESH << 8) + ADRESL;
    
    
    voltage= adc_value*5/1023 ; // Calculate voltage

    // Calculate speed based on potentiometer voltage
    speed = voltage*32; // // Assume 1V = 12 km/h

    if(speed>=60)
    {
   
        char my_string[16]; // Declare a character array to hold the string
    
        sprintf(my_string, "Speed:%.2f km/hour", speed);
        LCD_CLEAR();
       
        LCD_SET_CURSOR(1,1);
        LCD_PRINT_STRING("Max speed Reached.");
        LCD_SET_CURSOR(2,1);
        LCD_PRINT_STRING("Drive Safely.");
         __delay_ms(1000);
    }
    // Display speed on LCD or other output device
   // printf("Speed: %.2f km/h\n", speed);
       char my_string[16]; // Declare a character array to hold the string

    // Format the variable as a string using sprintf
        sprintf(my_string, "Speed:%.2f km/h", speed);
        
        LCD_SET_CURSOR(1,1);   // sets the cursor on the first line of the LCD
        //Lcd_Print_String("Temperature: %.2f C\n");
        LCD_PRINT_STRING(my_string);
        
        //Timer interrupt
            timerISR();
          
            char my_stringxp[16];
            
            distance=speed*timer_count;
            
            sprintf(my_stringxp, "Dis: %.0f Meters",distance);
            LCD_SET_CURSOR(2,1);   
            LCD_PRINT_STRING(my_stringxp);
            
            __delay_ms(1000);
            LCD_CLEAR();
}
 
void ADC_Initialize()
{
    ADCON0 = 0b01000001; //ADC ON and Fosc/16 is selected
    ADCON1 = 0b11000000; // Internal reference voltage is selected
}

unsigned int ADC_Read(unsigned char channel)
{
    ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
    ADCON0 |= channel<<3; //Setting the required Bits
    __delay_ms(2); //Acquisition time to charge hold capacitor
    GO_nDONE = 1; //Initializes A/D Conversion
    while(GO_nDONE); //Wait for A/D Conversion to complete
    return ((ADRESH<<8)+ADRESL); //Returns Result
}

int main()
{
    float volt,temp_val,kelvin,temp_celsius,temp_fahrenheit;
    unsigned int a;
    char tc[10];
    char tf[10];
    int temps=0;
    int x=1;    
   
    TRISA = 0x00; 
    PORTA = 0x00; 
    TRISB0 = 1;        //DEfine the RB0 pin as input to use as interrupt pin
    TRISD = 0;    //sets port D as output for LCD output
    TRISA2 = 1;    //sets port RA0 as input for the LM35
    OPTION_REG = 0b00000000;  // Enables PULL UPs
    GIE=1;          //Enable Global Interrupt
    PEIE=1;         //Enable the Peripheral Interrupt
    INTE = 1;       //Enable RB0 as external Interrupt pin
    
    LCD_START();

    LCD_CLEAR();
        
    LCD_SET_CURSOR(1,1); 
    LCD_PRINT_STRING("Speedometer");
    
    LCD_SET_CURSOR(2,1);  
    LCD_PRINT_STRING("Loading....");
    
    __delay_ms(1000);
        
    timer1_init();    
    
    while(1)
    {
        ISR();
        temps = 36+x;
        x++;
        ADC_Initialize();
        temp_celsius = temps;        
        
        if(temp_celsius>=40)
            {
                temp_fahrenheit = (9/5*temp_celsius)+32;
        
                LCD_CLEAR();
                
                LCD_SET_CURSOR(1,1);   // sets the cursor on the first line of the LCD
                LCD_PRINT_STRING("High Temperature");
                
                sprintf(tc,"%.2f",temp_celsius);
                LCD_PRINT_STRING(tc);
                
                LCD_SET_CURSOR(2,1);   // sets the cursor on the second line of the LCD
                LCD_PRINT_STRING("Check coolant");
                
                __delay_ms(2000);
            }
        
        temp_fahrenheit = (9/5*temp_celsius)+32;
        
        LCD_CLEAR();
        
        LCD_SET_CURSOR(1,1);   // sets the cursor on the first line of the LCD
        LCD_PRINT_STRING("Temperature:");
        
        sprintf(tc,"%.1f Celsius",temp_celsius);  
        LCD_PRINT_STRING(tc);
        
        __delay_ms(2000);
        
        SETUP();
         
        Speed_Function();           
    }
    return 0;
}



