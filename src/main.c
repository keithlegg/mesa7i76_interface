#include <avr/io.h>

#define F_CPU 16000000UL //AVR Clock Speed in MHZ
#define FOSC 16000000    // Clock Speed

#include <util/delay.h>
#include "nes_ctrlr.h"


// DDRB |= (1 << DDB3);     // set pin 3 of Port B as output
// PORTB |= (1 << PB3);     // set pin 3 of Port B high
// PORTB &= ~(1 << PB3);    // set pin 3 of Port B low
// PORTB |= (1 << PORTB3);  // set pin 3 high again

// DDRB &= ~(1<<0); // configure PB0 as an input
// PORTB |= (1<<0); // enable the pull-up on PB0


/*
  This is a "glue logic" board to use an Atmega128 chip as an interface to Linux CNC with the MESA 7i76 
  We read the differential signals and steper motor signals and output a single enable and direction   

  This is for small experimental CNC machines, obviously, dont run anything big and dangerous with it. 
  If you cut your hand off, its not my fault. 


  # PORT MAPPING FOR HEXBUS 
  # PIN MAPPING FOR HEXBUS IS BACKWARDS 
  0-1-2-3-4-5-6-7 
  ----------------
  PORTF --  -- PORTA
  PORTE --  -- PORTC
  ----------------
  PORTB --  -- XX
  PORTD --  -- XX
  ----------------




  ----------------- 
  
  # PIN MAPPING FOR CABLE TO 7i76  
    (FROM TOP - CABLE PLUGGED IN PORT_B/D - NOTCH FACING UP) 
  ----------------
  step0-| step0+| dir0-| dir0+ | step1-| step1+| dir1-| dir1+ 
  step2-| step2+| dir2-| dir2+ |  XX   |  XX   |  XX  |  XX   
  ----------------

  # PIN MAPPING FOR CABLE TO MOTOR DRIVERS  
    (FROM TOP - CABLE PLUGGED IN PORT_A/C - NOTCH FACING UP) 
  ----------------
  XEN    | YEN    | ZEN   | XX     | XX    | XX   
  XDIRLO | XDIRHI | YDIRLO| YDIRHI | ZDIRLO| ZDIRHI  
  ----------------

*/



// speed between stepper pulses in millisecond
// because this is driving a brushed motor - the time will directly affect the "step" length
const uint8_t run_del = 3; 


uint8_t xaxis_dir  = 0; 
uint8_t xaxis_step = 0; 
uint8_t yaxis_dir  = 0; 
uint8_t yaxis_step = 0; 
uint8_t zaxis_dir  = 0; 
uint8_t zaxis_step = 0; 


void read_state(void)
{
    /*
       example to read the state of PORTB and set PORTA as output  

      PIN MAPPING FOR CABLE TO 7i76  
      b0 step0-| b1 step0+| b2 dir0-| b3 dir0+ | b4 step1-| b5 step1+| b6 dir1-| b7 dir1+ 
      d0 step2-| d1 step2+| d2 dir2-| d2 dir2+ |          |          |         |     

    */

    //-------
    //READ X AXIS 

    // read step X 
    uint8_t bit1 = (PINB >> 1) & 1;   
    uint8_t bit2 = (PINB >> 2) & 1; 
    // read two differential pins   
    if (bit1==0x00 && bit2==0x01)
    {
        xaxis_step = 0x01;
    } else{
        xaxis_step = 0x00;        
    } 

    //read X direction 
    uint8_t bit3 = (PINB >> 3) & 1;    
    if (bit3==0x01)
    {
        xaxis_dir = 0x00; 
    } else{
        xaxis_dir = 0x01;         
    } 

    //-------
    // READ Y AXIS 
    
    // read step Y 
    uint8_t bit4 = (PINB >> 4) & 1;   
    uint8_t bit5 = (PINB >> 5) & 1; 
    // read two differential pins   
    if (bit4==0x00 && bit5==0x01)
    {
        yaxis_step = 0x01;
    } else{
        yaxis_step = 0x00;    
    } 

    //read Y direction 
    uint8_t bit6 = (PINB >> 6) & 1;    
    if (bit6==0x01)
    {
        yaxis_dir = 0x00; 
    } else{
        yaxis_dir = 0x01;         
    } 

    //-------
    // READ Z AXIS 

    // read step Z 
    uint8_t bit7 = (PIND >> 0) & 1;   
    uint8_t bit8 = (PIND >> 1) & 1; 
    // read two differential pins   
    if (bit7==0x00 && bit8==0x01)
    {
        zaxis_step = 0x01;
    } else{
        zaxis_step = 0x00;    
    } 

    //read Z direction 
    uint8_t bit9 = (PIND >> 2) & 1;    
    if (bit9==0x01)
    {
        zaxis_dir = 0x00; 
    } else{
        zaxis_dir = 0x01;         
    } 

}



void knightrider(void)
{
    uint8_t i = 0; 
    uint8_t dly = 10; 

    for(uint8_t i = 0; i<7; i++)  
    {           
        PORTA |= (1<<i);     
        _delay_ms(dly);  
        PORTA &= ~(1<<i);   
        _delay_ms(dly);  
    }  
    for(i=7; i>0; i--)
    {
        PORTA |= (1<<i);
        _delay_ms(dly);
        PORTA &= ~(1<<i);
        _delay_ms(dly);
    } 

}


void set_x_dir(uint8_t xdir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (xdir==0x00)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTA |=  (1 << PA1); //pin 1 high        
        PORTA &= ~(1 << PA2); //pin 2 low 

        // // pulse the enable line 
        // _delay_ms(run_del);
        // PORTA |= (1 << PA0);    
        // _delay_ms(run_del);
        // PORTA &= ~(1 << PA0);    
    }
    
    if (xdir==0x01)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTA |=  (1 << PA2); //pin 1 low        
        PORTA &= ~(1 << PA1); //pin 2 high 
    }

}

void set_y_dir(uint8_t ydir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (ydir==0x00)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTA |=  (1 << PA4); //pin 1 high        
        PORTA &= ~(1 << PA5); //pin 2 low 
    }
    
    if (ydir==0x01)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTA |=  (1 << PA5); //pin 1 low        
        PORTA &= ~(1 << PA4); //pin 2 high 
    }

}

void set_z_dir(uint8_t zdir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (zdir==0)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTC |=  (1 << PA1); //pin 1 high        
        PORTC &= ~(1 << PA2); //pin 2 low 

    }
    
    if (zdir==1)
    {
        // set hi-low bridge controls (never both the same!) 
        PORTC |=  (1 << PA2); //pin 1 low        
        PORTC &= ~(1 << PA1); //pin 2 high 
    
    }

}



void step_x_axis(void)
{
    // STEPX (pulse the enable line )
    _delay_ms(run_del);
    PORTA |= (1 << PA0);      
    _delay_ms(run_del);
    PORTA &= ~(1 << PA0); 
}


void step_y_axis(void)
{
    // STEPY (pulse the enable line )
    _delay_ms(run_del);
    PORTA |= (1 << PA3);      
    _delay_ms(run_del);
    PORTA &= ~(1 << PA3); 
}


void step_z_axis(void)
{
    // STEPZ (pulse the enable line )
    _delay_ms(run_del);
    PORTC |= (1 << PA0);      
    _delay_ms(run_del);
    PORTC &= ~(1 << PA0);
}
 


void test(void){

    set_x_dir(0);
    set_y_dir(0);
    set_z_dir(0);        
    _delay_ms(100);
    
    for (uint8_t xx=0;xx<10;xx++){
        step_x_axis();
    }
    for (uint8_t xx=0;xx<10;xx++){
        step_y_axis();
    }
    for (uint8_t xx=0;xx<10;xx++){
        step_z_axis();
    }    
  

    set_x_dir(1);
    set_y_dir(1);
    set_z_dir(1);
    _delay_ms(100);

    for (uint8_t xx=0;xx<10;xx++){
        step_x_axis();
    }
    for (uint8_t xx=0;xx<10;xx++){
        step_y_axis();
    }
    for (uint8_t xx=0;xx<10;xx++){
        step_z_axis();
    }  

}
 


int main (void)
{

    DDRA = 0xff; // PORTA all output
    DDRC = 0xff; // PORTC all output

    DDRB = 0x00;  // PORTB all input
    //DDRD = 0x00;  // PORTB all input 

    //PORTB = 0xff; // PORTB pull all pins up  
    //PORTD = 0xff; // PORTD pull all pins up 

    NES_CTRLR_Init();

    while (1)
    {
        //xaxis_step = (PINB & 0x03) == 0x01;
        
        //uint8_t pinValue = (PINB & (1 << PINB3)) >> PINB3;
        //uint8_t pinValue = (PINB&0x01)>>0x01;

        //check a single pin and shift it to first bit to make a 1 or 0 for the resgister
        //uint8_t pinValue = PINB & (1 << PINB4);
        //PORTA = pinValue;

        //read_state();
        uint8_t foo = 0x00;

        // 0xFF - nothing pressed 
        // 0xF7 - up 
        // 0xFD - left 
        // 0xFB - down 
        // 0xF7 - up 
        // 0xF6 - right/up
        // 0xFA - right/down
        // 0xF5 - left/up      
        // 0xF9 - left/down  
        // 0xFE - right 
        // mask of high bit for buttons other than direction 
        // 0xDF - select 
        // 0xEF - start
        // 0xBF - B button 
        // 0x7F - A button 
        // 0x3X - both A,B buttons 

        uint8_t nes_byte = nes_controller_read();
        //if(nes_byte == 0xFE ) { foo=0x01; } //right
        //if(nes_byte == 0xFD ) { foo=0x02; } //left
        //if(nes_byte == 0xFB ) { foo=0x03; } //down
        //if(nes_byte == 0xF7 ) { foo=0x04; } //up
        //if(nes_byte == 0x7F ) { foo=0x05; } //button a

        PORTA = nes_byte;
    }

    return 0;
} 





