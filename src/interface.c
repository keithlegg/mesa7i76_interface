#include <avr/io.h>

#define F_CPU 16000000UL //AVR Clock Speed in MHZ
#define FOSC 16000000    // Clock Speed

#include <util/delay.h>


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
const uint8_t run_del = 100; 


uint8_t xaxis_dir  = 0; 
uint8_t xaxis_step = 0; 
uint8_t yaxis_dir  = 0; 
uint8_t yaxis_step  = 0; 
uint8_t zaxis_dir  = 0; 
uint8_t zaxis_step  = 0; 


void read_state(void)
{
    /*
       example to read the state of PORTB and set PORTA as output  

      PIN MAPPING FOR CABLE TO 7i76  
      b0 step0-| b1 step0+| b2 dir0-| b3 dir0+ | b4 step1-| b5 step1+| b6 dir1-| b7 dir1+ 
      d0 step2-| d1 step2+| d2 dir2-| d2 dir2+ |          |          |         |     

    */

    // xaxis_step =  (PINB & (1 << PINB2)) >> PINB2;

    uint8_t bit1 = (PINB >> 1) & 1; //b1 on 
    uint8_t bit2 = (PINB >> 2) & 1; //b2 off 

    if (bit1==0x00 && bit2==0x01)
    {
        PORTA = 0xaa;
    } else{
        PORTA = 0x00;        
    } 


    // xaxis_dir  = PINB & (1 << PINB4); 
    // yaxis_dir  = PINB & (1 << PINB4); 
    // yaxis_dir  = PINB & (1 << PINB4); 
    // zaxis_dir  = PINB & (1 << PINB4); 
    // zaxis_dir  = PINB & (1 << PINB4); 


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
 



 


int main (void)
{

    DDRA = 0xff; // PORTA all output
    DDRC = 0xff; // PORTC all output

    DDRB = 0x00;  // PORTB all input
    //DDRD = 0x00;  // PORTB all input 

    //PORTB = 0xff; // PORTB pull all pins up  
    //PORTD = 0xff; // PORTD pull all pins up 

    while (1)
    {
        //xaxis_step = (PINB & 0x03) == 0x01;
        
        //uint8_t pinValue = (PINB & (1 << PINB3)) >> PINB3;
        //uint8_t pinValue = (PINB&0x01)>>0x01;

        //check a single pin and shift it to first bit to make a 1 or 0 for the resgister
        //uint8_t pinValue = PINB & (1 << PINB4);
        //PORTA = pinValue;

        read_state();


    }

    return 0;
} 





