#include <avr/io.h>

#define F_CPU 16000000UL //AVR Clock Speed in MHZ
#define FOSC 16000000    // Clock Speed

#include <util/delay.h>



/*
  # PORT MAPPING FOR KEITHS BOARD 
  
  ----------------
  PORTF --  -- PORTA
  PORTE --  -- PORTC
  ----------------
  PORTB --  -- XX
  PORTD --  -- XX
  ----------------

  
  # PIN MAPPING FOR CABLE TO 7i76  
    (FROM TOP - CABLE PLUGGED IN - NOTCH FACING UP) 

  ----------------
  step0-| step0+| dir0-| dir0+ | step1-| step1+| dir1-| dir1+ 
  step2-| step2+| dir2-| dir2+ |  XX   |  XX   |  XX  |  XX   
  ----------------

*/


uint8_t xaxis_dir  = 0; 
uint8_t xaxis_step = 0; 

uint8_t yaxis_dir  = 0; 
uint8_t yaxis_step  = 0; 

uint8_t zaxis_dir  = 0; 
uint8_t zaxis_step  = 0; 



// DDRB |= (1 << DDB3);     // set pin 3 of Port B as output
// PORTB |= (1 << PB3);     // set pin 3 of Port B high
// PORTB &= ~(1 << PB3);    // set pin 3 of Port B low
// PORTB |= (1 << PORTB3);  // set pin 3 high again

// DDRB &= ~(1<<0); // configure PB0 as an input
// PORTB |= (1<<0); // enable the pull-up on PB0




void read_state(void)
{
    //xaxis_step = PINB & ((0 << PINB1) & (1 << PINB2));
    
    xaxis_step = (PINB & 0x03) == 0x01;


    // xaxis_dir  = PINB & (1 << PINB4); 
    // yaxis_dir  = PINB & (1 << PINB4); 
    // yaxis_dir  = PINB & (1 << PINB4); 
    // zaxis_dir  = PINB & (1 << PINB4); 
    // zaxis_dir  = PINB & (1 << PINB4); 


    if( xaxis_step )
    {
        PORTA = 0x01;          
    }else{
        PORTA = 0x00;        
    }

}




int main (void)
{

    DDRA = 0xff; // PORTA all output

    DDRB = 0x00;  // PORTB all input
    PORTB = 0xff; // PORTB pull all pins up  

    DDRD = 0xf0; // PORTB lower half bits input 

    while (1)
    {
 
        read_state();  
        //_delay_ms(200); //wait 350 miliseconds, notice we are including <util/delay.h> above to get this
    
    }


} 





