#include <avr/io.h>

#define F_CPU 16000000UL //AVR Clock Speed in MHZ
#define FOSC 16000000    // Clock Speed

#include <util/delay.h>
#include "nes_ctrlr.h"

#define BitSet(Port,Bit) (Port|=(1<<Bit))
#define BitClear(Port,Bit) (Port&=~(1<<Bit))
#define BitToggle(Port,Bit) (Port^=(1<<Bit))

#define SetBits(Port,BitMask) (Port|=BitMask)
#define ClearBits(Port,BitMask) (Port&=~BitMask)
#define ToggleBits(Port,BitMask) (Port^=BitMask)


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
uint8_t yaxis_step = 0; 
uint8_t zaxis_dir  = 0; 
uint8_t zaxis_step = 0; 


//use these to AND all outputs together on each cycle
uint8_t ouput_porta = 0; 
uint8_t ouput_portc = 0;


//----------------------------------------// 

//void knightrider(uint8_t *outa)

void knightrider(void)
{
    uint8_t i = 0; 
    

    for(uint8_t i = 0; i<7; i++)  
    {           
        //*outa =i ;//|= (1<<i);
        PORTA |= (1<<i);
        _delay_ms(run_del); 

        //*outa &= ~(1<<i);
        PORTA &= ~(1<<i);        
        _delay_ms(run_del);  
    } 

     
    for(i=7; i>0; i--)
    {
        PORTA |= (1<<i);
        _delay_ms(run_del);
        PORTA &= ~(1<<i);
        _delay_ms(run_del);
    } 
   

}

//----------------------------------------// 
void set_x_dir(uint8_t xdir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (xdir==0x00)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_porta |=  (1 << PA1); //pin 1 high        
        ouput_porta &= ~(1 << PA2); //pin 2 low 

        // // pulse the enable line 
        // _delay_ms(run_del);
        // ouput_porta |= (1 << PA0);    
        // _delay_ms(run_del);
        // ouput_porta &= ~(1 << PA0);    
    }
    
    if (xdir==0x01)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_porta |=  (1 << PA2); //pin 1 low        
        ouput_porta &= ~(1 << PA1); //pin 2 high 
    }

}

//----------------------------------------// 
void set_y_dir(uint8_t ydir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (ydir==0x00)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_porta |=  (1 << PA4); //pin 1 high        
        ouput_porta &= ~(1 << PA5); //pin 2 low 
    }
    
    if (ydir==0x01)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_porta |=  (1 << PA5); //pin 1 low        
        ouput_porta &= ~(1 << PA4); //pin 2 high 
    }

}

//----------------------------------------// 
void set_z_dir(uint8_t zdir){

    // PORTA -- 0 XEN | 1 XDIRLO | 2 XDIRHI | 3 YEN | 4 YDIRLO | 5 YDIRHI    
    // PORTC -- 0 ZEN | 1 ZDIRLO | 2 ZDIRHI     

    if (zdir==0)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_portc |=  (1 << PA1); //pin 1 high        
        ouput_portc &= ~(1 << PA2); //pin 2 low 

    }
    
    if (zdir==1)
    {
        // set hi-low bridge controls (never both the same!) 
        ouput_portc |=  (1 << PA2); //pin 1 low        
        ouput_portc &= ~(1 << PA1); //pin 2 high 
    
    }

}


//----------------------------------------// 
void step_x_axis(void)
{
    // STEPX (pulse the enable line )
    //ouput_porta |= (1 << PA0);   
    BitSet(ouput_porta,0);
    //_delay_ms(run_del);

    //ouput_porta &= ~(1 << PA0); 
    //BitClear(PORTA,0);
    //_delay_ms(run_del);

}

//----------------------------------------// 
void step_y_axis(void)
{
    // STEPY (pulse the enable line )
    //_delay_ms(run_del);
    ouput_porta |= (1 << PA3);      
    //_delay_ms(run_del);
    //PORTA &= ~(1 << PA3); 
}

//----------------------------------------// 
void step_z_axis(void)
{
    // STEPZ (pulse the enable line )
    //_delay_ms(run_del);
    ouput_portc |= (1 << PA0);      
    //_delay_ms(run_del);
    //PORTC &= ~(1 << PA0);
}
 

//----------------------------------------// 
void update(void){
    //set the final output per cycle  
    PORTA = ouput_porta;
    PORTC = ouput_portc;
    _delay_ms(run_del);

    //clear the step pulses in sync
    BitClear(PORTA,0);
    PORTA &= ~(1 << PA3); 
    PORTC &= ~(1 << PA0);
    _delay_ms(run_del);

}

//----------------------------------------//
void test(void){

    set_x_dir(0);
    set_y_dir(0);
    set_z_dir(0);        
    _delay_ms(run_del);

    for (uint8_t xx=0;xx<10;xx++){
       step_x_axis();
       step_y_axis();
       step_z_axis();
       update();
    }    
  
    set_x_dir(1);
    set_y_dir(1);
    set_z_dir(1);
    _delay_ms(run_del);

    for (uint8_t xx=0;xx<10;xx++){
        step_x_axis();
        step_y_axis();
        step_z_axis();
        update();
    }  

}
 

//----------------------------------------//
void nes_play(void)
{

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
    
    //down
    if(nes_byte == 0xFB ){   
        set_x_dir(0);
        step_x_axis();
        update();
    } 
    //up
    if(nes_byte == 0xF7 ){   
        set_x_dir(1);
        step_x_axis();
        update();        
    } 

    //right
    if(nes_byte == 0xFE ){   
        set_y_dir(0);
        step_y_axis();
        update();
    } 
    //left
    if(nes_byte == 0xFD ){   
        set_y_dir(1);
        step_y_axis();
        update();        
    } 
   

    //B button
    if(nes_byte == 0xBF ){   
        set_z_dir(0);
        step_z_axis();
        update();
    } 
    //A button
    if(nes_byte == 0x7F ){   
        set_z_dir(1);
        step_z_axis();
        update();        
    } 


    //idle - no stepping 
    if(nes_byte == 0xFF ){   
        ouput_porta = 0x00;
        ouput_portc = 0x00;

    }

    
    //set_z_dir(0);        

}


//----------------------------------------// 
void glu_logik(void)
{
    /*
      example to read the state of PORTB and set PORTA as output  

      # PIN MAPPING FOR CABLE TO 7i76  

      0 pb0 step0-
      1 pb1 step0+ 
      2 pb2 dir0- 
      3 pb3 dir0+  //(ignore)  
      4 pb4 step1- 
      5 pb5 step1+ 
      6 pb6 dir1- 
      7 pb7 dir1+ //(ignore) 
      
      0 pd0 step2- 
      1 pd1 step2+ 
      2 pd2 dir2- 
      3 pd2 dir2+     

    */

    // this is difficult - it cant pulse the steps as fast as it can read

    uint8_t bit0 = (PINB >> 0) & 1; //x step -   
    uint8_t bit1 = (PINB >> 1) & 1; //x step + 
    uint8_t bit2 = (PINB >> 2) & 1; //x dir  -  
    //      bit3 = (PINB >> 3) & 1; //ignore differental dir+  
    uint8_t bit4 = (PINB >> 4) & 1; //y step -   
    uint8_t bit5 = (PINB >> 5) & 1; //y step +  
    uint8_t bit6 = (PINB >> 6) & 1; //y dir  -   

    uint8_t bit7 = (PIND >> 0) & 1; //z step -  
    uint8_t bit8 = (PIND >> 1) & 1; //z step +
    uint8_t bit9 = (PIND >> 2) & 1; //z dir  -

    // Initialise global variables in case of reset
    ouput_porta = 0;
    ouput_portc = 0;

    //-------
    // dir -X 
    if (bit2==0x01)
    {
        set_x_dir(0);
    } else{
        set_x_dir(1);
    } 

    //-------
    // dir -Y  
    if (bit6==0x01)
    {
        set_y_dir(0);
    } else{
        set_y_dir(1);
    } 
    
    /*
    //-------
    // dir -Z 
    if (bit9==0x01)
    {
        set_z_dir(0);
    } else{
        set_z_dir(1);
    }  
    */

    //-------------------//

    if (bit0==0x00 && bit1==0x01)
    {
        step_x_axis();
    } 
    if (bit4==0x00 && bit5==0x01)
    {
        step_y_axis();
    } 
    if (bit7==0x00 && bit8==0x01)
    {
        step_z_axis();
    } 

    //attempt to stop the thing when no signal
    if (bit0== 0 && bit1==0 || bit4==0 && bit5==0 || bit4==0 && bit5==0 )
    {
        ouput_porta = 0x00;
        ouput_portc = 0x00;
    }


}

//----------------------------------------// 

int main (void)
{

    DDRA = 0xff; // PORTA all output
    DDRC = 0xff; // PORTC all output

    DDRB = 0x00;  // PORTB all input
    DDRD = 0x00;  // PORTB all input 

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


        //read incoming - set step and dir bits 
        //glu_logik();
        //update();

        //test();
        nes_play();
    
     
             
    }

    return 0;
} 










