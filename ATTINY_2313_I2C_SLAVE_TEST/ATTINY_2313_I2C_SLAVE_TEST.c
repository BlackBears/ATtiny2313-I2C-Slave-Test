/*
 * ATTINY_2313_I2C_SLAVE_TEST.c
 *
 * Created: 11/28/2011 9:07:52 AM
 *  Author: Owner
 */ 

/***********************************************************************+/
/| ATTiny2313 setup diagram	                                         |/
 |																								|
 |					---------	--																|
 |		Vcc ----|1		  20|----Vcc														|
 |	   HADDR0--|2		  19|----SCL														|
 |		HADDR1--|3		  18|----NC														|
 |	   NC------|4		  17|----SDA														|
 |		NC------|5		  16|----NC														|
 |		HADDR2--|6		  15|----NC														|
 |		1s sig--|7		  14|----NC														|
 |		NC------|8		  13|----NC														|
 |		NC------|9		  12|----BURN												   |
 |		GND-----|10		  11|----NC														|
 |				  ------------																|
 |																								|
 | Where HADDR0-2 is the hardware address that is read by the code at	   |
 |	startup.																					|
 |	BURN is the burn trigger.  This pin is set to logic level 1 to begin |
 |	the burn process, typically triggering a relay to complete the burn	|
 |	circuit.
 |	1s sig is a 1 second pulse meant only for debugging.  It will be		|
 |	disabled in production code.													   |
/************************************************************************/

//	Default clock speed
/*
 *	The internal oscillator of the ATTiny2313 runs at 8 MHz.  If the CKDIV8
 *	fuse is set, the system clock is prescaled by 8; therefore, we are setting
 *	the F_CPU at 1 MHz
 */
#define F_CPU 1000000UL
#define DEBUG 0

#include <util/delay.h>
#include <avr/io.h>	
#include <avr/sfr_defs.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

#define NOP asm("nop");				//	skip one clock cycle

#ifndef SDA 
#define SDA             (1<<PB5)    //  I2C SDA
#endif

#ifndef SCL 
#define SCL             (1<<PB7)    //  I2C SCL
#endif

#ifndef BURN_TRIGGER
#define BURN_TRIGGER		(1<<PB0)		//	When this pin is high, the burn is turned on
#endif

#define DEFAULT_BURN_DURATION	2

//
//	OPCODES FOR OUR VIRTUAL DEVICE
//

#define I2C_INITIATE_BURN	0x10	//	stage 1 of the sequence
#define I2C_CONFIRM_BURN	0x20	//	stage 2 of the sequence; followed by secret code
#define I2C_CANCEL_BURN		0x30	//	cancel the process
#define I2C_SET_BURN_DURATION 0x40	//	followed by duration in seconds
#define I2C_CONFIRM_BURN_SECRET_CODE 0xCC	// this is the code that must be passed after I2C_CONFIRM_BURN
#define I2C_ACKNOWLEDGE	0x7F	//	acknowledgment sent back to host after successfully confirming burn

//
//	FUNCTION PROTOTYPES
//
void initTimer();
uint8_t hardwareAddress();
void beginBurn();

enum {
	MODE_DEFAULT,
	MODE_INITIATED,
	MODE_BURN
};
typedef uint8_t AKDBurnMode;

//
//	GLOBALS
//
uint16_t second_count;
AKDBurnMode _burn_mode;
uint8_t _burn_duration;

int main(void)
{
	_burn_mode = MODE_DEFAULT;
	_burn_duration = DEFAULT_BURN_DURATION;
	second_count = 0;
		
	//	setup Timer/Counter1 (16bit) which we will use for intervals
	initTimer();
	
	//	setup PORTD data direction (PIND0-2 are the hardware address)
	DDRD = 0b11111000;
	//	PB0 is the burn trigger; so set the data direction register
	DDRB |= BURN_TRIGGER;
	//	we must not trigger a burn right now (don't want to drop the payload before takeoff!)
	PORTB &= ~BURN_TRIGGER;
	//	obtain I2C address at PIND0-2
	uint8_t slave_address = hardwareAddress();
  	// initialize as slave with our hardware address
	usiTwiSlaveInit( slave_address );
	
  	// enable interrupts (must be there, i2c needs them!)
  	sei();
  	// handle commands via I2C bus
  	while (1)
  	{
		  //	check if data is in the i2c receive buffer
		  if( usiTwiDataInReceiveBuffer() )
		  {
			  //	the first byte in the stream is our opcode
			  uint8_t b = usiTwiReceiveByte();
			  _delay_ms(25);
			  if( b == I2C_INITIATE_BURN )
			  {
				  //	if request is to initiate burn, only initiate if we are in default mode
				  //	otherwise, for safety, we drop back to default mode
				  _burn_mode = (_burn_mode == MODE_DEFAULT)?MODE_INITIATED:MODE_DEFAULT;				  
			  }	
			  else if( b == I2C_CANCEL_BURN )
			  {
				  //	if the request is to cancel, always drop back to default mode
				  _burn_mode = MODE_DEFAULT;
				  PORTD &= ~BURN_TRIGGER;
			  }
			  else if( b == I2C_CONFIRM_BURN )
			  {
				  //	if the request is to confirm, look for a second byte that has the 
				  //	confirmation code.
				  uint8_t confirm_byte = usiTwiReceiveByte();
				  if( confirm_byte == I2C_CONFIRM_BURN_SECRET_CODE )
				  {
					  usiTwiTransmitByte(I2C_ACKNOWLEDGE);
					  _delay_ms(10);
					  _burn_mode = MODE_BURN;
					  beginBurn();
				  }					  					  
			  }	
			  else if( b == I2C_SET_BURN_DURATION )
			  {
				  //	if the request if to set the burn duration, then look for the duration
				  //	in seconds in the next byte
				  uint8_t duration_byte = usiTwiReceiveByte();
				  _burn_duration = duration_byte;
			  }			
		 }  
		 //	waste a cycle  	 
		 NOP
  	}
  	return 0;
}

//
//	Initiate the timer/counter
//	
//	We are using TIMER/COUNTER 1 in CTC mode
//	Target Timer Count = (Input Frequency / Prescale) / Target Frequency - 1
// or, (10^6/64/1)-1 = 15624
//
void initTimer()
{
	 // Configure timer 1 for CTC mode
   TCCR1B |= (1 << WGM12);
   // Enable CTC interrupt 
   TIMSK |= (1 << OCIE1A); 

   //  Enable global interrupts 
   sei(); 

   // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
   OCR1A   = 15624; 
   // Start timer at Fcpu/64
   TCCR1B |= ((1 << CS10) | (1 << CS11)); 
}
//	
//	Reads the hardware address of the device
//
//	The hardware address is set at PD0-2
//
uint8_t hardwareAddress() {
	return PIND & ~0b11111000;
}

void beginBurn() 
{
	PORTB |= BURN_TRIGGER;
}

ISR(TIMER1_COMPA_vect)
{
	//	pulse the PD3 pin every second for testing purposes
	if( DEBUG ) {
		PORTD ^= (1<<PD3);
	}
	//	don't increment second count if we're not burning
	if( _burn_mode == MODE_BURN )
	{
		second_count++;
		//	if we reach the end of the burn cycle, then turn off the relay
		if( second_count >= _burn_duration )
		{
			second_count = 0;
			_burn_mode = MODE_DEFAULT;
			PORTB &= ~BURN_TRIGGER;
		}
	}
}


/*

#include <avr/io.h>
#include <avr/interrupt.h>

int main (void)
{
   DDRB |= (1 << 0); // Set LED as output

   TIMSK |= (1 << TOIE1); // Enable overflow interrupt
   sei(); // Enable global interrupts

   TCNT1 = 49911; // Preload timer with precalculated value

   TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at Fcpu/64

   for (;;)
   {

   }
}

ISR(TIMER1_OVF_vect)
{
   PORTB ^= (1 << 0); // Toggle the LED
   TCNT1  = 49911; // Reload timer with precalculated value
} 
*/