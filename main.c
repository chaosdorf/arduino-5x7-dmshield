/*
 * main.c
 *
 */ 

/**********************************************************************************

Title:				5x7 DotMatrix Shield for ATMega328P

Hardware:			5x7-Segment DotMatrix Display Shield connected to an
					ATMega328p running at 8 MHz
Author:				Frank Andre, adjustments by Daniel Friesel
License:			This software is distributed under the creative commons license
					CC-BY-NC-SA.
Disclaimer:			This software is provided by the copyright holder "as is" and any 
					express or implied warranties, including, but not limited to, the 
					implied warranties of merchantability and fitness for a particular 
					purpose are disclaimed. In no event shall the copyright owner or 
					contributors be liable for any direct, indirect, incidental, 
					special, exemplary, or consequential damages (including, but not 
					limited to, procurement of substitute goods or services; loss of 
					use, data, or profits; or business interruption) however caused 
					and on any theory of liability, whether in contract, strict 
					liability, or tort (including negligence or otherwise) arising 
					in any way out of the use of this software, even if advised of 
					the possibility of such damage.
					
**********************************************************************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "config.h"
#include "dot_matrix.h"
#include "animations.h"


/*********
* fuses *
*********/

FUSES =
{
	.low = 0xE2,
	.high = 0xD9,
	.extended = 0xFF,
};


/********************
 * global variables *
 ********************/

uint8_t scroll_speed = 8;					// scrolling speed (0 = fastest)
volatile uint8_t button = PB_ACK;			// button event
//uint8_t* msg_ptr = (uint8_t*) messages;		// pointer to next message in EEPROM
uint8_t* msg_ptr;							// pointer to next message in EEPROM
uint8_t* ee_write_ptr = (uint8_t*) messages;


/**********
 * macros *
 **********/

// Usage: b=swap(a) or b=swap(b)
#define swap(x)													\
	({															\
		unsigned char __x__ = (unsigned char) x;				\
		asm volatile ("swap %0" : "=r" (__x__) : "0" (__x__));	\
		__x__;													\
	})


/*************
 * functions *
 *************/

/*======================================================================
	Function:		InitHardware
	Input:			none
	Output:			none
	Description:	.
======================================================================*/
void InitHardware(void)
{
	// switch all pins that are connected to the dot matrix to output
	DDRB = DISP_MASK_B;
	DDRC = DISP_MASK_C;
	DDRD = DISP_MASK_D;
	
	// enable pull-ups on all input pins to avoid floating inputs
	PORTB |= ~DISP_MASK_B;
	PORTC |= ~DISP_MASK_C;
	PORTD |= ~DISP_MASK_D;
	
	// timer 0
	TCCR0A = 0;				// timer mode = normal
	TCCR0B = _BV(CS02) | _BV(CS00);					// prescaler = 1:1024
	OCR0A = OCR0A_CYCLE_TIME;
	OCR0B = OCR0B_CYCLE_TIME;
	TIMSK0 = _BV(OCIE0B) | _BV(OCIE0A);
	
}


/*======================================================================
	Function:		SetMode
	Input:			mode byte
	Output:			none
	Description:	Set display parameters.
					The mode byte is interpreted as follows:
					Bit 7:		reverse scrolling direction (0 = no, always scroll forward, 1 = yes, bidirectional scrolling)
					Bit 6..4:	delay between scrolling repetitions (0 = shortest, 7 = longest)
					Bit 3:		scrolling increment (cleared = +1 (for texts), set = +5 (for animations))
					Bit 2..0:	scrolling speed (1 = slowest, 7 = fastest)
======================================================================*/
void SetMode(uint8_t mode)
{
	uint8_t inc, dir, dly, spd;

	if (mode & 0x08)	{ inc = 5; }
		else			{ inc = 1; }
	if (mode & 0x80)	{ dir = BIDIRECTIONAL; }
		else			{ dir = FORWARD; }
	spd = mode & 0x07;
	dly = swap(mode) & 0x07;
	dmSetScrolling(inc, dir, pgm_read_byte(&dly_conv[dly]));
	scroll_speed = pgm_read_byte(&spd_conv[spd]);
}		


/*======================================================================
	Function:		DisplayMessage
	Input:			pointer to zero terminated message data in EEPROM memory
	Output:			pointer to next message
	Description:	Show a message (i. e. text or animation) on the display.

					Escape characters:

					Character '^' is used to access special characters by 
					shifting the character code of the next character by 96 
					so that e. g. '^A' becomes char(161).
					To enter a '^' character simply double it: '^^'

					Character '~' followed by an upper case letter is used
					to insert (animation) data from flash.
					
					The character 0xFF is used to enter direct mode in which 
					the following bytes are directly written to the display 
					memory without being decoded using the character font.
					Direct mode is ended by 0xFF.
======================================================================*/
uint8_t* DisplayMessage(uint8_t* ee_adr)
{
	uint8_t ch;

	SetMode(eeprom_read_byte(ee_adr));
	ee_adr++;
	dmClearDisplay();

	ch = eeprom_read_byte(ee_adr++);
	while (ch) {
		if (ch == '~') {					// animation
			ch = eeprom_read_byte(ee_adr++);
			if (ch != '~') {
				ch -= 'A';
				if (ch < ANIMATION_COUNT) {
					dmDisplayImage((const uint8_t*)pgm_read_word(&animation[ch]));
				}				
			}
		}
		else if (ch == 0xFF) {				// direct mode
			ch = eeprom_read_byte(ee_adr++);
			while (ch != 0xFF) {
				dmPrintByte(ch);
				ch = eeprom_read_byte(ee_adr++);
			}
		}
		else {								// character
			if (ch == '^') {				// special character
				ch = eeprom_read_byte(ee_adr++);
				if (ch != '^') {
					ch += 63;
				}
			}
			dmPrintChar(ch);
		}
		ch = eeprom_read_byte(ee_adr++);
		if (ch) { dmPrintByte(0); }			// print a narrow space except for the last character					
	}
	ch = eeprom_read_byte(ee_adr);			// read mode byte of next message
	if (ch)		{ return(ee_adr); }
		else	{ return((uint8_t*) messages); }	// restart all-over if mode byte is 0
}


/*======================================================================
	Function:		GoToSleep
	Input:			none
	Output:			none
	Description:	Put the controller into sleep mode and prepare for
					wake-up by a pin change interrupt.
======================================================================*/
void GoToSleep(void)
{
	dmClearDisplay();
	_delay_ms(1000);
	PCIFR |= _BV(PCIF2);				// clear interrupt flag
	PCMSK2 = _BV(PCINT16);			// enable pin change interrupt
	PCICR  = _BV(PCIE2);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
	PCICR = 0;
	dmPrintChar(131);				// happy smiley
	_delay_ms(500);
	msg_ptr = DisplayMessage((uint8_t*) messages);
}


/********
 * main *
 ********/

int main(void)
{
	InitHardware();
	dmInit();
	sei();									// enable interrupts

	ADCSRA = 0;

	GoToSleep();
	dmPrintChar(131);				// happy smiley
	button |= PB_ACK;

	while(1)
	{
		if (button == PB_RELEASE) {			// short button press
			msg_ptr = DisplayMessage(msg_ptr);
			button |= PB_ACK;
		}
		
		if (button == PB_LONGPRESS) {		// button pressed for some seconds
			dmClearDisplay();
			dmPrintChar(130);				// sad smiley
			_delay_ms(500);
			GoToSleep();
			button |= PB_ACK;
		}
		
	} // of while(1)
}


/******************************
 * interrupt service routines *
 ******************************/

ISR(TIMER0_COMPA_vect)
// display interrupt
{
	OCR0A += OCR0A_CYCLE_TIME;				// setup next cycle

	dmDisplay();							// show next column on dot matrix display
}


ISR(TIMER0_COMPB_vect)
// system timer interrupt
{
	static uint8_t scroll_timer = 1;
	static uint8_t pb_timer = 0;			// push button timer
	uint8_t temp;
		
	OCR0B += OCR0B_CYCLE_TIME;				// setup next cycle


	if (scroll_timer) {
		scroll_timer--;
	}
	else {
		scroll_timer = scroll_speed;		// restart timer
		dmScroll();							// do a scrolling step
	}
	
	// push button sampling
	temp = ~PB_PIN;							// sample push button
	temp &= PB_MASK;						// extract push button state
	if (temp == 0) {						// --- button not pressed ---
		if (button & PB_PRESS) {			// former state = pressed?
			button &= ~(PB_PRESS | PB_ACK);	// -> issue release event
		}
	}
	else {									// --- button pressed ---
		if ((button & PB_PRESS) == 0) {		// former state = button released?
			button = PB_PRESS;				// issue new press event
			pb_timer = PB_LONGPRESS_DELAY;	// start push button timer
		}
		else {
			if (button == PB_PRESS) {		// holding key pressed
				if (pb_timer == 0) {		// if push button timer has elapsed
					button = PB_LONGPRESS;	// issue long event
				}
				else {
					pb_timer--;
				}
			}			
		}		
	}
	
}


ISR(PCINT2_vect)
// pin change interrupt (for wake-up)
{
}



/*
ISR(TIMER1_COMPA_vect)
{
}


ISR(TIMER1_COMPB_vect)
{
}
*/
