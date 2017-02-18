/* Simple Serial UART 2 USB for Teensy USB Development Board
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include "uart.h"
#include "usb_serial.h"

#define BAUD_RATE 115200

#define LED_CONFIG	(DDRD |= (1<<6))                       /* Configure Led on Pin D6 */
#define LED_ON		(PORTD |= (1<<6))                        /* Turn Led On */
#define LED_OFF		(PORTD &= ~(1<<6))                       /* Turn Led Off */
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))        /* Set CPU clock speed */
#define uart_printf(s) uart_print_P(PSTR(s))               /* write a string to the uart */
#define usb_printf(s) usb_print_p(PSTR(s))                 /* write a string to the usb */

void usb_print_p(const char *s);
void uart_print_p(const char *str);

/* Send a string to the USB serial port. The string must be in
   flash memory, using PSTR */
void usb_print_p(const char *s) {
    char c;
    while (1) {
        c = pgm_read_byte(s++);
        if (!c) break;
        usb_serial_putchar(c);
    }
}

/* Send a string to the UART serial port. */
void uart_print_P(const char *str) {
    char c;
    while (1) {
        c = pgm_read_byte(str++);
        if (!c) break;
        uart_putchar(c);
    }
}

/* The entry for our App code */
int main(void) {
    uint8_t n;

    /* set for 16 MHz clock, and turn on the LED */
    CPU_PRESCALE(0);
    LED_CONFIG;
    LED_ON;

    /* initialize the USB, and then wait for the host
	     to set configuration.  If the Teensy is powered
	     without a PC connected to the USB port, this 
	     will wait forever. */
    usb_init();
    while (!usb_configured()) /* wait */ ;
    _delay_ms(1000);

    /* Main loop with reconection */
    while (1) {
		    /* wait for the user to run their terminal emulator program
		       which sets DTR to indicate it is ready to receive. */
		    while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

		    /* discard anything that was received prior.  Sometimes the
		       operating system or other software will send a modem
		       "AT command", which can still be buffered. */
		    usb_serial_flush_input();

        uart_init(BAUD_RATE);   /* Initialize UART on given baud rate */
		    
		    usb_printf("\r\nTeensy Serial UART <> USB, \r\nCopyright (c) 2008 PJRC.COM, LLC\r\n");  /* print a nice welcome message */
 
	      LED_OFF; /* turn off the led now to indicate we are done and ready for action */
	      
	      /* Start UART <> USB communication and loop over */
        while (1) { 
		        /* Check if Terminal is still connected. If not, we turn on the LED, break the loop and wait for a new connection */
		        if (!(usb_serial_get_control() & USB_SERIAL_DTR)) { LED_ON; break; }
		    
		        if (uart_available()) {                /* Do we have something on the UART channel ? */
			          LED_ON;                            /* Turn led on to indicate action */
			          while (uart_available()) {         /* Write uart output to usb input */
			              n = uart_getchar();
			              usb_serial_putchar(n);
			          }
			          LED_OFF;                           /* Turn led off, we are done */
		        }

			      if (usb_serial_available()) {           /* Do we have something on the USB channel ? */
			          LED_ON;                             /* Turn led on to indicate action */
			          while (usb_serial_available()) {   /* Write usb output (user input) to uart input */
			              n = usb_serial_getchar();
		                uart_putchar(n);
			          }
		            LED_OFF;                            /* Turn led off, we are done */
		        }
		    }
	  }
}
