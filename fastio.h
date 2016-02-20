/*
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef	_FASTIO_ARDUINO_H
#define	_FASTIO_ARDUINO_H

#include "gpio_lib.h"

/*
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define _READ(IO) get_gpio_value(IO)
/// write to a pin
#define _WRITE(IO, v) set_gpio_value(IO, (PIN_VALUE)v)

/// toggle a pin
#define _TOGGLE(IO) set_gpio_value(IO, (PIN_VALUE)(get_gpio_value(IO)? 0:1))

/// set pin as input
#define	_SET_INPUT(IO) set_gpio_input(IO)
/// set pin as output
#define	_SET_OUTPUT(IO) set_gpio_output(IO, (PIN_VALUE)get_gpio_value(IO))

/// check if pin is an input
#define	_GET_INPUT(IO) 0//((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define	_GET_OUTPUT(IO)  0//((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

/// check if pin is an timer
#define	_GET_TIMER(IO)  0//((DIO ## IO ## _PWM)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE(IO, v)  _WRITE(IO, (PIN_VALUE)v)

/// toggle a pin wrapper
#define TOGGLE(IO)  _TOGGLE(IO)

/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// set pin as output wrapper
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define GET_INPUT(IO)  _GET_INPUT(IO)
/// check if pin is an output wrapper
#define GET_OUTPUT(IO)  _GET_OUTPUT(IO)

/// check if pin is an timer wrapper
#define GET_TIMER(IO)  _GET_TIMER(IO)

#endif /* _FASTIO_ARDUINO_H */
