/*
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef	_FASTIO_ARDUINO_H
#define	_FASTIO_ARDUINO_H

//#include <avr/io.h>

/*
  utility functions
*/

#ifndef MASK
/// MASKING- returns \f$2^PIN\f$
#define MASK(PIN)  (1 << PIN)
#endif

/*
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define _READ(IO) 0//((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))
/// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.

#define _WRITE_NC(IO, v)  do{}while(0)//do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do{}while(0)//do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                       else {\
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }\
                                         CRITICAL_SECTION_END; \
                                       }\
                                     }\
                                     while (0)

#define _WRITE(IO, v)  do{}while(0)//do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

/// toggle a pin
#define _TOGGLE(IO)  do{}while(0)//do {DIO ##  IO ## _RPORT = MASK(DIO ## IO ## _PIN); } while (0)

/// set pin as input
#define	_SET_INPUT(IO) do{}while(0)//do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
/// set pin as output
#define	_SET_OUTPUT(IO) do{}while(0)//do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)

/// check if pin is an input
#define	_GET_INPUT(IO)  0//((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define	_GET_OUTPUT(IO)  0//((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

/// check if pin is an timer
#define	_GET_TIMER(IO)  0//((DIO ## IO ## _PWM)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE(IO, v)  _WRITE(IO, v)

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
