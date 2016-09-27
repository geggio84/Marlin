/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


#ifndef AT90USB
// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define RX_BUFFER_SIZE 128

#define MYSERIAL MSerial

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print(x,y))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(x))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.print(x),MYSERIAL.write_ser('\n'))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(x),MYSERIAL.write_ser('\n'))

extern int serial_file;

const char errormagic[] ="Error:";
const char echomagic[] ="echo:";
#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(((const char *)(name)),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);
//Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
	write(serial_file,str,strlen(str));
}

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};

extern ring_buffer rx_buffer;

class MarlinSerial //: public Stream
{

  public:
    MarlinSerial();
    void begin(long,char*);
    int peek(void);
    int read_buf(void);
    void flush(void);
    
    FORCE_INLINE int available(void)
    {
        int c;  
        if (read(serial_file,&c,1)>0){
                if (c != EOF)
                {
                        int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;
                        //printf("%c",c);
                        //write(serial_file,&c,1);
                        // if we should be storing the received character into the location
                        // just before the tail (meaning that the head would advance to the
                        // current location of the tail), we're about to overflow the buffer
                        // and so we don't write the character or advance the head.
                        if (i != rx_buffer.tail) {
                                rx_buffer.buffer[rx_buffer.head] = c;
                                rx_buffer.head = i;
                        }
                        return ((unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE);
                }
                else return -1;
        }
        return ((unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE);
    }
    
    FORCE_INLINE void write_ser(uint8_t c)
    {
      write(serial_file,&c,1);
    }

    private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);
    
    
  public:
    
    FORCE_INLINE void write_ser(const char *str)
    {
		write(serial_file,str,strlen(str));
    }


    FORCE_INLINE void write_ser(const uint8_t *buffer, size_t size)
    {
		write(serial_file,buffer,size);
    }

    FORCE_INLINE void print(const char *str)
    {
      write_ser(str);
    }
    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    void println(const char &s);
    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
	void close_serial(void);
};

extern MarlinSerial MSerial;
#endif // !AT90USB

// Use the UART for BT in AT90USB configurations
#if defined(AT90USB) && defined (BTENABLED)
   extern HardwareSerial bt;
#endif

#endif
