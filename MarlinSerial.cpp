/*
  HardwareSerial.cpp - Hardware serial library for Wiring
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
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include "Marlin.h"
#include "MarlinSerial.h"

ring_buffer rx_buffer  =  { { 0 }, 0, 0 };

// Constructors ////////////////////////////////////////////////////////////////

MarlinSerial::MarlinSerial()
{

}

// Public Methods //////////////////////////////////////////////////////////////

void MarlinSerial::begin(long baud, char* serial)
{
    int serial_baud;
    struct termios serial_config;
    
        // OPEN Serial Port
		if (serial_file != 0) {
			printf("Serial Port File already open: %s\n\r", serial);
			return;
		}
			
        //printf("Now We Open Serial Port File: %s\n\r", serial);
        serial_file = open(serial, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_file == 0) {
                printf("Failed to open file %s\n\r",serial);
                marlin_kill();
        }
        if(!isatty(serial_file)) {
                printf("Serial file %s is NOT a TTY\n\r",serial);
                marlin_kill();
        }
        // Set serial port baudrate
        if(tcgetattr(serial_file, &serial_config) < 0){
                printf("Failed to get serial attribute\n\r");
                marlin_kill();
        }
        switch (baud)
        {
                case 500000:
                        serial_baud = B500000;
                        break;
                case 460800:
                        serial_baud  = B460800;
                        break;
                case 230400:
                        serial_baud  = B230400;
                        break;
                case 115200:
                default:
                        serial_baud  = B115200;
                        break;
                case 57600:
                        serial_baud  = B57600;
                        break;
                case 38400:
                        serial_baud  = B38400;
                        break;
                case 19200:
                        serial_baud  = B19200;
                        break;
                case 9600:
                        serial_baud  = B9600;
                        break;
        }
        memset(&serial_config,0,sizeof(serial_config));
        serial_config.c_iflag=0;
        serial_config.c_oflag=0;
        serial_config.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        serial_config.c_lflag=0;
        serial_config.c_cc[VMIN]=1;
        serial_config.c_cc[VTIME]=5;
        if(cfsetispeed(&serial_config, serial_baud) < 0 || cfsetospeed(&serial_config, serial_baud) < 0) {
                printf("Failed to set serial baudrate\n\r");
                marlin_kill();
        }

        if(tcsetattr(serial_file, TCSANOW, &serial_config) < 0){
                printf("Failed to set serial attribute\n\r");
                marlin_kill();
        }
}

int MarlinSerial::peek(void)
{
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MarlinSerial::read_buf(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ((rx_buffer.head == rx_buffer.tail) && (MYSERIAL.available() <= 0)) {
		return 0;
  } else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

void MarlinSerial::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  rx_buffer.head = rx_buffer.tail;
}




/// imports from print.h




void MarlinSerial::print(char c, int base)
{
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base)
{
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base)
{
  if (base == 0) {
    write_ser(n);
  } else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  } else {
    printNumber(n, base);
  }
}

void MarlinSerial::print(unsigned long n, int base)
{
  if (base == 0) write_ser(n);
  else printNumber(n, base);
}

void MarlinSerial::print(double n, int digits)
{
  printFloat(n, digits);
}

void MarlinSerial::println()
{
  print("\r\n");
}

void MarlinSerial::println(const char &s)
{
  print(s);
  println();
}

void MarlinSerial::println(const char c[])
{
  print(c);
  println();
}

void MarlinSerial::println(char c, int base)
{
  print(c, base);
  println();
}

void MarlinSerial::println(unsigned char b, int base)
{
  print(b, base);
  println();
}

void MarlinSerial::println(int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(double n, int digits)
{
  print(n, digits);
  println();
}

void MarlinSerial::close_serial(void)
{
    if (serial_file != 0) {
        printf("Now We Close Serial Port File: %s\n\r", SERIAL_PORT);
        close(serial_file);
    }
}

// Private Methods /////////////////////////////////////////////////////////////

void MarlinSerial::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}
// Preinstantiate Objects //////////////////////////////////////////////////////

MarlinSerial MSerial;


