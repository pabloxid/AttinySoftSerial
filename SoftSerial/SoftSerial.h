/*
SoftSerial.h  
Multi-instance, receive-only, timer-driven software serial library for Arduino Tiny environment
by Pablo Gindel (www.pablogindel.com)

Based on previous SoftwareSerial library by ladyada, Mikal Hart, Paul Stoffregen et al.

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

*/

#ifndef SoftSerial_h
#define SoftSerial_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF   64 // RX buffer size

class SoftSerial /*: public Stream*/
{
private:
  // per object data
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  
  uint16_t _rx_delay_halfbit;
  
  uint16_t _inverse_logic:1;

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  
  // private methods
  uint8_t rx_pin_read();
  void setRX (uint8_t receivePin);

public:
  // public methods
  SoftSerial (uint8_t receivePin, bool inverse_logic = false);
  ~SoftSerial ();
  void begin( long speed );
  bool listen ();
    
  void set_rx_interrupt ();
  void clear_rx_interrupt (); 

  static int read();
  static int available();
  
  // public only for easy access by interrupt handler
  static SoftSerial *active_object;
  static void recv (boolean ext);
  static volatile boolean busy;
  
};

#endif
