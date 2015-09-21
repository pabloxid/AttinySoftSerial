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


//
// Includes
// 
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "Arduino.h"
#include "SoftSerial.h"
#include "UserTimer.h"


// Statics

SoftSerial *SoftSerial::active_object = 0;
char SoftSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftSerial::_receive_buffer_head = 0;
volatile boolean SoftSerial::busy = false;


// Private methods

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftSerial::listen()
{
  if (active_object != this)
  { 
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    set_rx_interrupt ();     // el usuario debe desactivar las otras interrupciones
    SREG = oldSREG;
    return true;
  }
  return false;
}

// The receive routine called by the interrupt handler
// modificación para operar con timer interrupts en lugar de "tunedDelay"

void SoftSerial::recv (boolean ext) {                                          // static
  static enum {START, HALFBIT, DATABIT, STOP, IDLE} estado = IDLE;
  static byte data;
  static byte index;

  if (ext) {
    // If RX line is high, then we don't see any start bit
    // so interrupt is probably not for us
    if (estado == IDLE && (active_object->_inverse_logic ? active_object->rx_pin_read() : !active_object->rx_pin_read())) {          
      busy = true;
      active_object->clear_rx_interrupt ();    
      // timer on
      TIFR |= MASK3(OCF0A,OCF0B,TOV0);        // provisorio (hay que buscarle una sintaxis) (borra las flags)
      UserTimer_SetCount( 0 );
      UserTimer_EnableOutputCompareInterruptA();
      estado = START;
      // delay halfbit
    }
  } else {
    switch (estado) {
      case START:
        // vuelve a comprobar el bit de arranque
        if (active_object->_inverse_logic ? active_object->rx_pin_read() : !active_object->rx_pin_read()) {
          index = 1;
          data = 0;
          estado = HALFBIT;
          // delay halfbit
        } else {
          UserTimer_InterruptsOff();
          active_object->set_rx_interrupt ();
          estado = IDLE;
          busy = false;
        }  
        break;
      case HALFBIT:
        if (index == 0) {estado = STOP;} else {estado = DATABIT;}
        // delay halfbit
        break;
      case DATABIT: { 
        byte noti = ~index;
        if (active_object->rx_pin_read()) {
          data |= index;
        } else {   // else clause added to ensure function timing is ~balanced
          data &= noti;
        }
        index <<= 1;
        estado = HALFBIT;
        // delay halfbit
        break;
      }
      case STOP:
        if (active_object->_inverse_logic) {data = ~data;}
        // If RX line is low, then we don't see any stop bit
        if (active_object->_inverse_logic ? !active_object->rx_pin_read() : active_object->rx_pin_read()) {
          if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) {
            // save new data in buffer: tail points to where byte goes
            _receive_buffer[_receive_buffer_tail] = data; // save new byte
            _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF; 
          } else {
            //_buffer_overflow
          }
        }
        // deshabilita timer
        UserTimer_InterruptsOff();
        active_object->set_rx_interrupt ();
        estado = IDLE;
        busy = false;   
        break;  
    }
  }
}

uint8_t SoftSerial::rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

// Interrupt handling

extern volatile boolean isSleeping;          // 

// El ATtiny85 tiene una sola interrupción externa
ISR (PCINT0_vect) {
  if (isSleeping) {return;}
  if (SoftSerial::active_object) {
    SoftSerial::recv (true);
  }
}

// timer compare match interrupt
ISR( USERTIMER_(COMPA_vect) ) {
  if (SoftSerial::active_object) {
    SoftSerial::recv (false);
  }
}
  
    /////////////////
    // Constructor //
    /////////////////

SoftSerial::SoftSerial (uint8_t receivePin, bool inverse_logic /* = false */) : 
  _rx_delay_halfbit (0),
  _inverse_logic (inverse_logic)
{
  setRX (receivePin);
}


// Destructor
SoftSerial::~SoftSerial() {clear_rx_interrupt ();}

void SoftSerial::setRX (uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask (rx);
  uint8_t port = digitalPinToPort (rx);
  _receivePortRegister = portInputRegister (port);
}


// Public methods

void SoftSerial::begin (long speed)
{
  // configuración del timer
  UserTimer_SetToPowerup();
  UserTimer_SetWaveformGenerationMode( UserTimer_(CTC_OCR) );
  UserTimer_(DisconnectOutputs);
  /// UserTimer_ClockSelect( UserTimer_(Stopped) );          /// ojo esto es para debug solamente
   
  // cálculo del prescaler
  if ((_rx_delay_halfbit = F_CPU/(speed*2*USERTIMER_(PRESCALER_VALUE_1))) < 256) {
    UserTimer_ClockSelect( UserTimer_(Prescale_Index_1) );
  } else if ((_rx_delay_halfbit = F_CPU/(speed*2*USERTIMER_(PRESCALER_VALUE_2))) < 256) {
    UserTimer_ClockSelect( UserTimer_(Prescale_Index_2) );
  } else if ((_rx_delay_halfbit = F_CPU/(speed*2*USERTIMER_(PRESCALER_VALUE_3))) < 256) {
    UserTimer_ClockSelect( UserTimer_(Prescale_Index_3) );
  } else if ((_rx_delay_halfbit = F_CPU/(speed*2*USERTIMER_(PRESCALER_VALUE_4))) < 256) {
    UserTimer_ClockSelect( UserTimer_(Prescale_Index_4) );
  } else {
    _rx_delay_halfbit = 0;
  }  
  
  UserTimer_SetOutputCompareMatchAndClear (_rx_delay_halfbit);   // halfbit
  
  // habilita el pin change interrupt en forma general (si el baudrate es válido)
  if (_rx_delay_halfbit) {
    if (digitalPinToPCICR(_receivePin)) {
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    }
  }
    
  // listen();  
}

void SoftSerial::clear_rx_interrupt () {
  GIFR |= _BV(PCIF);                          // clear flag
  if (digitalPinToPCICR(_receivePin)) {
    //*digitalPinToPCICR(_receivePin) &= ~_BV(digitalPinToPCICRbit(_receivePin));  // esto no va
    *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
  }
}

void SoftSerial::set_rx_interrupt () {
  if (digitalPinToPCICR(_receivePin)) {
    //*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));  // esto no va
    *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
  }
}

// Read data from buffer
int SoftSerial::read()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftSerial::available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

