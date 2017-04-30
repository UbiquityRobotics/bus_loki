// Copyright (c) 2014 by Wayne C. Gramlich.  All rights reserved.

// See "Bus.h" for for an overview.

#include <Arduino.h>
#include <Bus_Slave.h>

// *UART* routines:

void UART::integer_print(Integer integer) {
  UByte digits[10];
  UByte index = 0;

  // Deal with *negative* numbers:
  Logical is_negative = (Logical)0;
  if (integer < 0) {
    integer = -integer;
    is_negative = (Logical)1;
  }

  // Repeatibly eivide *integer* by 10 until it reaches 0; stuff each digit
  // into *digits*:
  while (integer != 0) {
    digits[index++] = integer % 10;
    integer = integer / 10;
  }

  // Make sure we have at least one digit in *digits*:
  if (index == 0) {
    digits[index++] = 0;
  }

  // Output any negative sign:
  if (is_negative) {
    frame_put((UShort)'-');
  }

  // Output *digits* in reverse order:
  while (index-- > 0) {
    frame_put((UShort)(digits[index] + '0'));
  }
}

void UART::string_print(Text text) {
  Character character = '\0';
  while ((character = *text++) != '\0') {
    frame_put((UShort)character);
  }
}

void UART::uinteger_print(UInteger uinteger) {
  // If *uinteger* is 0, just output "0":
  if (uinteger == 0) {
    // *uinteger* is 0:
    frame_put((UShort)'0');
  } else {
    // *uinteger* is non-zero; use *non_zero_output* to suppress leading zeros:
    Logical non_zero_output = (Logical)0;

    // Make *shift* iterate through 28, 24, 20, ... , 8 , 4, 0:
    UByte shift = 32;
    do {
      shift -= 4;

      // Use *shift* to extract the a *nibble* from left to right
      // across *uinteger*:
      UByte nibble = (uinteger >> shift) & 0xf;

      // Check zero nibbles for leading zero suppression:
      if (nibble == 0) {
	if (non_zero_output) {
	  frame_put((UShort)'0');
	}
      } else {
	frame_put((UShort)"0123456789abcdef"[nibble]);
	non_zero_output = (Logical)1;
      }
    } while (shift != 0);
  }
}

// *AVR_UART* routines:

AVR_UART::AVR_UART(volatile UByte *ubrrh, volatile UByte *ubrrl,
 volatile UByte *ucsra, volatile UByte *ucsrb, volatile UByte *ucsrc,
 volatile UByte *udr) {
  // First hang onto all of the register addresses:
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _ucsrc = ucsrc;
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _udr = udr;

  reset();
}

void AVR_UART::reset() {
  // Clear the UART configuration registers:
  *_ubrrh = 0;
  *_ubrrl = 0;
  *_ucsra = 0;
  *_ucsrb = 0;
  *_ucsrc = 0;

  // Intialize the UART data structures;
  _get_head = 0;
  _get_tail = 0;
  _interrupt = (Logical)0;
  _put_head = 0;
  _put_tail = 0;
}

void AVR_UART::end() {
  reset();
}

void AVR_UART::begin(UInteger frequency,
  UInteger baud_rate, Character *configuration) {
  // We want to do the following to the USART:
  //  - Set transmit/receive rate to .5Mbps
  //  - Single rate transmission (Register A)
  //  - All interrupts disabled for now (Register B)
  //  - Transmit enable (Register B)
  //  - Receive enable (Register B)
  //  - Turn on Receiver interrupt enable flag (Register B)
  //  - 9-bits of data (Registers B and C)
  //  - Asynchronous mode (Register C)
  //  - No parity (Register C)
  //  - 1 Stop bit (Register C)
  //
  // There are a total of 5 registers to deal with.
  //   UBRRnH = USART Baud Rate Register (High byte)
  //   UBRRnL = USART Baud Rate Register (low byte)
  //   UCSRnA = USART Control status register A
  //   UCSRnB = USART Control status register B
  //   UCSRnC = USART Control status register C
  // They will be initialized in the order above.

  // Hang onto *configuration* for debugging purposes:
  _configuration = configuration;

  // Initialize USART baud rate to .5Mbps:
  UInteger ubrr = frequency / (baud_rate * 8) - 1;
  *_ubrrh = (UByte)(ubrr >> 8);
  *_ubrrl = (UByte)ubrr;

  // UCSR0A: USART Control and Status Register 0 A:
  //   rtef opdm: (0010 0000: Default):
  //      r: Receive complete
  //      t: Transmit complete
  //      e: data register Empty
  //      f: Frame error
  //      o: data Overrun
  //      p: Parity error
  //      d: Double transmission speed
  //      m: Multi-processor communication mode
  // Only d and m can be set:  We want d=1 and m=0:
  //   rtef opdm = 0000 0000 = 0
  UByte a_flags = _BV(U2X0);

  // UCSR0B: USART Control and Status Register 0 B:
  //   rtea bcde: (0000 0000: Default):
  //      r: Receive complete interrupt enable
  //      t: Transmit complete interrupt enable
  //      e: data register Empty complete interrupt enable
  //      a: (A) Transmitter enable
  //      b: (B) Receiver enable
  //      c: (C) size bit 2 (see register C):
  //      d: Receive data bit 8
  //      e: Transmit data bit 8
  // All bits except d can be set.  We want:
  //  r=1   (i.e. receive enable)
  //  t=1   (i.e. transmit enable)
  // rtea bcde = 0001 1000 = 0x18:
  UByte b_flags = _BV(TXEN0) | _BV(RXEN0);

  // UCSR0C: USART Control and Status Register 0 C:
  //   mmpp szzp: (0000 0110):
  //      mm: USART Mode
  //          00 Asynchronous USART
  //          01 Synchronous USART
  //          10 reserved
  //          11 Master SPI
  //      pp: Parity mode
  //          00 Disabled
  //          01 reserved
  //          10 enabled, even parity
  //          11 enabled, odd parity
  //      s: Stop bit (0=1 stop bit, 1=2 stop bits)
  //      czz: Character size (include C bit from register B):
  //          000 5-bit
  //          001 6-bit
  //          010 7-bit
  //          011 8-bit
  //          10x reserved
  //          110 reserved
  //          111 9-bit
  //      p: Clock polarity (synchronous mode only)
  // All bits can set.  We want:
  //    mm=00 (i.e. Asynchronous USART)
  //    pp=00 (i.e.  None)
  //    s = 0 (i.e. 1 stop bit)
  //    p = 0 (i.e. don't care)
  // The zz bits are set from the *size_bits* bits below.  Thus, for now,
  //   mmpp szzp = 0000 0000 == 0:
  UByte c_flags = 0;

  // This is where we deal with *configuration*.  Right now we really
  // only support a *configuration* value of "8N1" or "9N1".  Other,
  // possiblities could be supported, but for now we only bother with
  // "8N1" and "9N1":

  // Use the 1st character of *configuration* to get the either 8-bit
  // or 9-bit mode:
  UByte size_bits = 0b000;	// = 0000 0czz
  if (configuration[0] == '8') {
    // 8 bit mode (czz = 011):
    size_bits = 0b011;
  } else {
    // 9-bit mode (czz = 111):
    size_bits = 0b111;
  }

  // Merge *size_flags* into *b_flags* and *c_flags*:
  b_flags |= size_bits & 0b100;		// ---- -c--
  c_flags |= (size_bits & 0b11) << 1;	// ---- -zz-

  // Now set the flags for UCSRnA, UCSRnB, and UCSRnC:
  *_ucsra = a_flags;
  *_ucsrb = b_flags;
  *_ucsrc = c_flags;
}

Logical AVR_UART::can_receive() {
  Logical result = (Logical)0;
  if (_interrupt) {
    result = (Logical)(_get_tail != _get_head);
  } else {
    result = (Logical)((*_ucsra & _BV(RXC0)) != 0);
  }
  return result;
}

Logical AVR_UART::can_transmit() {
  Logical result = (Logical)0;
  if (_interrupt) {
    result = (Logical)(((_put_head + 1) & _ring_mask) != _put_tail);
  } else {
    result = (Logical)((*_ucsra & _BV(UDRE0)) != 0);
  }
  return result;
}

UShort AVR_UART::frame_get() {
  // Wait for a 9-bit frame to arrive and return it:

  UShort frame = 0;

  // Set to 1 to use interrupt buffers; 0 for direct UART access:
  if (_interrupt) {
    // When tail is equal to head, there are no frames in ring buffer:
    while (_get_tail == _get_head) {
      // Wait for a frame to show up.
    }

    // Read the {frame} and advance the tail by one:
    frame = _get_ring[_get_tail++];
    _get_tail &= _ring_mask;
  } else {
    while (!(*_ucsra & _BV(RXC0))) {
      // Nothing yet, keep waiting:
    }
    if ((*_ucsrb & _BV(RXB80)) != 0) {
      frame = 0x100;
    }
    frame |= (UShort)(*_udr);
  }
  return frame;
}

void AVR_UART::frame_put(UShort frame) {
  // This routine will output the low order 9-bits of {frame} to {self}.
  // The echo due to the fact the bus is half-duplex is automatically
  // read and ignored.

  // For *interrupt*'s, we fetch from the buffer; otherwise we directly
  // read from the register:
  if (_interrupt) {
    while (((_put_head + 1) & _ring_mask) == _put_tail) {
      // Wait for space to show up in put ring buffer:
    }

    UByte put_head = _put_head;
    _put_ring[put_head] = frame;
    _put_head = (put_head + 1) & _ring_mask;

    *_ucsrb |= _BV(UDRIE0);
  } else {
    // Wait until UART can take another character to output:
    while ((*_ucsra & _BV(UDRE0)) == 0) {
      // UART is still busy, keep waiting:
    }
  
    // Set 9th bit:
    *_ucsrb &= ~_BV(TXB80);
    if ((frame & 0x100) != 0) {
      *_ucsrb |= _BV(TXB80);
    }

    // Output the lower 8 bits:
    *_udr = (UByte)frame;
  }
}

void AVR_UART::interrupt_set(Logical interrupt) {
  _interrupt = interrupt;
  if (interrupt) {
    *_ucsrb |= _BV(UDRIE0) | _BV(RXCIE0);
  } else {
    *_ucsrb &= ~(_BV(UDRIE0) | _BV(RXCIE0));
  }
}

void AVR_UART::receive_interrupt()
{
  // This is the interrupt service routine that is called to when there
  // is a new frame (i.e. 9-bit value) from the Maker Bus USART.  This
  // routine reads the 9-bit value and stuffs into the {_get_ring} buffer.

  // Read the data out; 9th bit is in *ucsrb,* remaining 8-bits are in *udr*:
  UShort frame = 0;
  if ((*_ucsrb & _BV(RXB80)) != 0) {
    frame = 0x100;
  }
  frame |= (UShort)(*_udr);

  // We need to insert {frame} into {_get_ring}.  If the {ring}
  // is full we drop {frame} on the floor:
  UByte get_head = _get_head;
  UByte next_get_head = (get_head + 1) & _ring_mask;

  // Is {_get_ring} full?:
  if (next_get_head != _get_tail) {
    // No, {_get_ring} has some space for at least one more *frame*:

    // Stuff {frame} into {_get_ring}, and bump {_get_head} forward:
    _get_ring[get_head] = frame;
    _get_head = next_get_head;

    // For now we stay out of Multi-Processor Mode:
    //UCSR1A &= ~_BV(MPCM1);
  }
}

void AVR_UART::transmit_interrupt() {
  // This is the interrupt service routine that is invoked when the
  // transmit frame buffer is ready for another 9-bit frame.  The
  // frame is removed from {_put_buffer}.  If there are no frames
  // ready for transmission, the interrupt is disabled.

  //avr_uart0.frame_put((UShort)'m');
  //avr_uart0.string_print(_configuration);
  //avr_uart0.uinteger_print((UInteger)_put_head);

  // The transmit buffer is ready for a new frame.  Now check to
  // see if we have a frame to feed it:
  UByte put_tail = _put_tail;
  if (_put_head == put_tail) {
    // *put_buffer* is empty, so disable transmit interrupts:
    //avr_uart0.frame_put((UShort)'n');
    *_ucsrb &= ~_BV(UDRIE0);
  } else {
    // *put_buffer* is not empty; grab *frame* from *put_buffer* and
    // update *put_tail*:
    //avr_uart0.frame_put((UShort)'o');
    UShort frame = _put_ring[put_tail];
    _put_tail = (put_tail + 1) & _ring_mask;

    // Deal with 9th bit of *frame*.  Most of the time the 9th bit
    // is not set.  So we clear the 9th bit by default and then set
    // it if necssary:
    //avr_uart0.frame_put((UShort)'p');
    *_ucsrb &= ~_BV(TXB80);
    if ((frame & 0x100) != 0) {
      // Set 9th bit:
      *_ucsrb |= _BV(TXB80);
    }

    //avr_uart0.frame_put((UShort)'q');
    // Stuff the low order 8-bits into *udr* to send the 9-bit frame
    // on its way.
    *_udr = (UByte)frame;
  }
  //avr_uart0.frame_put((UShort)'r');
}


// We define the one only only *avr_uart0* object here along with the
// its two associated interrupt routines only if the *UDR0* symbol exists:
// This code only compiles and works if the associated code from
// SerialHardware.cpp is disabled; otherwise, there will be a conflict
// defining two interrupt service routines:

#if defined(UDR0)
  // *UDR0* is defined so we create the *avr_uart0* object and
  // the two interrupt service routines:

  // This is the only instance of an *AVR_UART0* object:
  AVR_UART0 avr_uart0;

  // The receive interrupt is sent off to the shared
  // *AVR_UART::receive_interrupt*() routine:
  #if defined(USART_RX_vect)
    ISR(USART_RX_vect) {
  #elif defined(USART0_RX_vect)
    ISR(USART0_RX_vect) {
  #elif defined(USART0_RXC_vect)
    ISR(USART0_RXC_vect) {
  #else
    #error "None of {USART_RX,USART0_RX,USART0_RXC}_vect is defined."
  #endif
      avr_uart0.receive_interrupt();
    }

  // The transmit interrupt is sent off to the shared
  // *AVR_UART::transmit_interrupt*() routine:

  #if defined(UART_UDRE_vect)
    ISR(UART_UDRE_vect) {
  #elif defined(USART_UDRE_vect)
    ISR(USART_UDRE_vect) {
  #elif defined(UART0_UDRE_vect)
    ISR(UART0_UDRE_vect) {
  #elif defined(USART0_UDRE_vect)
    ISR(USART0_UDRE_vect) {
  #else
    #error "None of {UART,USART,UART0,USART0}_URDE_vect is defined."
  #endif
      avr_uart0.transmit_interrupt();
    }

#endif // defined(UDR0)

// *AVR_UART1* routines:

// We define the one only only *avr_uart1* object here along with the
// its two associated interrupt routines only if the *UDR1* symbol exists:
// This code only compiles and works if the associated code from
// SerialHardware.cpp is disabled; otherwise, there will be a conflict
// defining two interrupt service routines:

#if defined(UDR1)
  // *UDR0* is defined so we create the *avr_uart0* object and
  // the two interrupt service routines:

  // This is the only instance of an *AVR_UART1* object:
  AVR_UART1 avr_uart1;

  // The receive interrupt is sent off to the shared
  // *AVR_UART::receive_interrupt*() routine:
  ISR(USART1_RX_vect) {
    //avr_uart0.frame_put((UShort)'[');
    avr_uart1.receive_interrupt();
    //avr_uart0.frame_put((UShort)']');
  }

  // The transmit interrupt is sent off to the shared
  // *AVR_UART::transmit_interrupt*() routine:
  ISR(USART1_UDRE_vect) {
    //avr_uart0.frame_put((UShort)'<');
    avr_uart1.transmit_interrupt();
    //avr_uart0.frame_put((UShort)'>');
  }

#endif // define(UDR1)

// *Bus_Buffer* routines:

Bus_Buffer::Bus_Buffer() {
  // Initialize the buffer indices and count.

  reset();
  //show('i');
}

void Bus_Buffer::reset() {
  _get_index = 0;
  _put_index = 0;
  _error_flags = 0;
}

UByte Bus_Buffer::check_sum() {
  // This routine will return the 4-bit checksum of the first {count}
  // bytes in {buffer}.

  UByte check_sum = 0;
  for (UByte index = 0; index < _put_index; index++) {
    check_sum += _ubytes[index];
    //debug_character('S');
    //debug_hex(ubyte);
  }
  return (check_sum + (check_sum >> 4)) & 0xf;
}

void Bus_Buffer::show(UByte tag) {
  //debug_character('<');
  //debug_character(tag);
  //debug_character(':');
  //debug_character('p');
  //debug_hex(_put_index);
  //debug_character('g');
  //debug_hex(_get_index);
  //debug_character('>');
}

UByte Bus_Buffer::ubyte_get() {
  // This routine will return the next byte from the buffer.

  UByte ubyte = 0;
  if (_get_index >= _put_index) {
    // We are attempting to read past the end of the buffer:
    _error_flags += 1;
  } else {
    ubyte = _ubytes[_get_index++];
  }
  return ubyte;
}

void Bus_Buffer::ubyte_put(UByte ubyte) {
  // This routine will return the next byte from the buffer.

  UByte put_index = _put_index;
  if (put_index >= _ubytes_size) {
    // Attempting to write past end of buffer:    
    _error_flags += 1;
  } else {
    _ubytes[_put_index++] = ubyte;
  }
}

UInteger Bus_Buffer::uinteger_get() {
  //...

  UByte byte3 = ubyte_get();
  UByte byte2 = ubyte_get();
  UByte byte1 = ubyte_get();
  UByte byte0 = ubyte_get();
  UInteger uinteger =
    (((UInteger)byte3) << 24) |
    (((UInteger)byte2) << 16) |
    (((UInteger)byte1) <<  8) |
     ((UInteger)byte0);
  return uinteger;
}

void Bus_Buffer::uinteger_put(UInteger uinteger) {
  // ...

  ubyte_put((UByte)(uinteger >> 24));
  ubyte_put((UByte)(uinteger >> 16));
  ubyte_put((UByte)(uinteger >> 8));
  ubyte_put((UByte)(uinteger));
}

UShort Bus_Buffer::ushort_get() {
  // This routine will return the next short from the buffer.

  UByte high_byte = ubyte_get();
  UByte low_byte = ubyte_get();
  UShort ushort = (((UShort)high_byte) << 8) | ((UShort)low_byte);
  return ushort;
}

void Bus_Buffer::ushort_put(UShort ushort) {
  // This routine will return the next short from the buffer.

  ubyte_put((UByte)(ushort >> 8));
  ubyte_put((UByte)ushort);
}

// *Bus_Slave* routines:

Bus_Slave::Bus_Slave(UART *bus_uart, UART *debug_uart) {
  // Initialize some the private member values:
  _bus_uart = bus_uart;
  _debug_uart = debug_uart;
  _desired_address = (UShort)0;
  _current_address = (UShort)0xffff;
  _auto_flush = (Logical)1;
}

// *Bus_Slave::command_begin*() will queue up *command* to be sent to the
// module at *address*.  The number of bytes to be sent after *command*
// is *put_bytes*.  If there is inadequate space in the send buffer
// for *command* and the following *put_bytes* of data, any previous
// command(s) are flushed first.

void Bus_Slave::command_begin(UByte address, UByte command, UByte put_bytes) {
    // For debugging, commands get enclosed in '{' ... '}':
    debug_character('{');

    // Force a flush if the address is
    UShort full_address = (UShort)address | 0x100;
    if (_desired_address != full_address) {
	flush();
	_desired_address = full_address;
    }

    // If command will not fit into *remaining* bytes, force a flush:
    UByte remaining = _maximum_request_size - _put_buffer._put_index;
    if (1 + put_bytes > remaining) {
	flush();
    }

    // Stuff the *command* byte into to the *put_buffer*:
    ubyte_put(command);
}

void Bus_Slave::command_end() {
  // This command indicates that the current command is done.

  // When *auto_flush* is enabled, we force a flush at the
  // end of each command:
  if (_auto_flush) {
    flush();
  }

  // Close off '}' for debugging:
  debug_character('}');
  debug_character('\r');
  debug_character('\n');
}

UByte Bus_Slave::command_ubyte_get(UByte address, UByte command) {
  // For debugging:
  //debug_text((Text)"cmd_ubyte_get(");
  //debug_hex((UInteger)address);
  //debug_text((Text)",");
  //debug_hex((UInteger)command);
  //debug_text((Text)")");

  command_begin(address, command, 0);
  command_end();
  flush();
  UByte ubyte = ubyte_get();

  // For debugging:
  //debug_text((Text)"=>");
  //debug_hex((UInteger)ubyte);
  //debug_text((Text)"\r\n");

  return ubyte;
}

void Bus_Slave::command_ubyte_put(UByte address, UByte command, UByte ubyte) {
  command_begin(address, command, sizeof(UByte));
  ubyte_put(ubyte);
  command_end();
}

UShort Bus_Slave::command_ushort_get(UByte address, UByte command) {
  // For debugging:
  //debug_text((Text)"cmd_ubyte_get(");
  //debug_hex((UInteger)address);
  //debug_text((Text)",");
  //debug_hex((UInteger)command);
  //debug_text((Text)")");

  command_begin(address, command, 0);
  command_end();
  flush();
  UShort ushort = ushort_get();

  // For debugging:
  //debug_text((Text)"=>");
  //debug_hex((UInteger)ushort);
  //debug_text((Text)"\r\n");

  return ushort;
}

void Bus_Slave::command_ushort_put(
 UByte address, UByte command, UShort ushort) {
  command_begin(address, command, sizeof(UShort));
  ushort_put(ushort);
  command_end();
}

// Flush current buffer and get any response back:
Logical Bus_Slave::flush() {
  debug_character('!');
  Logical error = (Logical)0;

  // See if there is anything to flush:
  UByte request_size = _put_buffer._put_index;
  if (request_size > 0) {

    // For now force the *desired_address* out:
   _current_address = (UShort)0xffff;

    // Keep sending out an address until we succeed:
    while (_current_address != _desired_address) {
      // Make sure that there is nothing left in receive queue:
      while (can_receive()) {
	(void)frame_get();
      }

      // Send the *desired_address*:
      frame_put(_desired_address);
      UShort address_echo = frame_get();
      if (_desired_address == address_echo) {
	// Deal with addresses that are in the lower half:
	if ((_desired_address & 0x80) == 0) {
	  // Wait for an acknowledgement byte:
	  UShort acknowledge_frame = frame_get();
	  if (acknowledge_frame == 0) {
	    // Success: we have set the *current_address*:
	    _current_address = _desired_address;
	  } else {
	    debug_character('?');
	  }
	} else {
	  // Success: we have set the *current_address*:
	  _current_address = _desired_address;
	}
      } else {
	debug_character('@');
      }
    }
    // assert (_current_address == _desired_address);

    // Send the *request_header*:
    UByte request_size = _put_buffer._put_index;
    UByte request_check_sum = _put_buffer.check_sum();
    UByte request_ubyte = (request_size << 4) | request_check_sum;
    UShort request_frame = (UShort)request_ubyte;    
    frame_put(request_frame);
    UByte request_echo = frame_get();
    if (request_frame != request_echo) {
      debug_character('#');
    }

    // Send the request data followed by reseting the buffer:
    UByte index;
    for (index = 0; index < request_size; index++) {
      UShort ubyte_frame = (UShort)_put_buffer.ubyte_get();
      frame_put(ubyte_frame);
      UShort ubyte_echo = frame_get();
      if (ubyte_frame != ubyte_echo) {
	debug_character('$');
      }
    }
    _put_buffer.reset();

    // Wait for the response packet header:
    UByte response_header = (UByte)frame_get();

    // Process *response_header*:
    UByte response_length = response_header >> 4;
    if (response_length == 0) {
      // A *response_length* of 0 indicates an error:
      error = (Logical)1;
    } else {
      // Now slurp in the rest of response packet:
      _get_buffer.reset();
      for (index = 0; index < response_length; index++) {
        UByte ubyte = (UByte)frame_get();
	_get_buffer.ubyte_put(ubyte);
      }

      // For debugging:
      //debug_text((Text)" GB.gi=");
      //debug_hex((UInteger)_get_buffer._get_index);
      //debug_text((Text)" GB.pi=");
      //debug_hex((UInteger)_get_buffer._put_index);
      //debug_text((Text)" GB.buf[0]=");
      //debug_hex((UInteger)_get_buffer._ubytes[0]);
      //debug_text((Text)" ");

      // Check for a check sum error:
      UByte response_check_sum = response_header & 0xf;
      if (_get_buffer.check_sum() != response_check_sum) {
	debug_character('#');
	error = (Logical)1;
      }
    }
  }
  debug_character('!');
  return error;
}

UInteger Bus_Slave::command_integer_get(UByte address, UByte command) {
  command_begin(address, command, 0);
  command_end();
  flush();
  Integer integer = (Integer)uinteger_get();
  return integer;
}

void Bus_Slave::command_integer_put(UByte address, UByte command, Integer integer) {
  command_begin(address, command, sizeof(Integer));
  uinteger_put((UInteger)integer);
  command_end();
}


// This routine will perform all the operations to respond to
// commands sent to *address*.  *command_process* is a routine that
// is called to process each command in the received request packet.
// If *execute_mode*, the command is to be executed; otherwise the
// command is only parsed for correctness.  The return value
// from *command_process* is zero for success and non-zero for
// for failure.

static Logical selected = (Logical)0;
static UByte request_size = 0;
static UByte request_check_sum = 0;
static UByte selected_address = 0xff;

void Bus_Slave::slave_mode(UByte address,
 UByte (*command_process)(Bus_Slave *bus_slave,
 UByte command, Logical execute_mode)) {
  debug_character('[');
  if (can_receive()) {
    // Fetch the next frame from the UART:
    UShort frame = frame_get();

    // Dispatch on 9th bit:
    if ((frame & 0x100) != 0) {
      // We have an address frame:
      //debug_character('J');
      selected_address = (UByte)frame;
      selected = (Logical)(selected_address == address);
      if (selected) {
	// We have been selected:
	//debug_character('K');
	selected = (Logical)1;
	if ((address & 0x80) == 0) {
	  // We need to send an acknowledge
	  //debug_character('L');
	  frame_put(0x0);
	  //debug_character('M');
	  UShort acknowledge_echo = frame_get();
	  if (acknowledge_echo != 0) {
	    debug_character('!');
	  }
	}
	_get_buffer.reset();
	_put_buffer.reset();
      }
      debug_character('N');

      // We are starting over:
      request_size = 0;
    } else if (selected) {
      // We have a data frame:
      UByte data = (UByte)frame;

      if (request_size == 0) {
	// Process header request:
	//debug_character('m');
	request_size = (data >> 4) & 0xf;
	request_check_sum = data & 0xf;

	_get_buffer.reset();
      } else {
	// Take {data} byte and stuff it onto end of {_get_buffer}:
	_get_buffer.ubyte_put(data);

	// Do we have a complete request?
	if (_get_buffer._put_index >= request_size) {
	  // Yes, we have a complete request:

	  // Check that *request_check_sum* matches *check_sum*:
	  UByte check_sum = _get_buffer.check_sum();
	  //debug_character('o');
	  //debug_hex(check_sum);
	  if (check_sum == request_check_sum) {
	    // The check sums match; now iterate over all the
	    // commands and make sure that they parse correctly.
	    // We pass over the request bytes twice.  The first
	    // time ({pass} == 0), we just make sure that the
	    // commands and associated arguments "make sense".
	    // The second time ({pass} == 1), we actually perform
	    // the commands:
	    UByte flags;
	    UByte pass;

	    // Pass over request bytes two times:
	    for (pass = 0; pass < 2; pass++) {
	      // Now iterate over all the command sequences
	      // in {_get_buffer}:
	      flags = 0;
	      _get_buffer._get_index = 0;
	      _get_buffer._error_flags = 0;
	      while (_get_buffer._get_index < request_size) {
		UByte command = ubyte_get();
		flags |= command_process(this, command, (Logical)pass);
	      }
	      //debug_character('r');

	      // Make sure we detect errors from trying to
	      // read too many bytes from request:
	      flags |= _get_buffer._error_flags;

	      // If there are any errors, we generate an error
	      // response and do not perform the second pass:
	      _get_buffer._error_flags = 0;
	      if (flags != 0) {
		debug_character('q');
		// We have a parse error:
		break;
	      }
	      //debug_character('s');
	    }

	    //debug_character('v');
	    // Did we have any errors:
	    if (flags == 0) {
	      // Time to pump out a response packet:

	      // Compute {response_header} and output it:
	      UByte response_size = _put_buffer._put_index;
	      UByte response_check_sum = _put_buffer.check_sum();
	      UByte header_byte = (response_size << 4) | response_check_sum;
	      UShort header_frame = (UShort)(header_byte);
	      frame_put(header_frame);
	      UShort header_echo = frame_get();
	      if (header_frame != header_echo) {
		debug_character('/');
	      }
	      //debug_character('y');

	      for (UByte index = 0; index < response_size; index++) {
		UShort ubyte_frame = (UShort)_put_buffer.ubyte_get();
		frame_put(ubyte_frame);
		UShort ubyte_echo = frame_get();
		if (ubyte_frame != ubyte_echo) {
		  debug_character(';');
		}
	      }
	      _put_buffer.reset();
	    } else {
	      // We had at least one error; kick out an error:
	      //debug_character('z');
	      debug_hex(flags);
	      frame_put((UShort)0x03);
	    }
	  } else {
	    // The checksums do match; respond with an error byte:
	    //debug_character('w');
	    //debug_hex(request_checksum);
	    //debug_character('x');
	    //debug_hex(checksum);
	    frame_put((UShort)0x01);
	  }

	  request_size = 0;
	  debug_character(']');
	  debug_character('\r');
	  debug_character('\n');
	  debug_character('[');
	}
      }
    }
  }
}

void Bus_Module::bind(Bus_Slave *bus_slave, UByte address)
{
  _bus_slave = bus_slave;
  _address = address;
}

