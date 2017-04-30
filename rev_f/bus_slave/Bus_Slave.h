// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_SLAVE_H_INCLUDED
#define BUS_SLAVE_H_INCLUDED 1

//! @file Bus.h
//! @brief Header file for Bus classes
//!
//! This file defines the classes used for Bus communication
//!
//! First and formost, the file defines a bunch of typedef's for
//! common types.  Yeah, we could use the more standard *uint8_t* crud,
//! but let's admit it, *uint8_t* is really ugly.  Instead we go
//! with simple types that start with one or two capital letters
//!
//!  The integer types are:
//!
//!      Signed      Unsigned       Size
//!  ========================================
//!      Byte        UByte          8-bit
//!      Short       UShort         16-bit
//!      Integer     UInteger       32-bit
//!      Long        ULong          64-bit
//!
//! The remaining types are:
//!
//!      Type        Description
//!  ========================================
//!    Character     8-bit ANSI-C character
//!    Double        64-bit IEEE floating point number
//!    Float         32-bit IEEE floating point number
//!    Text          ANSI-C style null terminated string
//!
//! Now that types have been covered, we can move on...
//!
//! Bus communication takes place using UART's (sometimes called USART's.)
//!
//! We start with an abstract class called *UART*.  This class is
//! sub-classed into *Null_UART* and *AVR_UART*.  The *Null_UART*
//! sub-class is a place holder object that does nothing.  The
//! *Null_UART* is used as a debugging UART when there is no debugging
//! UART available.  The *AVR_UART* sub-class is used to talk to an
//! AVR UART.  There  will typically be an *AVR_UART* object for each
//! available UART on givent hardware.  The *AVR_UART* is sub-classed
//! into *AVR_UART0*, ..., *AVR_UART*n, depending upon how many UART's
//! are available.
//!
//! The *AVR_UART* object support both polled and non-polled operation.
//! The non-polled operation is primarily used when bring software up
//! on a new piece of hardware.  Once it is basically working, interrupts
//! are enabled.
//!
//! There will be at most a single instance of each *AVR_UART*n object
//! for each available UART.  The reason for this is because the two
//! transmit and receive interrupts must linked to specific interrupt
//! vectors.  These UART's have the characteristic that they conflict
//! with the UART code defined in the Arduino SerialHardware.cpp file.
//!
//! Once you have UART's, the *Bus* object is used to communicate
//! with the UART's.  The *Bus* object needs both a *bus_uart* and
//! a *debug_uart*.  The *debug_uart* can be a *Null_UART*.  The
//! *bus_uart* must be a real UART.
//!
//! The *Bus_Buffer* class is a helper class that buffers up data packets
//! for sending and receiving.

// All typedef's go up here before the #includes':

// *BUS_LOG* is pretty stale:
//#define BUS_LOG 0

// Set *BUS_DEBUG* to 1 to enable debug tracing:
#define BUS_DEBUG 0

// Define *BUS_LOKI_AS_DEBUG to 1 to use debug trace output on BUS uart
#define BUS_LOKI_UART1_AS_DEBUG

// Signed types:
typedef signed char Byte;		// 8-bit signed byte (-128 ... 128):
typedef char Character;			// 8-bit character (sign whatever)
typedef double Double;			// 64-bit double precision float number
typedef float Float;			// 32-bit single precision float number
typedef signed long Integer;		// 32-bit signed integer
typedef signed long long int Long;	// 64-bit signed integer
typedef signed short Short;		// 16-bit signed integer
typedef Character *Text;		// Null terminated *Character* string

// Unsigned types start with 'U' character:
typedef unsigned char UByte;		// 8-bits unsigned
typedef unsigned long UInteger;		// 32-bits unsigned
typedef unsigned char Logical;		// 1-bit logical value
typedef unsigned long long int ULong;	// 64-bit unsigned
typedef unsigned short UShort;		// 16-bit unsigned
typedef unsigned short Unicode;		// 16-bit Unicode character

#include "Arduino.h"
#include "Print.h"

class UART {
  public:
    virtual void begin(UInteger frequency,
      UInteger baud_rate, Text configuration) = 0;
    virtual Logical can_transmit() = 0;
    virtual Logical can_receive() = 0;
    virtual UShort frame_get() = 0;
    virtual void frame_put(UShort frame) = 0;
    void integer_print(Integer integer);
    virtual void interrupt_set(Logical interrupt) = 0;
    void print(UByte ubyte) { uinteger_print((UInteger)ubyte); }
    void print(Character character) { frame_put((UShort)character); }
    void print(UShort ushort) { uinteger_print((UInteger)ushort); }
    void print(Text text) { string_print(text); }
    void string_print(Text text);
    void uinteger_print(UInteger uinteger);
};

class AVR_UART : public UART {
  public:
    AVR_UART(volatile UByte *ubrrh, volatile UByte *ubrrl,
     volatile UByte *ucsra, volatile UByte *ucsrb, volatile UByte *ucsrc,
     volatile UByte *udr);
    void begin(UInteger frequency,
     UInteger baud_rate, Text configuration);
    virtual Logical can_receive();
    virtual Logical can_transmit();
    void end();
    virtual UShort frame_get();
    virtual void frame_put(UShort frame);
    virtual void interrupt_set(Logical interrupt);
    void receive_interrupt();
    void reset();
    void transmit_interrupt();
  private:
    UByte static const _ring_power = 4;
    UByte static const _ring_size = 1 << _ring_power;
    UByte static const _ring_mask = _ring_size - 1;
    volatile UByte _get_head;
    UShort _get_ring[_ring_size];
    volatile UByte _get_tail;
    Logical _interrupt;
    volatile UByte _put_head;
    UShort _put_ring[_ring_size];
    volatile UByte _put_tail;
    Character *_configuration;
    UByte volatile *_ubrrh;
    UByte volatile *_ubrrl;
    UByte volatile *_ucsra;
    UByte volatile *_ucsrb;
    UByte volatile *_ucsrc;
    UByte volatile *_udr;
};

class NULL_UART : public UART {
  public:
    virtual void begin(UInteger frequency,
     UInteger baud_rate, Text configuration) { };
    virtual Logical can_receive() { return (Logical)1; };
    virtual Logical can_transmit() { return (Logical)1; };
    virtual UShort frame_get() { return 0xffff; };
    virtual void frame_put(UShort frame) { };
    virtual void interrupt_set(Logical interrupt) { };
};

#if defined(UDR0)
  class AVR_UART0 : public AVR_UART {
    public:
      AVR_UART0() :
       AVR_UART(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0) { } ;
  };

  extern AVR_UART0 avr_uart0;
#endif // defined(UDR0)

#if defined(UDR1)
  class AVR_UART1 : public AVR_UART {
    public:
      AVR_UART1() :
       AVR_UART(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1) { } ;
  };

  extern AVR_UART1 avr_uart1;
#endif // defined(UDR1)

// These defines are empty when *BUS_TRACE* is 0:
//! @class Bus_Buffer
//! @brief Send/Receive packet buffer.
//!
//! This is class to hold bus packets that are sent and received.

class Bus_Buffer
{
  public:				// In alphabetical order:
    Bus_Buffer();			// Constructor
    Byte byte_get() {
      return (Byte)ubyte_get();
    };
    void byte_put(Byte byte) {
      ubyte_put((UByte)byte); 
    };
    Character character_get() {
      return (Character)ubyte_get();
    };
    void character_put(Character character) {
      ubyte_put((UByte)character); 
    };
    UByte check_sum();			// Compute 4-bit check sum
    Integer integer_get() {
      return (Integer)uinteger_get();
    };
    void integer_put(Integer integer) {
      uinteger_put((UInteger)integer); 
    };
    Logical logical_get() {
      return (Logical)ubyte_get();
    };
    void logical_put(Logical logical) {
      ubyte_put((UByte)logical); 
    };
    void reset();			// Reset/clear buffer
    Short short_get() {
      return (Short)ushort_get();
    };
    void short_put(Short xshort) {
      ushort_put((UShort)xshort); 
    };
    void show(UByte Tag);		// Show buffer (for debgging)
    UByte ubyte_get();			// Get next *UByte* from buffer
    void ubyte_put(UByte ubyte);	// Put a *UByte* into buffer
    UInteger uinteger_get();		// Get next *UInteger* from buffer
    void uinteger_put(UInteger uinteger); // Put a *UInteger* into buffer
    UShort ushort_get();		// Get next *UShort* from buffer
    void ushort_put(UShort ushort);	// Put a *UShort* into buffer

    // Any fields with a preceeding underscore, should only be
    // set byt *Bus_Buffer* methods.  Reading the values is OK:
    UByte _error_flags;			// Error flags go here
    UByte _get_index;			// Next byte to get index
    UByte _put_index;			// Next byte to put index
    static const UByte _ubytes_power = 4; // Bufere size must be power of 2
    static const UByte _ubytes_size = 1 << _ubytes_power; // Actual buffer size
    UByte _ubytes[_ubytes_size];	// The actual buffer bytes
};

//! @class Bus
//! @brief managing the Bus UART
//!
//! This a helper class that takes care of the UART that talks to the
//! Bus.

class Bus_Slave
{
  public:
    //! @brief Constructor for Bus object.
    Bus_Slave(UART *bus_uart, UART *debug_uart);

    void auto_flush_set(Logical auto_flush) {
      _auto_flush = auto_flush;
    };

    //! @brief Return the a signed byte from currently selected module.
    //!   @return the next signed byte from the command.
    //!
    //! ...
    Byte byte_get() {
      // Return the next {Byte} from recieve buffer:
      return (Byte)ubyte_get();
    }

    //! @brief Send *byte* to currently selected module.
    //!   @param byte byte value to send to command.
    //!
    //! This member function will queue *byte* to be sent to the
    //! currently selected module as part of a multi-byte command.
    //! *byte* is signed.
    void byte_put(Byte byte) {
      // Queue {byte} to be sent off to bus:
      ubyte_put((UByte)byte);
    }

    Logical character_get() {
      // Return the next {character} from recieve buffer:
      return (Character)ubyte_get();
    }

    //! @brief Send *byte* to currently selected module.
    //!   @param character Character value to send to command.
    //!
    //! This member function will queue *character* to be sent to the
    //! currently selected module as part of a multi-byte command.
    void character_put(Character character) {
      // Queue {character} to be sent off to bus:
      ubyte_put((UByte)character);
    }

    //! @brief Send *command* to the module at *address*.
    //!
    //!   @param address address (0 - 254) of module to connect to
    //!   @param command first byte of command
    //!
    //! This mebmer function will cause the module at *address* to be
    //! selected if it has not already been selected.   A sequence of
    //! 1 or more bytes is sent to the selected module followed by 0,
    //! 1 or more returned bytes.  The final call is *command_end*().
    //! The first byte sent to the selected module is *command*.
    void command_begin(UByte address, UByte command, UByte put_bytes);

    //! @brief Mark current command as completed.
    //!
    //! This method marks that the current command has ended.  The command
    //! is started by a call to *command_begin*().  In automatic flush mode,
    //! this command will cause the command to be immediately sent
    //! to the desired module.  In non automatic flush mode, the
    //! the command bytes will continue to collect until an explicit
    //! call to *flush*() occurs.
    void command_end();

    void debug_character(Character character) {
      #if BUS_DEBUG
	_debug_uart->frame_put((UShort)character);
      #endif //BUS_DEBUG
    };

    void debug_hex(UInteger hex) {
      #if BUS_DEBUG
	_debug_uart->uinteger_print((UInteger)hex);
      #endif //BUS_DEBUG
    };

    void debug_text(Text text) {
      #if BUS_DEBUG
	_debug_uart->string_print(text);
      #endif //BUS_DEBUG
    }

    Logical can_receive(){
      return _bus_uart->can_receive();
     };

    Logical can_transmit() {
      return _bus_uart->can_transmit();
    };

    Byte command_byte_get(UByte address, UByte command) {
      return (Byte)command_ubyte_get(address, command);
    };

    void command_byte_put(UByte address, UByte command, Byte byte) {
      command_ubyte_put(address, command, (UByte)byte);
    }

    UInteger command_integer_get(UByte address, UByte command);

    void command_integer_put(UByte address, UByte command, Integer integer);

    Short command_short_get(UByte address, UByte command) {
      return (Short)command_ushort_get(address, command);
    };

    void command_short_put(UByte address, UByte command, Short xshort) {
      command_ushort_put(address, command, (UShort)xshort);
    }

    UByte command_ubyte_get(UByte address, UByte command);

    void command_ubyte_put(UByte address, UByte command, UByte ubyte);

    UShort command_ushort_get(UByte address, UByte command);

    void command_ushort_put(UByte address, UByte command, UShort ushort);

    UShort frame_get() { 
      UShort frame = _bus_uart->frame_get();
      debug_character('g');
      debug_hex((UInteger)frame);
      return frame;
    };

    void frame_put(UShort frame) {
      debug_character('p');
      debug_hex((UInteger)frame);
      _bus_uart->frame_put(frame);
    };

    Logical flush();

    //! @brief Return next *Integer* from currently selected module.
    //! @return the next *Integer* from currently selected module.
    //!
    //! This method will return the next *Integer* from the currently
    //! selected module.
    Integer integer_get() {
      return _get_buffer.integer_get();
    }

    //! @brief Send *integer* to currently selected module.
    //! @param integer	*Integer* unsigned short to send.
    //!
    //! This method will send *integer* to the currently selected module.
    void integer_put(Integer integer) {
      // Queue {integer} to be sent off to bus:
      _put_buffer.integer_put(integer);
    }

    void interrupt_set(Logical interrupt) {
      _bus_uart->interrupt_set(interrupt);
    };

    Logical logical_get() {
      return (Logical)ubyte_get();
    };

    //! @brief Send *logical* to currently selected module.
    //!   @param byte byte value to send to command.
    //!
    //! This member function will queue *logical* to be sent to the
    //! currently selected module as part of a multi-byte command.
    void logical_put(Logical logical) {
      // Queue {byte} to be sent off to bus:
      ubyte_put((UByte)logical);
    }

    //! @brief Return next *Short* from currently selected module.
    //! @return the next *Short* from currently selected module.
    //!
    //! This method will return the next *Short* from the currently
    //! selected module.
    Short short_get() {
      return _get_buffer.short_get();
    }

    //! @brief Send *short* to currently selected module.
    //! @param short	*Short* unsigned short to send.
    //!
    //! This method will send *short* to the currently selected module.
    void short_put(Short xshort) {
      // Queue {short} to be sent off to bus:
      _put_buffer.short_put(xshort);
    }

    //! @brief Handle bus communication for a module in slave mode.
    //!   @param address Module address to listen to
    //!   @param command_process helper routine to process each
    //!                          byte of received from master
    void slave_mode(UByte address, UByte (*command_process)
     (Bus_Slave *bus_slave, UByte command, Logical mode));


    //! @brief Return next byte from currently selected module.
    //!   @return the next byte from currently selected module.
    //!
    //! This method will return the next unsigned byte from the currently
    //! selected module.
    UByte ubyte_get() {
      return (UByte)_get_buffer.ubyte_get();
    };

    //! @brief Send *ubyte* to currently selected module.
    //!    @param ubyte	unsigned byte to send
    //!
    //! This method will send *ubyte* to the currently selected module.
    void ubyte_put(UByte ubyte) {
      // Queue {ubyte} to be sent off to bus:
      _put_buffer.ubyte_put(ubyte);
    };

    //! @brief Return next *UInteger* from currently selected module.
    //! @return the next *UInteger* from currently selected module.
    //!
    //! This method will return the next *UInteger* from the currently
    //! selected module.
    UInteger uinteger_get() {
      return _get_buffer.uinteger_get();
    }

    //! @brief Send *uinteger* to currently selected module.
    //! @param uinteger	*UInteger* unsigned short to send.
    //!
    //! This method will send *uinteger* to the currently selected module.
    void uinteger_put(UInteger uinteger) {
      // Queue {uinteger} to be sent off to bus:
      _put_buffer.uinteger_put(uinteger);
    }

    //! @brief Return next short from currently selected module.
    //!   @return the next short from currently selected module.
    //!
    //! This method will return the next unsigned short from the currently
    //! selected module.
    UShort ushort_get() {
      return _get_buffer.ushort_get();
    }

    //! @brief Send *ushort* to currently selected module.
    //!    @param ushort	unsigned short to send
    //!
    //! This method will send *ushort* to the currently selected module.
    void ushort_put(UShort ushort) {
      // Queue {ushort} to be sent off to bus:
      _put_buffer.ushort_put(ushort);
    }

  private:
    static const UByte _maximum_request_size = 15;

    UART *_bus_uart;		// *UART* connected to bus
    UART *_debug_uart;		// *UART* used for debugging messages
    Bus_Buffer _get_buffer;	// FIFO for received bytes
    Bus_Buffer _put_buffer;	// FIFO queue for bytes to send

    Logical _auto_flush;	// 1=>Auto flush every cmd; 0=>queue up cmds
    //Logical _master_mode;	// 1=>master mode; 0=>slave mode 
    UShort _desired_address;	// Desired address
    UShort _current_address;	// Current address

    // This stuff is pretty stale:
    // The frame log is only enabled when *BUS_DEBUG* is set to 1:
    //#if BUS_DEBUG
    //  UShort _log_buffer[BUS_LOG_SIZE];	// Buffer of read/written frames
    //  UByte _log_total;			// Total number read/written
    //  UByte _log_dumped;		// Total number dumped out
    //#endif // BUS_DEBUG
};

class Bus_Module
{
  public:
    Bus_Module() {
      // Construct an empty module:
      _bus_slave = (Bus_Slave *)0;
      _address = 0xff;
    }

    void auto_flush_set(Logical auto_flush) {
      // This routine will Set the automatic flush mode for {this}
      // to {auto_flush}.  If {auto_flush} is 1, each command as
      // flushed as soon as possible.  If {auto_flush} is 0,
      // commands are queued until an explicit flush occurs.
      // The previous value of flush mode is returned.

      _bus_slave->auto_flush_set(auto_flush);
    }

    void bind(Bus_Slave *bus_slave, UByte address);
      // Slave module is located at {address} on {bus}:

    Byte byte_get() {
      // This routine will return next next {Byte} from the current
      // module selected by {this}.

	return (Byte)_bus_slave->ubyte_get();
    }

    void byte_put(Byte byte) {
      // This routine will send {byte} to the current module
      // selected by {this}.

 	_bus_slave->ubyte_put((UByte)byte);
    }

    Character character_get() {
      // This routine will return next {Character} from current
      // module selected by {this}.

      return (Logical)_bus_slave->ubyte_get();
    }

    void character_put(Character character) {
      // This routine will send {character} to the current module
      // selected by {this}.

      _bus_slave->ubyte_put((UByte)character);
    }

    void command_begin(UByte command, UByte put_bytes) {
      // This routine will start a new bus command that starts
      // with the byte {command}.  This command is sent to the
      // current module selected by {this}.

      _bus_slave->command_begin(_address, command, put_bytes);
    }

    void command_end() {
      // This routine will end the current command being sent to
      // the current module selected by {this}.

      _bus_slave->command_end();
    }

    void flush() {
      // This routine will flush all pending commands to the current
      // module selected by {this}.

      _bus_slave->flush();
    }

    Logical logical_get() {
      // This routine will return next next byte from current
      // module selected by {this}.

      return (Logical)_bus_slave->ubyte_get();
    }

    Integer integer_get() {
      return _bus_slave->integer_get();
    }

    void integer_put(Integer integer) {
      //_bus->integer_put(integer);
      _bus_slave->integer_put(-123);
    }

    void logical_put(Logical logical) {
      // Send {logical} to slave:
      _bus_slave->ubyte_put((UByte)logical);
    }

    UByte ubyte_get() {
      // This routine will return next next byte from current
      // module selected by {this}.

      return _bus_slave->ubyte_get();
    }

    void ubyte_put(UByte ubyte) {
      // Send {ubyte} to slave:
      _bus_slave->ubyte_put(ubyte);
    }

    void slave_mode(UByte address,
     UByte (*command_process)(Bus_Slave *, UByte, Logical)) {
      return _bus_slave->slave_mode(address, command_process);
    }

  private:
    Bus_Slave *_bus_slave;
    UByte _address;
};

extern Bus_Slave bus_slave;

#endif // BUS_SLAVE_H_INCLUDED
