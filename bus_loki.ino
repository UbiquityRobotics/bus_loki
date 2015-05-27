// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.
#include <Bus_Slave.h>
#include <Frame_Buffer.h>
#include <bus_server.h>

#define BUS_LOKI_PROGRAM_BLINK 0
#define BUS_LOKI_PROGRAM_MOTOR 1
#define BUS_LOKI_PROGRAM_ENCODERS_TO_LEDS 2
#define BUS_LOKI_PROGRAM_LEDS_COUNT 3
#define BUS_LOKI_PROGRAM_SONAR_TEST 4
#define BUS_LOKI_PROGRAM_UART_WRITE 5
#define BUS_LOKI_PROGRAM_UART_ECHO 6
#define BUS_LOKI_PROGRAM_RAB 7		// RAB = ROS Arduino Bridge

#define BUS_LOKI_PROGRAM BUS_LOKI_PROGRAM_RAB

// Make sure we are using the ROS Arduino Bridge as configured for LOKI:
#define TEST TEST_RAB_LOKI


// Setup control for debug printouts that we can manage as a remote command
// These are binary bits in a flag word that can be viewed from other modules
static int system_debug_flags = DBG_FLAG_UART_SETUP | DBG_FLAG_PARAMETER_SETUP;


int system_debug_flags_get() {
  return system_debug_flags;
}
void system_debug_flags_set(int flags) {
  system_debug_flags = flags;
}

class Loki_Motor_Encoder : Bus_Motor_Encoder {
 public:
  Loki_Motor_Encoder(UByte input1_pin, UByte input2_pin,
   UByte enable_pin, Integer *encoder_pointer);
  Integer encoder_get();
  void encoder_set(Integer encoder);
  void pwm_set(Byte pwm);
 private:
  UByte _input1_pin;
  UByte _input2_pin;
  UByte _enable_pin;
  Integer *_encoder_pointer;
};

// Encoder buffer:
#define BUFFER_POWER 8
#define BUFFER_SIZE (1 << BUFFER_POWER)
#define BUFFER_MASK (BUFFER_SIZE - 1)
static UByte encoder_buffer[BUFFER_SIZE];
static volatile UByte encoder_buffer_in = 0;
static volatile UByte encoder_buffer_out = 0;

// There are two encoders:
//
//    Encoder   Phase  Arduino Pin   PCInt
//    =======   =====  ===========   =======
//      1	  A        A0        PCInt8
//      1         B        A1        PCInt9
//      2         A        A2        PCInt10
//      2         B        A3        PCInt11

// This table is computed by bus_bridge_encoders_sonar.py
// Please read the comment in the Python code to understand
// the structure of the *state_transition_table* below:
static Byte state_transition_table[32] = {
    //            ccccc         sgg   // [xx]: SGG[gg] => sgg +/-
    (signed char)((0x00 << 3) | 0x0), // [00]: 000[00] => 000  0
    (signed char)((0x1f << 3) | 0x4), // [01]: 001[00] => 100 -1
    (signed char)((0x01 << 3) | 0x0), // [02]: 010[00] => 000 +1
    (signed char)((0x02 << 3) | 0x0), // [03]: 011[00] => 000 +2
    (signed char)((0x00 << 3) | 0x4), // [04]: 100[00] => 100  0
    (signed char)((0x1f << 3) | 0x4), // [05]: 101[00] => 100 -1
    (signed char)((0x01 << 3) | 0x0), // [06]: 110[00] => 000 +1
    (signed char)((0x1e << 3) | 0x4), // [07]: 111[00] => 100 -2
    (signed char)((0x01 << 3) | 0x1), // [08]: 1000[01] => 001 +1
    (signed char)((0x00 << 3) | 0x1), // [09]: 1001[01] => 001  0
    (signed char)((0x02 << 3) | 0x1), // [10]: 1010[01] => 001 +2
    (signed char)((0x1f << 3) | 0x5), // [11]: 1011[01] => 101 -1
    (signed char)((0x01 << 3) | 0x1), // [12]: 1100[01] => 001 +1
    (signed char)((0x00 << 3) | 0x5), // [13]: 1101[01] => 101  0
    (signed char)((0x1e << 3) | 0x5), // [14]: 1110[01] => 101 -2
    (signed char)((0x1f << 3) | 0x5), // [15]: 1111[01] => 101 -1
    (signed char)((0x1f << 3) | 0x6), // [16]: 10000[10] => 110 -1
    (signed char)((0x02 << 3) | 0x2), // [17]: 10001[10] => 010 +2
    (signed char)((0x00 << 3) | 0x2), // [18]: 10010[10] => 010  0
    (signed char)((0x01 << 3) | 0x2), // [19]: 10011[10] => 010 +1
    (signed char)((0x1f << 3) | 0x6), // [20]: 10100[10] => 110 -1
    (signed char)((0x1e << 3) | 0x6), // [21]: 10101[10] => 110 -2
    (signed char)((0x00 << 3) | 0x6), // [22]: 10110[10] => 110  0
    (signed char)((0x01 << 3) | 0x2), // [23]: 10111[10] => 010 +1
    (signed char)((0x02 << 3) | 0x3), // [24]: 11000[11] => 011 +2
    (signed char)((0x01 << 3) | 0x3), // [25]: 11001[11] => 011 +1
    (signed char)((0x1f << 3) | 0x7), // [26]: 11010[11] => 111 -1
    (signed char)((0x00 << 3) | 0x3), // [27]: 11011[11] => 011  0
    (signed char)((0x1e << 3) | 0x7), // [28]: 11100[11] => 111 -2
    (signed char)((0x01 << 3) | 0x3), // [29]: 11101[11] => 011 +1
    (signed char)((0x1f << 3) | 0x7), // [30]: 11110[11] => 111 -1
    (signed char)((0x00 << 3) | 0x7), // [31]: 11111[11] => 111  0
};

// Define pin numbers:
static const int encoder_r1_pin = 10;		// IC Pin 23
static const int encoder_r2_pin = 11;		// IC Pin 24
static const int encoder_l1_pin = 12;		// IC Pin 25
static const int encoder_l2_pin = 13;		// IC Pin 26
static const int led0_pin = 30;
static const int led1_pin = 31;
static const int led2_pin = 32;
static const int led3_pin = 33;
static const int led4_pin = 34;
static const int led5_pin = 35;
static const int led6_pin = 36;
static const int led7_pin = 37;
static const int miso_pin = 50;
static const int motor1_input1_pin = 43;
static const int motor1_input2_pin = 44;
static const int motor1_enable_pin = 45;
static const int motor2_input1_pin = 5;
static const int motor2_input2_pin = 3;
static const int motor2_enable_pin = 2;

// for sonar pins also see the usonar tables we use to lookup
static const int sonar_echo1_pin  = A15;	// IC Pin 82
static const int sonar_echo2_pin  = A14;	// IC Pin 83
static const int sonar_echo3_pin  = A13;	// IC Pin 84
static const int sonar_echo4_pin  = A12;	// IC Pin 85
static const int sonar_echo5_pin  = A11;	// IC Pin 86
static const int sonar_echo6_pin  = A10;	// IC Pin 87
static const int sonar_echo7_pin  = A9;		// IC Pin 88
static const int sonar_echo8_pin  = A8;		// IC Pin 89
static const int sonar_echo9_pin  = 0; 		// IC Pin 69
static const int sonar_echo10_pin = 0; 		// IC Pin 68
static const int sonar_echo11_pin = 0; 		// IC Pin 67
static const int sonar_echo12_pin = 0; 		// IC Pin 66
static const int sonar_echo13_pin = 0; 		// IC Pin 65
static const int sonar_echo14_pin = 0; 		// IC Pin 65
static const int sonar_echo15_pin = 0; 		// IC Pin 64
static const int sonar_echo16_pin = 0; 		// IC Pin 64

static const int sonar_trig1_pin  = 39;		// IC Pin 70
static const int sonar_trig2_pin  = 22;		// IC Pin 78
static const int sonar_trig3_pin  = 23;		// IC Pin 77
static const int sonar_trig4_pin  = 24;		// IC Pin 76
static const int sonar_trig5_pin  = 25;		// IC Pin 75
static const int sonar_trig6_pin  = 26;		// IC Pin 74
static const int sonar_trig7_pin  = 27;		// IC Pin 73
static const int sonar_trig8_pin  = 28;		// IC Pin 72
static const int sonar_trig9_pin  = 29;		// IC Pin 71
static const int sonar_trig10_pin = 0;		// IC Pin 79
static const int sonar_trig11_pin = 46;		// IC Pin 38
static const int sonar_trig12_pin = 47;		// IC Pin 37
static const int sonar_trig13_pin = 48;		// IC Pin 36
static const int sonar_trig14_pin = 49;		// IC Pin 35
static const int sonar_trig15_pin = 0; 		// IC Pin 29
static const int sonar_trig16_pin = 0; 		// IC Pin 28

// Define the UART's:
NULL_UART null_uart;
AVR_UART *bus_uart = &avr_uart1;
AVR_UART *debug_uart = &avr_uart0;
AVR_UART *host_uart = &avr_uart0;

// The two encoder values:
Integer encoder1 = 0;
Integer encoder2 = 0;

Bus_Slave bus_slave((UART *)bus_uart, (UART *)host_uart);

Loki_Motor_Encoder left_motor_encoder(motor1_input1_pin, motor1_input2_pin,
 motor1_enable_pin, &encoder1);
Loki_Motor_Encoder right_motor_encoder(motor2_input1_pin, motor2_input2_pin,
 motor2_enable_pin, &encoder2);

Bridge bridge(&avr_uart0, &avr_uart1, &avr_uart0, &bus_slave,
 (Bus_Motor_Encoder *)&left_motor_encoder,
 (Bus_Motor_Encoder *)&right_motor_encoder);

void leds_byte_write(char byte) {
  //digitalWrite(led0_pin, (byte & 1) ? LOW : HIGH);
  //digitalWrite(led1_pin, (byte & 2) ? LOW : HIGH);
  //digitalWrite(led2_pin, (byte & 4) ? LOW : HIGH);
  //digitalWrite(led3_pin, (byte & 8) ? LOW : HIGH);
  //digitalWrite(led4_pin, (byte & 16) ? LOW : HIGH);
  //digitalWrite(led5_pin, (byte & 32) ? LOW : HIGH);
  //digitalWrite(led6_pin, (byte & 64) ? LOW : HIGH);
  //digitalWrite(led7_pin, (byte & 128) ? LOW : HIGH);
  PORTC = byte;
}

// *PCINT0_vect*() is the interrupt service routine for the
// pin change interrupts PCINT7/.../0.  The two encoders are
// attached to PCINT7/6/5/4, so these are the bits we want
// to capture.  This routine just stuffs the encoder bits
// into a buffer:

ISR(PCINT0_vect) {
  // Stuff the port C input bits into *buffer* and advance *buffer_in*:
  UByte bits = PINB;
  encoder_buffer[encoder_buffer_in] = bits;
  encoder_buffer_in = (encoder_buffer_in + 1) & BUFFER_MASK;
  PORTC = PINB;
  //leds_byte_write(bits);
  //leds_byte_write(encoder_buffer_in);
  //host_uart->print("0123456789abcdef"[(bits >> 4) & 0xf]);
  //UDR0 = '.';
}

/* ********************************************************************************************
 * UltraSonic Sonar Code
 *
 * One part of the code must be extremely fast code used in ISRs to log edge detections
 *
 * A second part of the code is used in background to do round robin sonar scans and keep track
 * of latest good measurements of distance so they can be fetched by interested subsystems.
 *
 * Note: The background code could be moved into a separate header and file for the class.
 */

// Some in-memory history of last sample times and readings in meters
#define USONAR_MAX_UNITS    17    // Sonar number as silkscreened on Loki board
unsigned long sonarSampleTimes[USONAR_MAX_UNITS];
float sonarDistancesInMeters[USONAR_MAX_UNITS];


// Define a max expected delay time in microseconds.
// 30000 is about 5 meters
#define USONAR_ECHO_MAX       ((long)(28100))   // Longest echo time we expect (28100 is 5  meters)
#define USONAR_MEAS_TIME     ((long)(100000))   // Time to wait per measurement
#define USONAR_MEAS_TOOLONG   ((long)(70000))   // Meas delay too long
#define USONAR_SAMPLES_US    ((long)(150000))   // One measurement each time this many uSec goes by
#define USONAR_ECHO_ERR1     ((long)(1*282))  
#define USONAR_ECHO_ERR2     ((long)(2*282)) 
#define USONAR_ECHO_ERR3     ((long)(3*282)) 
#define USONAR_ECHO_ERR4     ((long)(4*282)) 
#define USONAR_US_TO_METERS  ((float)(5620.0))  // Standard air nominal uSec delay for 2-way bounce time
#define USONAR_MAX_DIST_CM        900.0      // a cap used in reading well after meas has finished

// States of sonar sensor acquision
#define  USONAR_STATE_MEAS_START         0
#define  USONAR_STATE_WAIT_FOR_MEAS      1
#define  USONAR_STATE_POST_SAMPLE_WAIT   2

// System dependent clock for getting microseconds as fast as we can do it
#define SYSTEM_GET_MICROSECONDS    micros()

// Tables to map sonar number to trigger digital line # and echo Axx line
// If entry is 0, not supported yet as it does not have digital pin #
// Need a more clever set of code to deal with all the abnormal pins
typedef struct usonar_meas_spec_t {
    int measMethod;	// Method to be used for the measurement
    int trigPin;        // For direct pin this is digital pin number for custom it a routine to use
    int echoPin;        // The echo pin in terms of arduino pin to use for pulseIn
    int intRegNum;      // Int reg number for pinint interrupt enable
    int intBit;         // the bit for enable of interrupts for this pinint
} Usonar_Meas_Spec;

// Because a unit can support either of two methods the measurement method field is a bitmap
// Custom methods once we support them will add to the 2 well known methods of PIN and PCINT 
#define US_MEAS_METHOD_NONE    0
#define US_MEAS_METHOD_PIN     1     // Supports direct pulseIn() method
#define US_MEAS_METHOD_PCINT   2     // Supports pin change interrupt method
// Custom modes for trigger start
// Trigger is Port J bit 7
#define US_MEAS_METHOD_T01_PG2 (0x010|US_MEAS_METHOD_PIN|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T10_PJ7 (0x020|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T15_PG4 (0x040|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T16_PG3 (0x080|US_MEAS_METHOD_PCINT)

#define US_MEAS_METHOD_PIN_PCINT   (US_MEAS_METHOD_PIN|US_MEAS_METHOD_PCINT)

// Pin change interrupts are involved so we have a little table to help
// sort them out that must match the hardware.
// This table was done for Loki Rev C board
//
// IntBit: _BV(7)   _BV(6)   _BV(5)   _BV(4)   _BV(3)   _BV(2)  _BV(1)    _BV(0);
//
// Sonar:     1        2        3        4        5        6       7        8  
// ChipPin:  82       83       84       85       86       87      88       89
// PCINT2:   23       22       21       20       19       18      17       16
//
// Sonar:     9       10       11       12    13,14   15,16       --       --
// ChipPin:  69       68       67       66       65      64       --       --
// PCINT1:   15       14       13       12       11      10       --       --

//  This table defines the methods that sonar measurements can be taken 
const Usonar_Meas_Spec  usonar_measSpecs[USONAR_MAX_UNITS] = { {0,0,0,0,0},
    {US_MEAS_METHOD_T01_PG2,   sonar_trig1_pin,  sonar_echo1_pin,  2,  _BV(7) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig2_pin,  sonar_echo2_pin,  2,  _BV(6) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig3_pin,  sonar_echo3_pin,  2,  _BV(5) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig4_pin,  sonar_echo4_pin,  2,  _BV(4) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig5_pin,  sonar_echo5_pin,  2,  _BV(3) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig6_pin,  sonar_echo6_pin,  2,  _BV(2) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig7_pin,  sonar_echo7_pin,  2,  _BV(1) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig8_pin,  sonar_echo8_pin,  2,  _BV(0) },
    {US_MEAS_METHOD_PIN_PCINT, sonar_trig9_pin,  sonar_echo9_pin,  1,  _BV(7) },
    {US_MEAS_METHOD_T10_PJ7,   sonar_trig10_pin, sonar_echo10_pin, 1,  _BV(6) },
    {US_MEAS_METHOD_PCINT,     sonar_trig11_pin, sonar_echo11_pin, 1,  _BV(5) },
    {US_MEAS_METHOD_PCINT,     sonar_trig12_pin, sonar_echo12_pin, 1,  _BV(4) },
    {US_MEAS_METHOD_PCINT,     sonar_trig13_pin, sonar_echo13_pin, 1,  _BV(3) },
    {US_MEAS_METHOD_PCINT,     sonar_trig14_pin, sonar_echo14_pin, 1,  _BV(3) },
    {US_MEAS_METHOD_T15_PG4,   sonar_trig15_pin, sonar_echo15_pin, 1,  _BV(2) },
    {US_MEAS_METHOD_T16_PG3,   sonar_trig16_pin, sonar_echo16_pin, 1,  _BV(2) } 
};

// Define the circular queue members and indexes.
// Remember! this is highly optimized so no fancy classes used here folks!
//
// Queue Rules:
//  - The index values increment for each entry and roll back to 0 when equal to USONAR_QUEUE_LEN
//    It is possible to have consumer index near end and producer near start index of 0 which means
//    the entries are rolling around but are still all valid.  Producers and consumer MUST deal with this.
//  - Producer may only put in entries if 2 or more spaces are open
//  - Producer must drop entries if queue is full
//  - Producer places data at next index past current producer index in both arrays 
//    and THEN bumps producer index to point to that index when values are intact.
//  - Consumer may only read entries up to and including current producer index.
//  - Consumer may only bump consumer index to one more than what has just been processed.
//    Consumer must NEVER try to read the values once consumer has bumped consumer index past
//
// Queue Member Descriptions
//  usonar_echoEdgeQueue[]    ISR fed queue to hold edge change detections produced in fast ISR.
//                            Entires are 30 bit timestamp with lower 2 bits showing the bank that changed
//                            Note that the timestamp will roll over in around 70 min
//  usonar_EchoChangedPins[]  Not used yet!  Is to hold a 1 for each pin that has changed at this timestamp.
//                            Bits 23:0 are to be placed properly by each ISR
//

// The Echo timestamp is saved with the least sig 2 bits being set to the channel for this timestamp
// We do not know which bits changed, we only know at least one changed in this bank.
// We also have a word of info to hold other info we may want the ISR to return
// So this means the sonar cycling needs to be careful to only do one sample at a time from any one bank
#define  USONAR_QUEUE_LEN     8             // MUST be a power of 2
unsigned long usonar_echoEdgeQueue[USONAR_QUEUE_LEN];
unsigned long usonar_echoInfoQueue[USONAR_QUEUE_LEN];

#define  USONAR_MIN_EMPTIES   2            // minimum space where producer can insert new entry
unsigned int  usonar_producerIndex = 0;    // Owned by ISRs and only inspected by consumer
unsigned int  usonar_consumerIndex = 0;    // Owned by consumer and only inspected by producer
int usonar_queueOverflow = 0;              // Producer can set this and consumer has to be aware. 

// 
// Class to pull together management routines for Loki ultrasonic sensor distance measurements
//
// This is really to make things a little bit more contained but does not stand on it's own
// This is specific to use of high speed ISR doing acquisition of data
// so this class must use external fast edge detect queue which is not 'ideal' but is acceptable
//
class Loki_USonar {
  private:
    int _numSonars;

  public:
    Loki_USonar() {
        _numSonars = USONAR_MAX_UNITS;
    };


  // Sonar echo completion Circular Queue interfaces for a background consumer


  // A generic circular queue utility to get number of entries in any circular queue
  int calcQueueLevel(int Pidx, int Cidx, int queueSize) {
    int queueLevel = 0;
    if (Pidx >= Cidx) {
      // Simple case is Cidx follows Pidx OR empty queue they are equal
      queueLevel = (Pidx - Cidx);  
    } else {
      queueLevel = (queueSize - Cidx) + Pidx + 1; 
    }
    return queueLevel;
  };

  //
  // getQueueLevel() returns number of entries in the circular queue but changes nothing
  //
  int getQueueLevel() {
    int localPI = usonar_producerIndex;		// get atomic copy of producer index

    return calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN);
  };


  //
  // Pull one entry from our edge detection circular queue if there are entries.
  // A return of 0 will happen if no entries are ready to pull at this time OR the entry is 0
  //
  unsigned long pullQueueEntry() {
    int localPI = usonar_producerIndex;		// get atomic copy of producer index
    unsigned long queueEntry = 0;

    if (calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN) > 0) {
 
      queueEntry = usonar_echoEdgeQueue[usonar_consumerIndex];

      // Find the next index we will bump the consumer index to once done
      int nextCI = usonar_consumerIndex + 1;
      if (nextCI >= USONAR_QUEUE_LEN) {
        nextCI = 0;	// case of roll-around for next entry
      }

      usonar_consumerIndex = nextCI;           // We have consumed so bump consumer index
    }

    return queueEntry;
  };

  //
  // This empties the queue and no members are seen, they just go bye-bye
  //
  // We do return how many members were flushed
  //
  int flushQueue() {
    int queueEntries = 0;
    while (pullQueueEntry() != 0) { queueEntries++; };    // Eat um all up, yum yum.
    return queueEntries;
  };


  // Get the trigger pin from sonar unit number
  //
  // Negative value indicates unsupported unit number
  // Zero return 0 indicates this unit does not support trigger line
  // a custom trigger is required for the measurement method.
  //
  int getMeasTriggerPin(int sonarUnit) {
    int trigPin = 0;

    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }
   
    // If either of two modes is supported for main modes there is a trigger line
    if (usonar_measSpecs[sonarUnit].measMethod & US_MEAS_METHOD_PIN_PCINT) {
        trigPin = usonar_measSpecs[sonarUnit].trigPin;
    }

    return trigPin;
  };

  // Get the echo detect pin from sonar unit number
  // Echo detect is only used for the inline measurement modes
  // which only the lower half of the sonars support.
  //
  // Negative value indicates unsupported unit number
  // Zero return indicates this unit does not support the feature
  int getEchoDetectPin(int sonarUnit) {
    int echoPin = 0;

    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }

    // We have to have this sonar unit at least support echo pin mode
    if (usonar_measSpecs[sonarUnit].measMethod & US_MEAS_METHOD_PIN) {
        echoPin = usonar_measSpecs[sonarUnit].echoPin;
    }

    return echoPin;
  };

  // Indicate if the sonar unit is supported in any way at all
  //
  // Negative value indicates unsupported unit number
  // Zero return indicates this unit does not support the feature
  // Non-zero returns the measurement methods and is thus non-zero
  int isUnitEnabled(int sonarUnit) {
    int enabled = 0;
    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }

    // This may seem 'silly' since NONE is zero but in case it changes
    // we will explicitly check with the define
    enabled = usonar_measSpecs[sonarUnit].measMethod;
    if (enabled == US_MEAS_METHOD_NONE) {
      enabled = 0;
    }

    return enabled;
  };


  // Trigger the given ultrasonic sonar unit
  // The sonar unit number is given to this routine as written on PC board
  // This routine shields the caller from knowing the hardware pin
  //
  // Return value is system clock in microseconds for when the trigger was sent
  // Note that the sonar itself will not reply for up to a few hundred microseconds
  //
  #define  USONAR_TRIG_PRE_LOW_US   4     // Time to hold trig line low b4 start
  #define  USONAR_TRIG_HIGH_US     20     // Time to hold trigger line high

  unsigned long  measTrigger(int sonarUnit)
  {
    unsigned long triggerTime;
    triggerTime = SYSTEM_GET_MICROSECONDS | 1;   // set lsb, zero is an error code

    // Trap out custom trigger pin modes (Ugly but necessary)
    if (usonar_measSpecs[sonarUnit].measMethod == US_MEAS_METHOD_T01_PG2) {
      PORTG &= 0xfb;
      delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
      PORTG |= 0x04;
      delayMicroseconds(USONAR_TRIG_HIGH_US);
      PORTG &= 0xfb;
    } else if (usonar_measSpecs[sonarUnit].measMethod == US_MEAS_METHOD_T10_PJ7) {
      PORTJ &= 0x7f;
      delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
      PORTJ |= 0x80;
      delayMicroseconds(USONAR_TRIG_HIGH_US);
      PORTJ &= 0x7f;
    } else if (usonar_measSpecs[sonarUnit].measMethod == US_MEAS_METHOD_T15_PG4) {
      PORTG &= 0xef;
      delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
      PORTG |= 0x10;
      delayMicroseconds(USONAR_TRIG_HIGH_US);
      PORTG &= 0xef;
    } else if (usonar_measSpecs[sonarUnit].measMethod == US_MEAS_METHOD_T16_PG3) {
      PORTG &= 0xf7;
      delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
      PORTG |= 0x08;
      delayMicroseconds(USONAR_TRIG_HIGH_US);
      PORTG &= 0xf7;
    } else {
      // Non-custom modes just lookup digital line and do trigger with that
      int trigPin = getMeasTriggerPin(sonarUnit);
      if (trigPin == 0) {
        return 0;
      }

      digitalWrite(trigPin, LOW);
      delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(USONAR_TRIG_HIGH_US);
      digitalWrite(trigPin, LOW);
    }

    return triggerTime;
  };

  float echoUsToMeters(unsigned long pingDelay) 
  {
    float  meters;
    meters = (float)(pingDelay) / USONAR_US_TO_METERS;
    return meters;
  };

  // Trigger and readback delay from an HC-SR04 Ulrasonic Sonar
  //
  // Returns distance in meters
  //    0.0 = Bad measurement result (unclear fault of sensor)
  //   -1.0 =  invalid sensor number
  //   -2.0 =  sensor number is in range but not supported yet
  //
  // Note that this routine will not cause our background edge triggering
  // interrupts as the pulseIn() routine seems to prevent edge interrupts
  //
  float inlineReadMeters(int sonarUnit) {
    float distInMeters = 0.0;
    unsigned long startTics;
    int echoPin = getEchoDetectPin(sonarUnit);
    if (echoPin < 0) {
        return -2.0;		// sensor number not supported yet for inline read
    }

    startTics = measTrigger(sonarUnit);    // Trigger the sonar unit to measure
    if (startTics == 0) {
        return -1.0;		// Bad sensor number or not supported yet
    }

    // Wait on the edge detect to measure pulse width in microseconds
    unsigned long duration = pulseIn(echoPin, HIGH, USONAR_ECHO_MAX);
    distInMeters = echoUsToMeters(duration);

    // If we get distance less than spec, return 0 as this can also be a fault
    if (distInMeters < 0.03) {
        distInMeters = 0;
    }

    return distInMeters;
  };

};


// *PCINT1_vect*() is one of two ISRs to gather sonar bounce pulse widths
//
// This ISR processes pin changes for Bank 1 which is for PCINT pins 15:8
// This ISR must be LIGHTNING FAST and is bare bones petal-to-the-metal processing!
//
// Pidx = Cidx when queue is empty.  Consumer sees this as empy. Pi can stuff away
// Pidx = Index that will next be filled by producer IF there was space
// Cidx = Index that will be read next time around IF Pidx != Cidx
//
// Pin Change Processing:
// This ISR will take in which pins had changes on one bank of pins
// and then push those changes with an associated timestamp in echoQueue[]
// The echoQueue[] is a circular queue so if the background consumer task
// has left the queue too full this ISR will loose data.
ISR(PCINT1_vect) {
   unsigned long now = SYSTEM_GET_MICROSECONDS; 	// Get clock FAST as we can

   int unused = 0;
   int inuse  = 0;
   // Ensure there is room in the queue even if we have rollover
   if (usonar_consumerIndex <= usonar_producerIndex) {
      inuse = usonar_producerIndex - usonar_consumerIndex;
   } else {
      inuse = (USONAR_QUEUE_LEN - usonar_consumerIndex) + usonar_producerIndex + 1;
   }
   unused = USONAR_QUEUE_LEN - inuse;

   if (unused > 0) {
       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_echoEdgeQueue[usonar_producerIndex]  = (now & 0xfffffffc) | 1;
       usonar_producerIndex = nextProducerIndex;  // Bump producer index to this valid one
   } else {
       usonar_queueOverflow  += 1;
   }
}

// *PCINT2_vect*() is one of two ISRs to gather sonar bounce pulse widths
//
// This ISR processes pin changes for Bank 2 which is for PCINT pins 23:16
// This ISR must be LIGHTNING FAST and is bare bones petal-to-the-metal processing!
//
// Please see PCINT1_vect() for comments on the details of Pin Change processing
// 
ISR(PCINT2_vect) {
   unsigned long now = SYSTEM_GET_MICROSECONDS; 	// Get clock FAST as we can

   int unused = 0;
   int inuse  = 0;
   // Ensure there is room in the queue even if we have rollover
   if (usonar_consumerIndex <= usonar_producerIndex) {
      inuse = usonar_producerIndex - usonar_consumerIndex;
   } else {
      inuse = (USONAR_QUEUE_LEN - usonar_consumerIndex) + usonar_producerIndex + 1;
   }
   unused = USONAR_QUEUE_LEN - inuse;

   if (unused >= USONAR_MIN_EMPTIES) {
       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_echoEdgeQueue[usonar_producerIndex]  = (now & 0xfffffffc) | 2;
       usonar_producerIndex = nextProducerIndex;  // Bump producer index to this valid one
   } else {
       usonar_queueOverflow  += 1;
   }
}


/*
 *********************  End UltraSonic Sonar Code ********************************************
*/


// Setup instance of class for Loki Ultrasonic Sensor support
static Loki_USonar usonar;
static int  currentSonarNumber;
static int  usonarSampleState;
static unsigned long sonarMeasTriggerTime;
static unsigned long currentDelayData1;
static unsigned long currentDelayData2;

// HACK!!!!  Calls accessable using extern for backdoors to query read sensor distances
// An external access hack so we can fetch sonar values from bus_server
int usonar_getLastDistInMm(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit >= USONAR_MAX_UNITS)) {
        return -10;
    }
    return (int)(sonarDistancesInMeters[sonarUnit] * (float)1000.0);
}
float usonar_inlineReadMeters(int sonarUnit) {
  return usonar.inlineReadMeters(sonarUnit);
}


// The *setup*() routine runs once when you press reset:
void setup() {

  usonarSampleState = USONAR_STATE_MEAS_START;
  currentSonarNumber = 0;
  sonarMeasTriggerTime = 0;
  currentDelayData1 = (unsigned long)0;
  currentDelayData2 = (unsigned long)0;
  usonar_producerIndex = 0;
  usonar_consumerIndex = 0;
  for (int ix = 0; ix < USONAR_MAX_UNITS; ix++) {
    sonarDistancesInMeters[ix] = (float)(0.0);
    sonarSampleTimes[ix] = (unsigned long)0;
  }

  // Initialize pin directions:
  pinMode(encoder_l1_pin, INPUT);
  pinMode(encoder_l2_pin, INPUT);
  pinMode(encoder_r1_pin, INPUT);
  pinMode(encoder_r2_pin, INPUT);
  pinMode(miso_pin, INPUT);
  pinMode(led0_pin, OUTPUT);
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(led3_pin, OUTPUT);
  pinMode(led4_pin, OUTPUT);
  pinMode(led5_pin, OUTPUT);
  pinMode(led6_pin, OUTPUT);
  pinMode(led7_pin, OUTPUT);
  pinMode(motor1_input1_pin, OUTPUT);
  pinMode(motor1_input2_pin, OUTPUT);
  pinMode(motor1_enable_pin, OUTPUT);
  pinMode(motor2_input1_pin, OUTPUT);
  pinMode(motor2_input2_pin, OUTPUT);
  pinMode(motor2_enable_pin, OUTPUT);
  pinMode(sonar_echo1_pin, INPUT);
  pinMode(sonar_echo2_pin, INPUT);
  pinMode(sonar_echo3_pin, INPUT);
  pinMode(sonar_echo4_pin, INPUT);
  pinMode(sonar_echo5_pin, INPUT);
  pinMode(sonar_echo6_pin, INPUT);
  pinMode(sonar_echo7_pin, INPUT);
  pinMode(sonar_echo8_pin, INPUT);
  pinMode(sonar_trig1_pin, OUTPUT);
  pinMode(sonar_trig2_pin, OUTPUT);
  pinMode(sonar_trig3_pin, OUTPUT);
  pinMode(sonar_trig4_pin, OUTPUT);
  pinMode(sonar_trig5_pin, OUTPUT);
  pinMode(sonar_trig6_pin, OUTPUT);
  pinMode(sonar_trig7_pin, OUTPUT);
  pinMode(sonar_trig8_pin, OUTPUT);
  pinMode(sonar_trig9_pin, OUTPUT);
  pinMode(sonar_trig11_pin, OUTPUT);
  pinMode(sonar_trig12_pin, OUTPUT);
  pinMode(sonar_trig13_pin, OUTPUT);
  pinMode(sonar_trig14_pin, OUTPUT);
  pinMode(sonar_trig15_pin, OUTPUT);
  pinMode(sonar_trig16_pin, OUTPUT);

  leds_byte_write(0);
  host_uart->begin(16000000L, 115200L, (Character *)"8N1");
  //host_uart->print((Text)"Double echo\r\n");

  switch (BUS_LOKI_PROGRAM) {
    case BUS_LOKI_PROGRAM_ENCODERS_TO_LEDS: {
      PCMSK0 = _BV(7) | _BV(6) | _BV(5) | _BV(4);
      PCICR = _BV(0);
      break;
    }
    case BUS_LOKI_PROGRAM_RAB: {
      //host_uart->print((Text)"Start bridge setup\r\n");
      bridge.setup(TEST);
      //host_uart->print((Text)"Bridge setup done\r\n");

      // Special port bit setups to allow sonar triggering
      DDRG = 0x1c;     // Sonar 15 and 16 and 1 triggers
      DDRJ = 0x80;     // Sonar 10 trigger

      // Set up Interrupt on Pin Change interrupt vector.  The encoder
      // pins are attached to PCINT7/6/5/4, so we only need to set
      // PCMSK0 to '1111 0000':
      PCMSK0 = _BV(7) | _BV(6) | _BV(5) | _BV(4);

      // Now enable interrupt on changes for PCINT7/.../0, by setting
      // PCICR to 1.  Thus, PCICR is set to 'xxxx x001' or '0000 0001':
      PCICR = _BV(0);

      // The Pin Change interrupts are enabled for each bit on Bank 1 
      // As that pin is used in the measurement sampling code, not here
      // MJ_FIXME1:  PCMSK1 = 0;
      PCMSK1 =  _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2) ;
      PCICR |= _BV(1);

      // Set up for interrupts on Bank 2 pin changes or pins 23:16
      // These are used for ultrasonic sonar units mostly in front of Loki
      // The Pin Change interrupts are enabled for each bit on Bank 1 
      // As that pin is used in the measurement sampling code, not here
      // MJ_FIXME1:  PCMSK2 = 0;
      PCMSK2 = _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2)          | _BV(0);
      PCICR |= _BV(2);


      // Enable global interrupts by setting the I bit (7th bit) in the
      // status register:
      //SREG |= _BV(7);

      break;
    }
  }
}

char led_counter = 0;

// The *loop*() routine runs over and over again forever:
void loop() {

  switch (BUS_LOKI_PROGRAM) {
    case BUS_LOKI_PROGRAM_BLINK: {
      int led_pin = led7_pin;

      // LOW will light up the LED:
      digitalWrite(led_pin, LOW);

      // Wait for a while:
      delay(1000);

      // HIGH will turn the LED off:
      digitalWrite(led_pin, HIGH);

      // Wait for a while:
      delay(1000);

      break;
    }
    case BUS_LOKI_PROGRAM_MOTOR: {
      for (int index = 0; index < 8; index++) {
	if ((index & 2) == 0) {
          digitalWrite(motor1_input1_pin, LOW);
          digitalWrite(motor2_input1_pin, LOW);
	  digitalWrite(led1_pin, HIGH);
        } else {
          digitalWrite(motor1_input1_pin, HIGH);
          digitalWrite(motor2_input1_pin, HIGH);
	  digitalWrite(led1_pin, LOW);
        }

	if ((index & 4) == 0) {
          digitalWrite(motor1_input2_pin, LOW);
          digitalWrite(motor2_input2_pin, LOW);
	  digitalWrite(led2_pin, HIGH);
        } else {
          digitalWrite(motor1_input2_pin, HIGH);
          digitalWrite(motor2_input2_pin, HIGH);
	  digitalWrite(led2_pin, LOW);
        }

	if ((index & 1) == 0) {
	  digitalWrite(led0_pin, HIGH);
          analogWrite(motor1_enable_pin, 0);
          analogWrite(motor2_enable_pin, 0);
        } else {
	  digitalWrite(led0_pin, LOW);
          analogWrite(motor1_enable_pin, 255);
          analogWrite(motor2_enable_pin, 255);
        }
        delay(2000);
      }
      break;
    }
    case BUS_LOKI_PROGRAM_ENCODERS_TO_LEDS: {
      // Read the encoder values:
      PORTC = PINB;
      break;
    }
    case BUS_LOKI_PROGRAM_LEDS_COUNT: {
      leds_byte_write(led_counter);
      delay(250);
      led_counter += 1;
      break;
    }
    case BUS_LOKI_PROGRAM_SONAR_TEST: {
      // Send the trigger pulse out:
      digitalWrite(sonar_trig5_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(sonar_trig5_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(sonar_trig5_pin, LOW);

      unsigned long duration = pulseIn(sonar_echo5_pin, HIGH);
      leds_byte_write(((duration >> 6) & 254) | (led_counter & 1));
      led_counter += 1;

      delay(100);
      break;
    }
    case BUS_LOKI_PROGRAM_UART_WRITE: {
      for (Character chr = ' '; chr < '`'; chr++) {
	host_uart->frame_put((UShort)chr);
      }
      host_uart->print((Text)"\r\n");
      delay(250);
      break;
    }
    case BUS_LOKI_PROGRAM_UART_ECHO: {
      Character character = host_uart->frame_get();
      host_uart->frame_put((UShort)character);
      host_uart->frame_put((UShort)character);
      break;
    }
    case BUS_LOKI_PROGRAM_RAB: {
      static UByte encoder1_state = 0;
      static UByte encoder2_state = 0;

      // Deal with encoder issues:
      if (encoder_buffer_in != encoder_buffer_out) {
	// Grab all 4 encoder bits from *encoder_buffer*.  The 4 bits are
	// in the high order nibble and need to be shifted down to the low
	// order nibble:
	UByte encoder_bits = encoder_buffer[encoder_buffer_out] >> 4;
	encoder_buffer_out = (encoder_buffer_out + 1) & BUFFER_MASK;

	// Just print out *encoder_bits*:
	//Serial.print(encoder_bits & 0x3);
	//Serial.print(":");
	//Serial.print((encoder_bits >> 2) & 0x3);
	//Serial.print("\n");

	// Process encoder2:
	// *encoder2_state* has 3 bits of state - '0000 0sss'.
	// We take encoder2 bits encoder from *encoder_bits* ('0000 00ee'),
	// shift them left by 3, and OR them with the *encoder2_state*:
	//
	//     encoder2_state:  0000 0sss
	//     encoder_bits<<3: 000e e000
	//  OR ==========================
	//     index            000e esss
	//
	UByte index = ((encoder_bits & 0x3) << 3) | encoder2_state;

	// Note that *state_transition* is signed.
	Byte state_transition = state_transition_table[index];
	encoder2 += state_transition >> 3;
	encoder2_state = (unsigned char)(state_transition & 0x7);

	// Process encoder1:
	// Now we do the same for encoder1 whose bits are 2 bits over
	// ('0000 eexx'):
	index = ((encoder_bits & 0xc) << 1) | encoder1_state;
	state_transition = state_transition_table[index];
	encoder1 += state_transition >> 3;
	encoder1_state = (unsigned char)(state_transition & 0x7);
      }

      // -----------------------------------------------------------------
      // Deal with sampling the sonar ultrasonic range sensors
      //
      switch (usonarSampleState) {
        case USONAR_STATE_MEAS_START: {
          int queueLevel = 0;
          queueLevel = usonar.getQueueLevel();  // In our scheme we expect this to always be 0
          if ((queueLevel != 0) && (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG)) {
            host_uart->print((Text)" Sonar WARNING at meas cycle ");
            host_uart->integer_print(currentSonarNumber);
            host_uart->print((Text)" start: Queue had ");
            host_uart->integer_print(queueLevel);
            host_uart->print((Text)" edges! \r\n");
            usonar.flushQueue();
            currentDelayData1 = 0;                // Reset the sample gathering values for this run
            currentDelayData2 = 0;
          } else {
            // Indicate we are going to start next sonar measurement cycle
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              host_uart->print((Text)" Sonar meas cycle ");
              host_uart->integer_print(currentSonarNumber);
              host_uart->print((Text)" starting.\r\n");
            }
          }

          currentSonarNumber += 1;
          if (currentSonarNumber > USONAR_MAX_UNITS) {
            currentSonarNumber = 1;
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_RESULTS) {
               char  outBuf[32];
               float distInCm;
               host_uart->print((Text)" Sonars: ");
               for (int sonarUnit = 1; sonarUnit < USONAR_MAX_UNITS ; sonarUnit++) {
                  distInCm = (float)(usonar_getLastDistInMm(sonarUnit))/(float)(10.0);
                  if (distInCm > USONAR_MAX_DIST_CM) {
                    distInCm = USONAR_MAX_DIST_CM;      // graceful hard cap
                  }
                  
                  dtostrf(distInCm, 3, 1, outBuf);
                  host_uart->string_print((Text)outBuf);
                  host_uart->string_print((Text)" ");
               }
               host_uart->string_print((Text)"\r\n");
            }
          }

          // Skip any unit that will not work with PinChange interrupt
          if ((usonar_measSpecs[currentSonarNumber].measMethod & US_MEAS_METHOD_PCINT) == 0) {
            // This unit will not work so just skip it and do next on on next pass
            break;
          }

          // Start the trigger for this sensor and enable interrupts
          usonar_producerIndex = 0;    // !!!BUG HACK_FIX!!!  FIXME!!! 
          usonar_consumerIndex = 0;    // !!!BUG HACK_FIX!!!  FIXME!!!

          // Enable this units pin change interrupt then enable global ints for pin changes
          PCIFR = 0x06;		// This clears any pending pin change ints
          switch (usonar_measSpecs[currentSonarNumber].intRegNum) {
            case 1:
              PCMSK2 = 0;
              // MJ_FIXME1: PCMSK1 = usonar_measSpecs[currentSonarNumber].intBit;
              PCMSK1 =  _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2) ;
              break;
            case 2:
              PCMSK1 = 0;
              // MJ_FIXME1: PCMSK2 = usonar_measSpecs[currentSonarNumber].intBit;
              if (currentSonarNumber == 7) {  // For some reason #7 is better if only done for itself
                PCMSK2 = _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2) | _BV(1) | _BV(0);
              } else {
                PCMSK2 = _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2)          | _BV(0);
              }
              break;
            default:
              // This is REALLY a huge coding issue or problem in usonar_measSpec
              if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
                host_uart->string_print((Text)" Sonar ERROR in code for intRegNum!\r\n");
              }
              break;
          } 
          SREG |= _BV(7);
         
          // Trigger the sonar unit to start measuring and return start time
          sonarMeasTriggerTime = usonar.measTrigger(currentSonarNumber);

          if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            ltoa(sonarMeasTriggerTime, longStr,10);
            host_uart->string_print((Text)" Sonar start Sample: ");
            host_uart->integer_print(currentSonarNumber);
            host_uart->string_print((Text)" at ");
            host_uart->string_print((Text)longStr);
            host_uart->string_print((Text)"us\r\n");
          }

          usonarSampleState = USONAR_STATE_WAIT_FOR_MEAS;
          }
          break;

        case USONAR_STATE_WAIT_FOR_MEAS: {
          // wait max time to ensure both edges get seen then sample for edges
          unsigned long measCycleTime;    // Time so far waiting for this measurement
          unsigned long rightNow;

          rightNow = SYSTEM_GET_MICROSECONDS;

          // Look for counter to go 'around' and ignore this one. 
          // Unsigned math so we can get HUGE number
          if (sonarMeasTriggerTime > rightNow) {
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              host_uart->print((Text)" Sonar system tic rollover in meas wait. \r\n");
            }
            usonarSampleState = USONAR_STATE_MEAS_START;
            break;
          }

          measCycleTime = rightNow - sonarMeasTriggerTime; 

          // If meas timer not done break on through till next pass
          if (measCycleTime < USONAR_MEAS_TIME)     {   
            break;     // Still waiting for measurement
          }


          // OK we think we have measurement data so check for and get edge data
          if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            host_uart->string_print((Text)" Sonar meas from: ");
            ltoa(sonarMeasTriggerTime, longStr,10);
            host_uart->string_print((Text)longStr);
            host_uart->string_print((Text)"us to: ");
            ltoa(rightNow, longStr,10);
            host_uart->string_print((Text)longStr);
            host_uart->string_print((Text)"us\r\n");
          }

          int edgeCount = usonar.getQueueLevel();
          if (edgeCount != 2) {
            // We expect exactly two edges.  If not we abort this meas cycle
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              host_uart->print((Text)" Sonar ERROR! meas saw ");
              host_uart->integer_print(edgeCount);
              host_uart->print((Text)" edges! \r\n");
              // special value as error type indicator but as things mature we should NOT stuff this
              sonarDistancesInMeters[currentSonarNumber] = 
              usonar.echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR1));
            }

            usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

            break;   // break to wait till next pass and do next sensor
          }

          // We clear the individual pin interrupt enable bits now
          //PCMSK1 = 0;
          //PCMSK2 = 0;

          // So lets (FINALLY) get the two edge samples
          unsigned long echoPulseWidth;    // Time between edges
          currentDelayData1 = usonar.pullQueueEntry();    // pull entry OR we get 0 if none yet
          currentDelayData2 = usonar.pullQueueEntry();    // pull entry OR we get 0 if none yet

          echoPulseWidth =  currentDelayData2 - currentDelayData1; // The 'real' meas data in uSec

          if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            host_uart->string_print((Text)" Sonar edges from: ");
            ltoa(currentDelayData1, longStr,10);
            host_uart->string_print((Text)longStr);
            host_uart->string_print((Text)"us to: ");
            ltoa(currentDelayData2, longStr,10);
            host_uart->string_print((Text)longStr);
            host_uart->string_print((Text)"us \r\n");
          }

          if (currentDelayData1 > currentDelayData2) {
            // This is another form of rollover every 70 minutes or so but just 
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              host_uart->print((Text)" Sonar sys tic rollover OR edges reversed\r\n");
              // special value as error type indicator but as things mature we should NOT stuff this
              sonarDistancesInMeters[currentSonarNumber] = 
                usonar.echoUsToMeters((unsigned long)USONAR_ECHO_MAX + (unsigned long)USONAR_ECHO_ERR2);
            }

            usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

          //} else if (echoPulseWidth > USONAR_MEAS_TOOLONG) {  
          //  if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
          //    host_uart->print((Text)" Sonar meas result over the MAX\r\n");
          //  }

          //  // special value as error type indicator but as things mature we should NOT stuff this
          //  sonarDistancesInMeters[currentSonarNumber] = 
          //    usonar.echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR3));
          //  usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

          } else {
              // Save our sample delay AND save our time we acquired the sample
              if (echoPulseWidth > USONAR_ECHO_MAX) {
               // We are going to cap this as a form of non-expected result so can it
                if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
                  char longStr[32];
                  ltoa(echoPulseWidth, longStr,10);
                  host_uart->print((Text)" Sonar echo delay of ");
                  host_uart->print((Text)longStr);
                  host_uart->print((Text)" is over MAX!\r\n");
                }
                // We really should ignore this once system is robust
                // echoPulseWidth = (unsigned long)USONAR_ECHO_MAX + (unsigned long)USONAR_ECHO_ERR4;
                usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle
                break;
              }

              // THIS IS THE REAL AND DESIRED PLACE WE EXPECT TO BE EACH TIME!
              sonarSampleTimes[currentSonarNumber] = currentDelayData2;
              float distanceInMeters = usonar.echoUsToMeters(echoPulseWidth);
              sonarDistancesInMeters[currentSonarNumber] = distanceInMeters;

              if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
                char outBuf2[32];
                float distInCm;
                int   echoCm;
                echoCm = echoPulseWidth / 58;
                distInCm = distanceInMeters * (float)(100.0);
                dtostrf(distInCm, 6, 1, outBuf2);
                host_uart->string_print((Text)" S: ");
                host_uart->integer_print((int)currentSonarNumber);
                host_uart->string_print((Text)" E: ");
                host_uart->integer_print((int)echoCm);
                host_uart->string_print((Text)"cm D: ");
                host_uart->string_print((Text)outBuf2);
                host_uart->string_print((Text)"cm \r\n");
              }
              }
              usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;
            }
          break;

        case USONAR_STATE_POST_SAMPLE_WAIT: {
          // We have included a deadtime so we don't totaly hammer the ultrasound 
          // this will then not drive dogs 'too' crazy
          unsigned long waitTimer;
          unsigned long curTicks;
          curTicks = SYSTEM_GET_MICROSECONDS;

          currentDelayData1 = 0;      // Reset the sample gathering values for this run
          currentDelayData2 = 0;

          waitTimer = curTicks - sonarMeasTriggerTime; 

          if (sonarMeasTriggerTime > curTicks) {   // Unsigned math so we can get HUGE number
            if (system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              host_uart->print((Text)" Sonar system timer rollover in meas spacing.\r\n");
            }
            usonarSampleState = USONAR_STATE_MEAS_START;
          } else if (waitTimer > USONAR_SAMPLES_US) {   
            usonarSampleState = USONAR_STATE_MEAS_START;
          }

          // If we fall through without state change we are still waiting
          }
          break;

        default:
              usonarSampleState = USONAR_STATE_MEAS_START;
        break;
      }
      
      bridge.loop(TEST);
      break;
    } // End case BUS_LOKI_PROGRAM_RAB

  }
}

// *Loki_Motor_Encoder* classes:

Loki_Motor_Encoder::Loki_Motor_Encoder(UByte input1_pin, UByte input2_pin,
 UByte enable_pin, Integer *encoder_pointer) {
  _input1_pin = input1_pin;
  _input2_pin = input2_pin;
  _enable_pin = enable_pin;
  _encoder_pointer = encoder_pointer;
}

void Loki_Motor_Encoder::pwm_set(Byte pwm) {
  static const UShort friction_pwm = 5;
  UByte input1 = LOW;
  UByte input2 = LOW;
  UShort enable_pwm = 0;
  if (pwm > 0) {
    enable_pwm = ((UShort)pwm << 1) + friction_pwm;
    input1 = HIGH;
  } else if (pwm < 0) {
    enable_pwm = (((UShort)(-pwm)) << 1) + friction_pwm;
    input2 = HIGH;
  }
  if (enable_pwm > 255) {
    enable_pwm = 255;
  }

  // Set the direction pins and pulse the output:
  digitalWrite(_input1_pin, input1);
  digitalWrite(_input2_pin, input2);
  analogWrite(_enable_pin, (UByte)enable_pwm);
}

Integer Loki_Motor_Encoder::encoder_get() {
  // Do something here:
  return *_encoder_pointer;
}

void Loki_Motor_Encoder::encoder_set(Integer encoder) {
  // Do something here:
  *_encoder_pointer = encoder;
}

