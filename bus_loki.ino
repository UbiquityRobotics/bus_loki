// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Frame_Buffer.h>
#include <bus_server.h>
#include <Sonar.h>
#include <RAB_Sonar.h>

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

// Trigger and readback delay from an HC-SR04 Ulrasonic Sonar, return in meters
extern float usonar_inlineReadMeters(int sonarUnit);

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

class Loki_RAB_Sonar : RAB_Sonar {
 public:
  Loki_RAB_Sonar(UART *debug_uart);
  virtual Short ping_get(UByte sonar);
  virtual Short system_debug_flags_get();
  virtual void system_debug_flags_set(Short system_flags);
  virtual UByte sonars_count_get();
 private:
  Short system_debug_flags_;
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
Loki_RAB_Sonar loki_rab_sonar((UART *)&debug_uart);

Bridge bridge(&avr_uart0, &avr_uart1, &avr_uart0, &bus_slave,
 (Bus_Motor_Encoder *)&left_motor_encoder,
 (Bus_Motor_Encoder *)&right_motor_encoder,
 (RAB_Sonar *)&loki_rab_sonar);

static Sonar usonar((UART *)debug_uart, (RAB_Sonar *)&loki_rab_sonar);

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
// These tables are organized by sonar number and not by measurement cycle number
// Also the index is the sonar number so entry [0] is zero


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
unsigned long usonar_echoEdgeQueue[USONAR_QUEUE_LEN];
unsigned long usonar_echoInfoQueue[USONAR_QUEUE_LEN];

#define  USONAR_MIN_EMPTIES   2            // minimum space where producer can insert new entry
unsigned int  usonar_producerIndex = 0;    // Owned by ISRs and only inspected by consumer
unsigned int  usonar_consumerIndex = 0;    // Owned by consumer and only inspected by producer
int usonar_queueOverflow = 0;              // Producer can set this and consumer has to be aware. 

// Encapsulated hardware specifics for Loki Platform
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
//  The index to this table is meant to be the measurement cycle number of our task 
//
//  Note that the code should work if you shuffle this table.
//  You may wish to shuffle the table to get faster updates around each side
//  sort of like how you tighten bolts on a wheel by doing the one across the center
//
const Usonar_Meas_Spec  usonar_measSpecs[USONAR_MAX_SPECS] = { 
    {  1, US_MEAS_METHOD_T01_PG2,   sonar_trig1_pin,  sonar_echo1_pin,  2,  _BV(7) },
    {  2, US_MEAS_METHOD_PIN_PCINT, sonar_trig2_pin,  sonar_echo2_pin,  2,  _BV(6) },
    {  3, US_MEAS_METHOD_PIN_PCINT, sonar_trig3_pin,  sonar_echo3_pin,  2,  _BV(5) },
    {  4, US_MEAS_METHOD_PIN_PCINT, sonar_trig4_pin,  sonar_echo4_pin,  2,  _BV(4) },
    {  5, US_MEAS_METHOD_PIN_PCINT, sonar_trig5_pin,  sonar_echo5_pin,  2,  _BV(3) },
    {  6, US_MEAS_METHOD_PIN_PCINT, sonar_trig6_pin,  sonar_echo6_pin,  2,  _BV(2) },
    {  7, US_MEAS_METHOD_PIN_PCINT, sonar_trig7_pin,  sonar_echo7_pin,  2,  _BV(1) },
    {  8, US_MEAS_METHOD_PIN_PCINT, sonar_trig8_pin,  sonar_echo8_pin,  2,  _BV(0) },
    {  9, US_MEAS_METHOD_PIN_PCINT, sonar_trig9_pin,  sonar_echo9_pin,  1,  _BV(7) },
    { 10, US_MEAS_METHOD_T10_PJ7,   sonar_trig10_pin, sonar_echo10_pin, 1,  _BV(6) },
    { 11, US_MEAS_METHOD_PCINT,     sonar_trig11_pin, sonar_echo11_pin, 1,  _BV(5) },
    { 12, US_MEAS_METHOD_NONE,      sonar_trig12_pin, sonar_echo12_pin, 1,  _BV(4) },
    { 13, US_MEAS_METHOD_PCINT,     sonar_trig13_pin, sonar_echo13_pin, 1,  _BV(2) },
    { 14, US_MEAS_METHOD_PCINT,     sonar_trig14_pin, sonar_echo14_pin, 1,  _BV(2) },
    { 15, US_MEAS_METHOD_T15_PG4,   sonar_trig15_pin, sonar_echo15_pin, 1,  _BV(3) },
    { 16, US_MEAS_METHOD_T16_PG3,   sonar_trig16_pin, sonar_echo16_pin, 1,  _BV(3) }
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
   unsigned long now = USONAR_GET_MICROSECONDS; 	// Get clock FAST as we can

#ifndef USONAR_ULTRA_FAST_ISR
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
#endif

       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_echoEdgeQueue[usonar_producerIndex]  = (now & 0xfffffffc) | 1;
       usonar_producerIndex = nextProducerIndex;  // Bump producer index to this valid one

#ifndef USONAR_ULTRA_FAST_ISR
   } else {
       usonar_queueOverflow  += 1;
   }
#endif
}

// *PCINT2_vect*() is one of two ISRs to gather sonar bounce pulse widths
//
// This ISR processes pin changes for Bank 2 which is for PCINT pins 23:16
// This ISR must be LIGHTNING FAST and is bare bones petal-to-the-metal processing!
//
// Please see PCINT1_vect() for comments on the details of Pin Change processing
// 
ISR(PCINT2_vect) {
   unsigned long now = USONAR_GET_MICROSECONDS; 	// Get clock FAST as we can

#ifndef USONAR_ULTRA_FAST_ISR
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
#endif

       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_echoEdgeQueue[usonar_producerIndex]  = (now & 0xfffffffc) | 2;
       usonar_producerIndex = nextProducerIndex;  // Bump producer index to this valid one

#ifndef USONAR_ULTRA_FAST_ISR
   } else {
       usonar_queueOverflow  += 1;
   }
#endif
}


/*
 *********************  End UltraSonic Sonar Code ********************************************
*/


Loki_RAB_Sonar::Loki_RAB_Sonar(UART *debug_uart) : RAB_Sonar(debug_uart) {
  system_debug_flags_ = 0;
}

Short Loki_RAB_Sonar::ping_get(UByte sonar) {
  // FIXME: Do this in fixed point!!!
  return (Short)(usonar.getLastDistInMm(sonar)/(float)(10.0) + (float)(0.5));
}

Short Loki_RAB_Sonar::system_debug_flags_get() {
  return system_debug_flags_;
}

void Loki_RAB_Sonar::system_debug_flags_set(Short system_debug_flags) {
  system_debug_flags_ = system_debug_flags;
}

UByte Loki_RAB_Sonar::sonars_count_get() {
  return 16;
}

// Setup instance of class for Loki Ultrasonic Sensor support
// The *setup*() routine runs once when you press reset:
void setup() {

  usonar_producerIndex = 0;
  usonar_consumerIndex = 0;

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
      PCMSK1 = 0;
      PCICR |= _BV(1);

      // Set up for interrupts on Bank 2 pin changes or pins 23:16
      // These are used for ultrasonic sonar units mostly in front of Loki
      // The Pin Change interrupts are enabled for each bit on Bank 1 
      // As that pin is used in the measurement sampling code, not here
      PCMSK2 = 0;
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

      usonar.poll();
      
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
  static const UShort friction_pwm = 1;    // Was 5 for yellow motors
  UByte input1 = LOW;
  UByte input2 = LOW;
  UShort enable_pwm = 0;

  // We convert the signed 8-bit value for PWM range of 0-255
  // and setup the motor direction bits also from sign of input value
  if (pwm > 0) {
    enable_pwm = ((UShort)pwm << 1) + friction_pwm;
    input1 = HIGH;
  } else if (pwm < 0) {
    enable_pwm = (((UShort)(-pwm)) << 1) + friction_pwm;
    input2 = HIGH;
  }
  if (enable_pwm > 255) {  
    enable_pwm = 255;       // Cap PWM value to 255
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

