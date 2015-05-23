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

// for sonar pins also see the usonar_ tables we use to lookup
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
 */
// Sonar number as silkscreened on Loki board
#define USONAR_MAX_UNITS  17    

// System dependent clock for getting microseconds as fast as we can do it
#define SYSTEM_GET_MICROSECONDS    micros()

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
//  usonar_EchoEdgeChanges[]  Holds 30 bit timestamp with lower 2 bits showing the bank that changed
//                            Note that the timestamp will roll over in around 70 min
//  usonar_EchoChangedPins[]  Not used yet!  Is to hold a 1 for each pin that has changed at this timestamp.
//                            Bits 23:0 are to be placed properly by each ISR
//
#define  USONAR_QUEUE_LEN    64            // MUST be a power of 2
#define  USONAR_MIN_EMPTIES   2            // minimum space where producer can insert new entry
unsigned int  usonar_producerIndex = 0;    // Owned by ISRs and only inspected by consumer
unsigned int  usonar_consumerIndex = 0;    // Owned by consumer and only inspected by producer
int usonar_queueOverflow = 0;              // Producer can set this and consumer has to be aware. 

// The Echo timestamp is saved with the least sig 2 bits being set to the channel for this timestamp
// We do not know which bits changed, we only know at least one changed in this bank.
// So this means the sonar cycling needs to be careful to only do one sample at a time from any one bank
unsigned long usonar_EchoEdgeChanges[USONAR_QUEUE_LEN];

//
// Ultrasonic sonar edge detection Background Processing
//
// Consumer of usonar edge detection measurements and must obey laws of the queue
//
unsigned int  usonar_inspectIndex = 0;        // Owned by consumer and unknown to the producer
void usonar_logMeasurements() {
    // TODO!  At this time just log a few as they show up.  THIS WILL BREAK ON QUEUE ROLLOVER !!!!
    if (usonar_producerIndex > usonar_consumerIndex) {
      if (usonar_consumerIndex > 1) {
        host_uart->print((Text)"Ci: ");
        host_uart->integer_print(usonar_consumerIndex);
        host_uart->print((Text)" Pi: ");
        host_uart->integer_print(usonar_producerIndex);
        host_uart->print((Text)" Edge: ");
        host_uart->integer_print((usonar_EchoEdgeChanges[usonar_consumerIndex] - 
                                  usonar_EchoEdgeChanges[(usonar_consumerIndex - 1)]));
        host_uart->print((Text)"\r\n");
      }
 
      usonar_consumerIndex += 1;
    }
}

// *PCINT1_vect*() is one of two ISRs to gather sonar bounce pulse widths
//
// This ISR processes pin changes for Bank 1 which is for PCINT pins 15:8
// This ISR must be LIGHTNING FAST and is bare bones petal-to-the-metal processing!
//
// Pin Change Processing:
// This ISR will take in which pins had changes on one bank of pins
// and then push those changes with an associated timestamp in echoQueue[]
// The echoQueue[] is a circular queue so if the background consumer task
// has left the queue too full this ISR will loose data.
ISR(PCINT1_vect) {
   unsigned long now = SYSTEM_GET_MICROSECONDS; 	// Get clock FAST as we can

   int empties = 0;
   // Ensure there is room in the queue even if we have rollover
   if (usonar_consumerIndex < usonar_producerIndex) {
      empties = usonar_producerIndex = usonar_consumerIndex;
   } else {
      empties = (USONAR_QUEUE_LEN - usonar_producerIndex) + usonar_consumerIndex;
   }

   if (empties >= USONAR_MIN_EMPTIES) {
       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_EchoEdgeChanges[nextProducerIndex]  = (now & 0xfffffffc) | 1;
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

   int empties = 0;
   // Ensure there is room in the queue even if we have rollover
   if (usonar_consumerIndex < usonar_producerIndex) {
      empties = usonar_producerIndex = usonar_consumerIndex;
   } else {
      empties = (USONAR_QUEUE_LEN - usonar_producerIndex) + usonar_consumerIndex;
   }

   if (empties >= USONAR_MIN_EMPTIES) {
       // We can produce this entry into the queue and bump producer index
       unsigned int nextProducerIndex = usonar_producerIndex + 1;
       if (nextProducerIndex >= USONAR_QUEUE_LEN) 
           nextProducerIndex = 0;

       // Since timer resolution is about 8us and this clock is usec we will
       // use the LOWER 2 bits to indicate which bank this change is from
       usonar_EchoEdgeChanges[nextProducerIndex]  = (now & 0xfffffffc) | 2;
       usonar_producerIndex = nextProducerIndex;  // Bump producer index to this valid one
   } else {
       usonar_queueOverflow  += 1;
   }
}

// Utilities to read distance from ultrasonic sonar unit
//

// Tables to map sonar number to trigger digital line # and echo Axx line
// If entry is 0, not supported yet as it does not have digital pin #
// Need a more clever set of code to deal with all the abnormal pins

int usonar_unitToTriggerPin[USONAR_MAX_UNITS] = { 0,
        sonar_trig1_pin, sonar_trig2_pin, sonar_trig3_pin, sonar_trig4_pin, 
        sonar_trig5_pin, sonar_trig6_pin, sonar_trig7_pin, sonar_trig8_pin, 
        sonar_trig9_pin, sonar_trig10_pin, sonar_trig11_pin, sonar_trig12_pin, 
        sonar_trig13_pin, sonar_trig14_pin, sonar_trig15_pin, sonar_trig16_pin };

int usonar_unitToEchoPin[USONAR_MAX_UNITS] = { 0,
        sonar_echo1_pin, sonar_echo2_pin, sonar_echo3_pin, sonar_echo4_pin, 
        sonar_echo5_pin, sonar_echo6_pin, sonar_echo7_pin, sonar_echo8_pin, 
        sonar_echo9_pin, sonar_echo10_pin, sonar_echo11_pin, sonar_echo12_pin, 
        sonar_echo13_pin, sonar_echo14_pin, sonar_echo15_pin, sonar_echo16_pin };

// Trigger the given ultrasonic sonar unit
// The sonar units are 1 - 16 and that is the value to enter.
// This routine shields the caller from knowing the hardware pin
//
// Return value is system clock in microseconds for when the trigger was sent
// Note that the sonar itself will not reply for a few hundred microseconds
//
unsigned long  usonar_trigger(int sonarNumber)
{
    if ((sonarNumber < 1) || (sonarNumber >= USONAR_MAX_UNITS)) {
        return 0;
    }

    int trigPin = usonar_unitToTriggerPin[sonarNumber];

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    return SYSTEM_GET_MICROSECONDS | 1;   // set lsb so zero is unique as error code
}

float usonar_microsecToMeters(unsigned long pingDelay) 
{
    return (float)(pingDelay) / 5820.0;
}

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
float usonar_inlineReadMeters(int sonarNumber) {
    float distInMeters = 0.0;
    unsigned long startTics;

    startTics = usonar_trigger(sonarNumber);    // Trigger the sonar unit to measure
    if (startTics == 0) {
        return -1.0;		// Bad sensor number 
    }
    int echoPin = usonar_unitToEchoPin[sonarNumber];
    if (echoPin == 0) {
        return -2.0;		// sensor number not supported yet
    }

    unsigned long duration = pulseIn(echoPin, HIGH, 90000);
    distInMeters = usonar_microsecToMeters(duration);

    // If we get distance less than spec, return 0 as this can also be a fault
    if (distInMeters < 0.03) {
        distInMeters = 0;
    }

    return distInMeters;
}


/*
 *********************  End UltraSonic Sonar Code ********************************************
*/



// The *setup*() routine runs once when you press reset:
void setup() {
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

      // Set up Interrupt on Pin Change interrupt vector.  The encoder
      // pins are attached to PCINT7/6/5/4, so we only need to set
      // PCMSK0 to '1111 0000':
      PCMSK0 = _BV(7) | _BV(6) | _BV(5) | _BV(4);

      // Now enable interrup on changes for PCINT7/.../0, by setting
      // PCICR to 1.  Thus, PCICR is set to 'xxxx x001' or '0000 0001':
      PCICR = _BV(0);

      // Set up for interrupts on Bank 1 pin changes or pins 15:8
      // These are used for ultrasonic sonar units mostly in back of Loki
      // Units:    9       10       11       12    13,14   15,16
      // ChipPin: 69       68       67       66       65      64
      // PCINT:   15       14       13       12       11      10
      PCMSK1 =  _BV(7) |         _BV(5) | _BV(4) | _BV(3)         ;
      PCICR |= _BV(1);

      // Set up for interrupts on Bank 2 pin changes or pins 23:16
      // These are used for ultrasonic sonar units mostly in front of Loki
      // Units:    1        2        3        4        5        6       7        8  
      // ChipPin: 82       83       84       85       86       87      88       89
      // PCINT:   23       22       21       20       19       18      17       16
      // PCMSK2 = _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2)| _BV(1) | _BV(0);
      PCMSK2 = _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2)                 ;
      //PCMSK2 =                                     _BV(3) | _BV(2)                 ;
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

      // Deal with ultrasonic sonar completions
      // Not ready for prime time but getting real close ...  20150523  mjstn usonar_logMeasurements();

      bridge.loop(TEST);
      break;
    }
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

