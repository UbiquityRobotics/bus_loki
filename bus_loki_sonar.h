// This file is indended to be specific to Loki platform
//

// We require these externally defined indexes due to need for super fast ISR
#define  USONAR_QUEUE_LEN     8             // MUST be a power of 2
extern unsigned long usonar_echoEdgeQueue[USONAR_QUEUE_LEN];
extern unsigned long usonar_echoInfoQueue[USONAR_QUEUE_LEN];
extern unsigned int  usonar_producerIndex;    // Owned by ISRs and only inspected by consumer
extern unsigned int  usonar_consumerIndex;    // Owned by consumer and only inspected by producer

// System dependent clock for getting microseconds as fast as we can do it
#define USONAR_GET_MICROSECONDS    micros()


// Define max number of sonar units used in this system
#define USONAR_MAX_UNITS    17    // Sonar number as silkscreened on Loki board

// Define the parameters for measurement of Sonar Units
//
// 30000 is about 5 meters
#define USONAR_ECHO_MAX       ((long)(28100))   // Longest echo time we expect (28100 is 5  meters)
#define USONAR_MEAS_TIME     ((long)(150000))   // Time to wait per measurement
#define USONAR_MEAS_TOOLONG   ((long)(70000))   // Measurement itself was too long
#define USONAR_SAMPLES_US    ((long)(200000))   // One measurement each time this many uSec goes by
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

// We found that the nice producer consumer queue has to be reset down wo
// just do one meas per loop and reset queue each time so we get 2 edges
#define  USONAR_ULTRA_FAST_ISR

// Tables to map sonar number to trigger digital line # and echo Axx line
// If entry is 0, not supported yet as it does not have digital pin #
// Need a more clever set of code to deal with all the abnormal pins
typedef struct usonar_meas_spec_t {
    int measMethod;     // Method to be used for the measurement
    int trigPin;        // For direct pin this is digital pin number for custom it a routine to use
    int echoPin;        // The echo pin in terms of arduino pin to use for pulseIn
    int intRegNum;      // Int reg number for pinint interrupt enable
    int intBit;         // the bit for enable of interrupts for this pinint
} Usonar_Meas_Spec;
extern const Usonar_Meas_Spec  usonar_measSpecs[];

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



// We define pins used on Loki Adruino side platform for Sonar functionalities

// for sonar pins also see the usonar tables we use to lookup
static const int sonar_echo1_pin  = A15;        // IC Pin 82
static const int sonar_echo2_pin  = A14;        // IC Pin 83
static const int sonar_echo3_pin  = A13;        // IC Pin 84
static const int sonar_echo4_pin  = A12;        // IC Pin 85
static const int sonar_echo5_pin  = A11;        // IC Pin 86
static const int sonar_echo6_pin  = A10;        // IC Pin 87
static const int sonar_echo7_pin  = A9;         // IC Pin 88
static const int sonar_echo8_pin  = A8;         // IC Pin 89
static const int sonar_echo9_pin  = 0;          // IC Pin 69
static const int sonar_echo10_pin = 0;          // IC Pin 68
static const int sonar_echo11_pin = 0;          // IC Pin 67
static const int sonar_echo12_pin = 0;          // IC Pin 66
static const int sonar_echo13_pin = 0;          // IC Pin 65
static const int sonar_echo14_pin = 0;          // IC Pin 65
static const int sonar_echo15_pin = 0;          // IC Pin 64
static const int sonar_echo16_pin = 0;          // IC Pin 64

static const int sonar_trig1_pin  = 39;         // IC Pin 70
static const int sonar_trig2_pin  = 22;         // IC Pin 78
static const int sonar_trig3_pin  = 23;         // IC Pin 77
static const int sonar_trig4_pin  = 24;         // IC Pin 76
static const int sonar_trig5_pin  = 25;         // IC Pin 75
static const int sonar_trig6_pin  = 26;         // IC Pin 74
static const int sonar_trig7_pin  = 27;         // IC Pin 73
static const int sonar_trig8_pin  = 28;         // IC Pin 72
static const int sonar_trig9_pin  = 29;         // IC Pin 71
static const int sonar_trig10_pin = 0;          // IC Pin 79
static const int sonar_trig11_pin = 46;         // IC Pin 38
static const int sonar_trig12_pin = 47;         // IC Pin 37
static const int sonar_trig13_pin = 48;         // IC Pin 36
static const int sonar_trig14_pin = 49;         // IC Pin 35
static const int sonar_trig15_pin = 0;          // IC Pin 29
static const int sonar_trig16_pin = 0;          // IC Pin 28



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
    int localPI = usonar_producerIndex;         // get atomic copy of producer index

    return calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN);
  };

  //
  // Pull one entry from our edge detection circular queue if there are entries.
  // A return of 0 will happen if no entries are ready to pull at this time OR the entry is 0
  //
  unsigned long pullQueueEntry() {
    int localPI = usonar_producerIndex;         // get atomic copy of producer index
    unsigned long queueEntry = 0;

    if (calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN) > 0) {

      queueEntry = usonar_echoEdgeQueue[usonar_consumerIndex];

      // Find the next index we will bump the consumer index to once done
      int nextCI = usonar_consumerIndex + 1;
      if (nextCI >= USONAR_QUEUE_LEN) {
        nextCI = 0;     // case of roll-around for next entry
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

  // Get the Pin change Interrupt Bank number (int mask reg number)
  //
  // Negative value indicates unsupported unit number
  int getInterruptMaskRegNumber(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }
    return usonar_measSpecs[sonarUnit].intRegNum;
  };

  // Get the interrupt enable bit for this sonar unit
  //
  // Negative value indicates unsupported unit number
  int getInterruptBit(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }
    return usonar_measSpecs[sonarUnit].intBit;
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

  // Fetch the measurement spec for this sonar
  //
  // A return of < 0 indicates bad sonar unit number
  int getMeasSpec(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit >= _numSonars)) {
        return -1;
    }

    return usonar_measSpecs[sonarUnit].measMethod;
  }


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
    triggerTime = USONAR_GET_MICROSECONDS | 1;   // set lsb, zero is an error code

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
        return -2.0;            // sensor number not supported yet for inline read
    }

    startTics = measTrigger(sonarUnit);    // Trigger the sonar unit to measure
    if (startTics == 0) {
        return -1.0;            // Bad sensor number or not supported yet
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



