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
static const int motor1_input1_pin = 43;
static const int motor1_input2_pin = 44;
static const int motor1_enable_pin = 45;
static const int motor2_input1_pin = 5;
static const int motor2_input2_pin = 3;
static const int motor2_enable_pin = 2;
static const int sonar_echo5_pin = A11;		// IC Pin 86
static const int sonar_echo6_pin = A10;		// IC Pin 87
static const int sonar_echo7_pin = A9;		// IC Pin 88
static const int sonar_echo8_pin = A8;		// IC Pin 89
static const int sonar_trig5_pin = 25;		// IC Pin 75
static const int sonar_trig6_pin = 26;		// IC Pin 74
static const int sonar_trig7_pin = 27;		// IC Pin 73
static const int sonar_trig8_pin = 28;		// IC Pin 72

// Define the UART's:
NULL_UART null_uart;
AVR_UART *bus_uart = &avr_uart1;
AVR_UART *debug_uart = &avr_uart0;
AVR_UART *host_uart = &avr_uart0;

Bus_Motor_Encoder left_motor_encoder, right_motor_encoder;
Bus_Slave bus_slave((UART *)bus_uart, (UART *)host_uart);
Bridge bridge(&avr_uart0, &avr_uart1, &avr_uart0, &bus_slave,
  &left_motor_encoder, &right_motor_encoder);

void leds_byte_write(char byte) {
    digitalWrite(led0_pin, (byte & 1) ? LOW : HIGH);
    digitalWrite(led1_pin, (byte & 2) ? LOW : HIGH);
    digitalWrite(led2_pin, (byte & 4) ? LOW : HIGH);
    digitalWrite(led3_pin, (byte & 8) ? LOW : HIGH);
    digitalWrite(led4_pin, (byte & 16) ? LOW : HIGH);
    digitalWrite(led5_pin, (byte & 32) ? LOW : HIGH);
    digitalWrite(led6_pin, (byte & 64) ? LOW : HIGH);
    digitalWrite(led7_pin, (byte & 128) ? LOW : HIGH);
}


// The *setup*() routine runs once when you press reset:
void setup() {                
  // Initialize pin directions:
  pinMode(encoder_r1_pin, INPUT);
  pinMode(encoder_r2_pin, INPUT);
  pinMode(encoder_l1_pin, INPUT);
  pinMode(encoder_r2_pin, INPUT);
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
  pinMode(sonar_echo5_pin, INPUT);
  pinMode(sonar_echo6_pin, INPUT);
  pinMode(sonar_echo7_pin, INPUT);
  pinMode(sonar_echo8_pin, INPUT);
  pinMode(sonar_trig5_pin, OUTPUT);
  pinMode(sonar_trig6_pin, OUTPUT);
  pinMode(sonar_trig7_pin, OUTPUT);
  pinMode(sonar_trig8_pin, OUTPUT);

  leds_byte_write(0);
  host_uart->begin(16000000L, 115200L, (Character *)"8N1");
  //host_uart->print((Text)"Double echo\r\n");

  switch (BUS_LOKI_PROGRAM) {
    case BUS_LOKI_PROGRAM_RAB: {
      host_uart->print((Text)"Start bridge setup\r\n");
      bridge.setup(TEST);
      host_uart->print((Text)"Bridge setup done\r\n");
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
      char encoder_r1 = digitalRead(encoder_r1_pin);
      char encoder_r2 = digitalRead(encoder_r2_pin);
      char encoder_l1 = digitalRead(encoder_l1_pin);
      char encoder_l2 = digitalRead(encoder_l2_pin);

      // Write the encoder values:
      digitalWrite(led0_pin, encoder_r1);
      digitalWrite(led1_pin, encoder_r2);
      digitalWrite(led3_pin, encoder_l1);
      digitalWrite(led4_pin, encoder_l2);
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
      bridge.loop(TEST);
      break;
    }
  }
}
