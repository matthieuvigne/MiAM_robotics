/*
 * Template for the motor control exercise.
 * 
 * Setup: a DC motor is connected to pin 2 of the Arduino through a RC-Servo -> DC motor control.
 * This means that motor input is a RC servo signal.
 * An quadrature encoder with resolution 360 ticks/turn is mounted on the motor, and plugged in port 6 and 7.
 * 
 * The first objective is to servo the motor to a fixed speed, 500 ticks /s.
 * The next step is then to servo the motor at a user-defined speed, through the serial port.
 * 
 * This template gives the encoder reading code, and the motor communication framework. The rest is up to you...
 *  
 */

// Include the servo library.
#include <Servo.h>

Servo motor;

/// Send power to the motor, normalized between -1 and 1 (-1: full reverse, 0: stop, 1: full forward).
void moveMotor(float motorPower)
{
  if (motorPower < -1)
    motorPower = -1;
  if (motorPower > 1)
    motorPower = 1;
  motor.writeMicroseconds(1500 + 500 * motorPower);
}
// This variable stores the current value of the encoder.
// Note: as we count up, the int may overflow! With the current encoder, this happens after 91 turns.
int encoderCount = 0;

// Interrupt-based encoder counter, see EncoderReading for more details.
#define PINA 6
#define PINB 7

bool oldB = false;
ISR(PCINT2_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PIND & (1 << PINA);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount += (oldB ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB =  PIND & (1 << PINB);
}

unsigned long last_time = 0;
int last_encoder_count = 0;

void setup() 
{
  // Setup serial communication with the PC.
  Serial.begin(115200);
  
  // Setup encoder
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << PINA;
  PCMSK2 |= 1 << PINB;
  PCICR = 0b100;
  
  // Setup motor port
  motor.attach(2);

  last_time = micros();
  last_encoder_count = 0;
}


float reference_velocity = 0;
float Kp = 1.0/200.0;


void loop() 
{
  unsigned long current_time = micros();

  float velocity = 1e6 * (encoderCount - last_encoder_count) / ((float)(current_time - last_time));
  
  last_time = current_time;
  last_encoder_count = encoderCount;

  
  float target = - Kp * (velocity - reference_velocity);

  
  // TODO: do motor velocity servoing...
  moveMotor(target);

  // Send the current encoder position to the Arduino.
  // TODO: replace this by velocity output instead of position output.
  Serial.println(velocity);

  delay(20);
}



