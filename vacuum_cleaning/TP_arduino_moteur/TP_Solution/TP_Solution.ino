/*
 * A possible solution for the motor control exercise.
 * 
 * Setup: a DC motor is connected to pin 2 of the Arduino through a RC-Servo -> DC motor control.
 * This means that motor input is a RC servo signal.
 * An quadrature encoder with resolution 360 ticks/turn is mounted on the motor.
 * 
 * A PI controller is used to servo the motor to the desired fixed speed, 300 ticks/s.
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

// Last time since the control code was called.
unsigned long lastTime;
// Last encoder value when control code was called.
int lastEncoderCount = 0;

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

  lastTime = micros();
}

// Proportional and integral gain, hand-tuned.
float const Kp = 0.001; 
float const Ki = 0.002; 
// Target speed, in ticks / s.
float const targetVelocity = 300;

float targetEncoderCount = 0;
void loop() 
{
  // Get time elapsed since last call.
  float dt = (micros() - lastTime) / (1.0e6);
  lastTime = micros();
  // Compute integral of target.
  targetEncoderCount += dt * targetVelocity;
  // Compute current velocity.
  float velocity = (encoderCount - lastEncoderCount) / dt;
  lastEncoderCount = encoderCount;

  // Motor PI controller.
  float u = - Kp * (velocity - targetVelocity) - Ki * (encoderCount - targetEncoderCount);
  moveMotor(u);
  // Send the current velocity to the PC
  Serial.println(velocity);
 
  // Sleep: do update at 50Hz only, since a servo RC signal is not faster than that.
  delay(20);
}



