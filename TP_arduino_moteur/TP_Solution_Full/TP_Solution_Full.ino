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

// Proportional gain
// Tuning of the PI loop was done using Ziegler–Nichols method. Critical Kp gain: 0.0016 ; period Tu = 151ms
// Note that this was done using a delay(25): faster loops could not be made to reach instability, I think due to the low encoder resolution.
float const Kp = 0.00072; 
float const Ki = 0.005721854; 

// Value of the integral.
float integral = 0;

// Integral anti-windup
float maxIntegral = 0.8;

// Target speed, in ticks / s.
float targetVelocity = 0;

// User input parsing
String userInput = "";
void loop() 
{
  // Get current time.
  unsigned long startTime = micros();
  
  // Get time elapsed since last call.
  float dt = (startTime - lastTime) / (1.0e6);
  lastTime = startTime;

  // Compute velocity.
  float velocity = (encoderCount - lastEncoderCount) / dt;
  lastEncoderCount = encoderCount;

  float error = (velocity - targetVelocity);
  // Update integral.
  integral += dt * error;
  // Clamp integral
  if (Ki * integral > maxIntegral)
    integral = maxIntegral / Ki;
  if (Ki * integral < -maxIntegral)
    integral = -maxIntegral / Ki;

  // Motor P controller.
  float u = - Kp * error - Ki * integral;
  moveMotor(u);
  // Send the current velocity to the PC
  Serial.println(velocity);
  
  // Read serial port to see if we have new user input.
  while (Serial.available())
  {
    char c = Serial.read();
    if (isDigit(c)) 
      userInput += c;
    if (c == '-' && userInput == "")
      userInput = "-";
    if (c == '\n') 
    {
      targetVelocity = userInput.toInt();
      userInput = "";
      // Reset integral when a new target is received
      integral = 0;
    }
  }
 
  // Sleep: do update at 50Hz only, since a servo RC signal is not faster than that.
  // To make sure the update rate is right, sleep only by the remaining amount.
  unsigned long elapsedTime =  micros() - startTime;
  if (elapsedTime < 20000)
    delayMicroseconds(20000 - elapsedTime);
}



