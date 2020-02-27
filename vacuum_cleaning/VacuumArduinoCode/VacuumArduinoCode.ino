/*
   Motor control code for the Arduino.

   The arduino is responsible for handling the three motors, the three encoders, and performing communication with the Raspberry.
   Encoder reading is done using interrupts ; likewise, timers are used to generate a steady signal for the motors.

  To make things a bit easier, here is the pinout for the Arduino Uno, with pin position mapping the Diymore Arduino

       B5 B4 B3 B2 B1 B0
       13 12 11 10  9  8 
       -- -- -- -- -- --
  C0 14|               | 7 D7
  C1 15|               | 6 D6
  C2 16|               | 5 D5
  C3 17|               | 4 D4
  C4 18|               | 3 D3
  C5 19|               | 2 D2
       |               | 1 D1 TX
       |               | 0 D0 RX


*/

/////////////////////////////////////////////////////////
// Pinout.
/////////////////////////////////////////////////////////


// Motor 0
#define PWM_MOTOR_0 OCR1A    // PWM: OCR1A, pin 9
#define DIRECTION_MOTOR_0 16 // Direction: pin 16
// Encoder pin IDs correspond to ID in pin port.
// Here using port Port C
#define PINA_ENCODER_0 0     //C0 -> 14
#define PINB_ENCODER_0 1     //C1 -> 15

// Motor 1
#define PWM_MOTOR_1 OCR1B    // PWM: OCR1B, pin 10
#define DIRECTION_MOTOR_1 7  // Direction: pin 7
#define PINA_ENCODER_1 4     //B4 ->12
#define PINB_ENCODER_1 5     //B5 ->13

// Motor 2
#define PWM_MOTOR_2 OCR2A    // PWM: ORC2A -> pin 11
#define DIRECTION_MOTOR_2 8  // Direction: pin 8
#define PINA_ENCODER_2 5     //D5 -> 5
#define PINB_ENCODER_2 6     //D6 -> 6



/////////////////////////////////////////////////////////
// Global variables.
/////////////////////////////////////////////////////////

// Encoder reading
int encoderCount[3] = {0, 0, 0};
int lastEncoderCount[3] = {0, 0, 0};
bool oldB[3] = {false, false, false};

// Messages
#define MESSAGE_LENGTH 7
boolean lastWasFF = false;
unsigned char message[MESSAGE_LENGTH];
int positionInMessage;

// Motor states
int targetVelocity[3] = {0, 0, 0};

// Controllers
float integralError[3] = {0., 0., 0.};
float const Kp[3] = {10.0 / 90000.0, 10.0 / 90000.0, 10.0 / 90000.0};
float const Ki[3] = {10.0 / 5000.0, 10.0 / 5000.0, 10.0 / 5000.0};

// Define period of the control loop, in ms.
#define CONTROL_PERIOD_MS 5

/////////////////////////////////////////////////////////
// Encoder reading interrupt.
/////////////////////////////////////////////////////////
// Encoder 0
ISR(PCINT1_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PINC & (1 << PINA_ENCODER_0);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount[0] += (oldB[0] ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB[0] =  PINC & (1 << PINB_ENCODER_0);
}

// Encoder 1
ISR(PCINT2_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PIND & (1 << PINA_ENCODER_2);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount[2] += (oldB[2] ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB[2] =  PIND & (1 << PINB_ENCODER_2);
}

// Encoder 2
ISR(PCINT0_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PINB & (1 << PINA_ENCODER_1);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount[1] += (oldB[1] ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB[1] =  PINB & (1 << PINB_ENCODER_1);
}

/////////////////////////////////////////////////////////
// Motor control.
/////////////////////////////////////////////////////////

// Send power to a motor, along a normalized scale between -1 and 1 (-1: full reverse, 0: stop, 1: full forward).
void moveMotor(int motor_id, float motorPower)
{
  if (motorPower < -1)
    motorPower = -1;
  if (motorPower > 1)
    motorPower = 1;

  bool motorDirection = motorPower > 0;


  if (motor_id == 0)
  {
    digitalWrite(DIRECTION_MOTOR_0, motorDirection);
    PWM_MOTOR_0 = floor(abs(motorPower) * ICR1);
  }
  else if (motor_id == 1)
  {
    digitalWrite(DIRECTION_MOTOR_1, motorDirection);
    PWM_MOTOR_1 = floor(abs(motorPower) * ICR1);
  }
  else
  {
    digitalWrite(DIRECTION_MOTOR_2, motorDirection);
    PWM_MOTOR_2 = (abs(motorPower) * 255);
  }
}

// Control loop
// This function is called at a fixed frequency, every 2ms.
ISR(TIMER0_COMPA_vect) 
{
  static unsigned char controlIteration = 0;
  static unsigned long lastTime = micros();
  static float velocity[3];
  
  controlIteration ++;
  if (controlIteration == CONTROL_PERIOD_MS)
  {
    unsigned long currentTime = micros();
  
    float dt = (float)(currentTime - lastTime) / 1e6;
    lastTime = currentTime;
    Serial.println(1000 * dt);
    controlIteration = 0;
    for (int i = 0; i < 3; i++)
    {
      // Compute current velocity.
      velocity[i] = (encoderCount[i] - lastEncoderCount[i]) / dt;
      lastEncoderCount[i] = encoderCount[i];
  
      // Run PI controller
      float error = (velocity[i] - targetVelocity[i]);
      integralError[i]  += error * dt;
      float u = - Kp[i] * error - Ki[i] * integralError[i];
  
      moveMotor(i, u);
    }
  
    // Send data
    Serial.write(0xFF);
    Serial.write(0xFF);
    // Compute checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 3; i++)
    {
      // Send velocity, use its 2s complement with 2^15
      int velocityInt = (1 << 15) - int(velocity[i]);
      Serial.write((velocityInt >> 8) & 0xFF);
      Serial.write(velocityInt & 0xFF);
      checksum += (velocityInt >> 8) & 0xFF;
      checksum += velocityInt & 0xFF;
    }
    Serial.write(checksum);
  }

  
}

//=============================================================
// SETUP + MAIN LOOP
//=============================================================

void setup()
{
  // Disable all interrupts during config.
  cli();
  // Setup serial communication with the PC.
  Serial.begin(115200);

  // Setup direction pin
  pinMode(DIRECTION_MOTOR_0, OUTPUT);
  pinMode(DIRECTION_MOTOR_1, OUTPUT);
  pinMode(DIRECTION_MOTOR_2, OUTPUT);

  // Setup encoder 0
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  PCMSK1 = 0x00;
  PCMSK1 |= 1 << PINA_ENCODER_0;
  PCMSK1 |= 1 << PINB_ENCODER_0;
  digitalWrite(14, HIGH);
  digitalWrite(15, HIGH);

  // Setup encoder 1
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << PINA_ENCODER_1;
  PCMSK2 |= 1 << PINB_ENCODER_1;
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  // Setup encoder 2
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  PCMSK0 = 0x00;
  PCMSK0 |= 1 << PINA_ENCODER_2;
  PCMSK0 |= 1 << PINB_ENCODER_2;
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);

  // Activate all interrupts
  PCICR = 0b111;

  // For hardware PWM, we now need to do the full setup.

  // I will here rephrase Chapter 13 describing the working of TIMER1 and aossicated functions, talking only about the parts related to our use case.
  // All TIMERS can be used for several functions, here, we are interested in the waveform generator function. Two pins are associated outputs
  // for TIMER1: they are called OC1A and OC1B (likewise OC0A and OC0B are associated to TIMER0...). These pins can be configured to act as
  // signal generators based on this timer (otherwise they are normal GPIOs). Taking a look at the pinout on page 2, we see that
  // OC1A is located on pin PB1 - this corresponds to pin9 on the Arduino. So first, we will set this pin as output.
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Now we must configure the signal to output on OC1A. The working principle is as follow: internally, TIMER1 has a 16-bit counter (i.e. it counts upward from 0 to 65535)
  // To generate a signal, a specific register OCR1A (output compare register for port 1A) gives a treshold value. We then set the output to toogle on the threshold:
  // this means that, when the counter is below the threshold, OC1A is set to 1 (high, i.e. 5V), and when it is above, the output becomes 0.
  // Thus, by changing the value of OCR1A, we can change the duty cycle, i.e. the amount of time that the signal spends high.
  // Now we need a way to change the frequency. This can be done through two mechanisms:
  // - first, changing how high the counter goes. Instead of counting up to 65535, we can decide to loop before: this is determined by the ICR1 register.
  // - second, we can choose at what speed the timer goes up. This timer is directly connected to the 16Mhz oscillator on the Arduino board. However, we can choose a
  // multiplicative coefficient, called a prescaler, to devide this frequency by a certain power of two. This is done using the TCCR1 register.
  // Basically ICR1 allows for small, almost continuous change in the frequency, whereas the prescaler is useful for brutal variations. Note that a low value of
  // ICR1 means less resolution (because OCR1A has to be smaller than ICR1), so it's important to choose the right prescaler to have maximum range.
  // Finally, OC1A and OC1B can have different duty cycle, through the OCR1A and OCR1B registers, but they share the same frequency (which is suitable for most cases,
  // motor control for instance).
  // As a final detail, note that TIMER1 is the only one where you can set the frequency continuously: for the other timers, you can only use the prescaler (i.e. there is no ICR0).
  // This is not really a problem for motor control, as you don't need to vary the frequency: using a prescaler of 2 for instance gives a signal at 31kHz, perfect for motor control

  // Let's put all this in practice.
  // The clock frequency is 16Mhz. Counting up to 65535 gives a (16000000 / 65536) = 244Hz frequency. So we don't need a specific preescaler in this case, and we just set it to 1.
  // We first configure OC1A in the correct mode (fast PWM, with ICR1 as TOP value): see page 108 onward.
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011001;

  // Now we need to set ICR1A for a 1kHz signal.
  // The frequency is given by CLK / ICR1A, so we have ICR1A = CLK/1kHz = 16000
  ICR1 = 15999;

  // Duty cycle of 10%: OCR1A is 10% of ICR1A
  OCR1A = 1600;
  OCR1B = 1600;

  // And that's it: now the signal is being generated, and there is no more code running !

  // Additonnaly, imagine you now want a signal at 10kHz, you just need to do:
  // ICR1A = 1600
  // OCR1A = 160

  TCCR2A = 0b10000011;
  // TCCR2A |= (1 << WGM21);
  // Set to CTC Mode
  // TIMSK2 |= (1 << OCIE2A);
  //Set interrupt on compare match
  TCCR2B |= (1 << CS20);
  // set prescaler to 64 and starts PWM
  // OCR2A = 127;

  lastEncoderCount[0] = 0;
  lastEncoderCount[1] = 0;
  lastEncoderCount[2] = 0;

  PWM_MOTOR_0 = 0;
  PWM_MOTOR_1 = 0;
  PWM_MOTOR_2 = 0;

  // Generate time-based interrupt every ms.
  TIMSK0 = (1 << OCIE0A) | (1 << TOV0);
  
  // Enable all interrupts.
  sei();
}



void loop()
{
  // Read message from host.
  while (Serial.available())
  {
    unsigned char lastData = Serial.read();
    if (lastData == 0xFF && lastWasFF)
      positionInMessage = 0;
    else
    {
      lastWasFF = lastData == 0xFF;

      // If we are currently reading a message, add it to the buffer.
      if (positionInMessage > -1)
      {
        message[positionInMessage] = lastData;
        positionInMessage ++;
      }
      // If the end of a message was reached, decode it.
      if (positionInMessage == MESSAGE_LENGTH)
      {
        // Reset status.
        lastWasFF = false;
        positionInMessage = -1;

        // Verify checksum.
        // Sum of previous bytes must be equal to the checksum.
        unsigned char checksum = 0;
        for (int i = 0; i < MESSAGE_LENGTH - 1; i++)
          checksum += message[i];
        if (checksum != message[MESSAGE_LENGTH - 1])
        {
          // Invalid message, simply discard it.
        }
        else
        {
          // Decode message from host.
          // Message is send by adding an offset of 16384 to it.
          targetVelocity[0] = (unsigned int)(message[0] + (message[1] << 8)) - 16384;
          targetVelocity[1] = (unsigned int)(message[2] + (message[3] << 8)) - 16384;
          targetVelocity[2] = (unsigned int)(message[4] + (message[5] << 8)) - 16384;
        }
      }
    }
  }
  
  delay(2);
  
}
