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


// Interrupt-based encoder counter, see EncoderReading for more details.

// Motor 0
// Pin 9 OCR1A
#define PWM_MOTOR_0 OCR1A
// Encoder 0
// Pin 14 (A4) -> A, pin 15 (A5) -> B
// (PC0 PC1)
#define PINA_MOTOR_0 0
#define PINB_MOTOR_0 1
#define DIRECTION_MOTOR_0 16

// Motor 1
// Pin 10 OCR1B
#define PWM_MOTOR_1 OCR1B
// Pin 
// Encoder 1
// Pin 5 -> A pin 6 -> B direction -> 7
// (PD6 PD7)
#define PINA_MOTOR_1 5
#define PINB_MOTOR_1 6
#define DIRECTION_MOTOR_1 7

// Motor 2
// Pin 11 OCR2A
#define PWM_MOTOR_2 OCR2A
// Encoder 2
// Pin 12 -> A pin 13 -> B
// (PB4 PB5)
#define PINA_MOTOR_2 4
#define PINB_MOTOR_2 5
#define DIRECTION_MOTOR_2 8

/// Send power to the motor, normalized between -1 and 1 (-1: full reverse, 0: stop, 1: full forward).
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
    PWM_MOTOR_2 = floor(abs(motorPower) * 255);
  }
}


// This variable stores the current value of the encoder.
// Note: as we count up, the int may overflow! With the current encoder, this happens after 91 turns.
int encoderCount_0 = 0;
bool oldB_0 = false;
ISR(PCINT1_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PINC & (1 << PINA_MOTOR_0);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount_0 += (oldB_0 ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB_0 =  PINC & (1 << PINB_MOTOR_0);
  
  //Serial.println("Interrupt 0");
}

int encoderCount_1 = 0;
bool oldB_1 = false;
ISR(PCINT2_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PIND & (1 << PINA_MOTOR_1);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount_1 += (oldB_1 ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB_1 =  PIND & (1 << PINB_MOTOR_1);

  //Serial.println("Interrupt 1");
}

int encoderCount_2 = 0;
bool oldB_2 = false;
ISR(PCINT0_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PINB & (1 << PINA_MOTOR_2);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCount_2 += (oldB_2 ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldB_2 =  PINB & (1 << PINB_MOTOR_2);
  
  //Serial.println("Interrupt 2");
}

unsigned long last_time = 0;
int last_encoder_count_0 = 0;
int last_encoder_count_1 = 0;
int last_encoder_count_2 = 0;

void setup() 
{
  // Setup serial communication with the PC.
  Serial.begin(115200);

  // Setup direction
  pinMode(DIRECTION_MOTOR_0, OUTPUT);
  pinMode(DIRECTION_MOTOR_1, OUTPUT);
  pinMode(DIRECTION_MOTOR_2, OUTPUT);
  
  // Setup encoder 0
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  PCMSK1 = 0x00;
  PCMSK1 |= 1 << PINA_MOTOR_0;
  PCMSK1 |= 1 << PINB_MOTOR_0;
  digitalWrite(14,HIGH);
  digitalWrite(15,HIGH);
 
  // Setup encoder 1
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << PINA_MOTOR_1;
  PCMSK2 |= 1 << PINB_MOTOR_1;
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);

   // Setup encoder 2
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  PCMSK0 = 0x00;
  PCMSK0 |= 1 << PINA_MOTOR_2;
  PCMSK0 |= 1 << PINB_MOTOR_2;
  digitalWrite(12,HIGH);
  digitalWrite(13,HIGH);

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

  last_time = micros();
  last_encoder_count_0 = 0;
  last_encoder_count_1 = 0;
  last_encoder_count_2 = 0;
}


float reference_velocity = 0;
float Kp = 1.0 / 90000.0;
float Ki = 1.0 / 5000.0;
float integral_error_0 = 0;
float integral_error_1 = 0;
float integral_error_2 = 0;

float dt = 100;

// User input parsing
String userInput = "";

boolean lastWasFF=false;
unsigned const int MESSAGE_LENGTH=6;
unsigned char message[MESSAGE_LENGTH];
int positionInMessage;
int TargetVelocity[3];


//
//void loop() 
//{
//  Serial.print(encoderCount_0);
//  Serial.print(" ");
//  Serial.print(encoderCount_1);
//  Serial.print(" ");
//  Serial.print(encoderCount_2);
//  Serial.println();
//  
//  delay(dt);
//}


void loop() 
{
  unsigned long current_time = micros();

  float time_since_last = (float)(current_time - last_time) / 1e6;

  float velocity_0 = (encoderCount_0 - last_encoder_count_0) / time_since_last;
  float velocity_1 = (encoderCount_1 - last_encoder_count_1) / time_since_last;
  float velocity_2 = (encoderCount_2 - last_encoder_count_2) / time_since_last;

  // Block for motor 0
  {
     
    last_time = current_time;
    last_encoder_count_0 = encoderCount_0;
  
    float error = (velocity_0 - TargetVelocity[0]);
    integral_error_0 += error * time_since_last;
    
    float target = - Kp * error - Ki * integral_error_0;
    
    // TODO: do motor velocity servoing...
    moveMotor(0, target);
    //moveMotor(0.5);
  }

  // Block for motor 1
  {
    last_time = current_time;
    last_encoder_count_1 = encoderCount_1;
  
    float error = (velocity_1 - TargetVelocity[1]);
    integral_error_1 += error * time_since_last;
    
    float target = - Kp * error - Ki * integral_error_1;
    
    // TODO: do motor velocity servoing...
    moveMotor(1, target);
    //moveMotor(0.5);
  }

  // Block for motor 2
  {
    last_time = current_time;
    last_encoder_count_2 = encoderCount_2;
  
    float error = (velocity_2 - TargetVelocity[2]);
    integral_error_2 += error * time_since_last;
    
    float target = - Kp * error - Ki * integral_error_2;
    
    // TODO: do motor velocity servoing...
    moveMotor(2, target);
    //moveMotor(0.5);
  }

  // Send the current encoder position to the Arduino.
  // TODO: replace this by velocity output instead of position output.
  // Serial.println(reference_velocity + " " + velocity_0 + " " + velocity_1);
//  Serial.println("Velocity:\n");
//  Serial.println(reference_velocity);
//  Serial.println(Mot1_velocity_velocity);
//  Serial.println(Mot2);
//  Serial.println(Mot3);

// If it's a 0xFF, and if the previous byte was also a 0xFF, a new message starts.


  // Read serial port to see if we have new user input.

while (Serial.available())
{
//     Serial.println(positionInMessage);
     unsigned char lastData = Serial.read();
//     Serial.println(int(lastData));
if(lastData == 0xFF && lastWasFF)
    positionInMessage = 0;
else
{
    lastWasFF = lastData == 0xFF;

    // If we are currently reading a message, add it to the buffer.
    if(positionInMessage > -1)
    {
        message[positionInMessage] = lastData;
        positionInMessage ++;
    }
    // If the end of a message was reached, decode it.
    if(positionInMessage == MESSAGE_LENGTH)
    {
        // Reset status.
        lastWasFF = false;
        positionInMessage = -1;

//        // Verify checksum.
//        // Sum of previous bytes must be equal to the checksum.
//        uint8_t checksum = 0;
//        for(int i = 0; i < MESSAGE_LENGTH - 1; i++)
//            checksum += buffer[i];
//        if(checksum != buffer[MESSAGE_LENGTH - 1])
//        {
//            #ifdef DEBUG
//                printf("[uCListener] Invalid checksum, refusing packet\n");
//            #endif
//        }
       TargetVelocity[0]=message[0]+ (message[1]<<8) - 16384;
       TargetVelocity[1]=message[2]+ (message[3]<<8) - 16384;
       TargetVelocity[2]=message[4]+ (message[5]<<8) - 16384;
       

    }
}

}
//  Serial.println("Velocity:\n");
//  Serial.println(reference_velocity);
  Serial.println(TargetVelocity[0]);
  Serial.println(TargetVelocity[1]);
  Serial.println(TargetVelocity[2]);
  
  delay(dt);
}
