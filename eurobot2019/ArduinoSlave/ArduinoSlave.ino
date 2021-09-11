
// Code for the slave Arduino uC, used in the robot to access low-level sensor.
// This code reads the following sensors, sending the result over uart:
//  - wheel rotary encoder.

// Read a single bit off a register: return 0 if low, >0 if high.
#define READ_BIT(REG, BIT) (REG & (1 << BIT))

#define ENCODER_TO_REGISTER(ENCODER) (ENCODER ? PINB : PIND)

// Encoder pin number: first encoder is in port D, second encoder is in port B.
const char encoderPinA[2] = {7,0};
const char encoderPinB[2] = {6,1};

// Encoder old B value.
bool oldB[2] = {0, 0};

// Current encoder position.
int encoderCount[2] = {0, 0};

// Max and min values of an int_15 (2^14 - 1 and -2^14), for cropping int to 15 bits.
#define INT15_MIN -16384
#define INT15_MAX 16383

// Interrupt function, called when an encoder interrupt is triggered.
void handleEncoder(char encoderNumber)
{
  // Get current status.
  bool currentA = READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinA[encoderNumber]);

  // The direction of the encoder is given by currentA xor oldB
  encoderCount[encoderNumber] += (oldB[encoderNumber] ^ currentA ? 1 : -1);
  oldB[encoderNumber] =  READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinB[encoderNumber]);

  // Keep the number within 15 bits.
  if(encoderCount[encoderNumber] > INT15_MAX)
    encoderCount[encoderNumber] = INT15_MIN ;
  else if(encoderCount[encoderNumber] < INT15_MIN)
    encoderCount[encoderNumber] = INT15_MAX;

}

// Interrupt for first encoder.
ISR(PCINT2_vect)
{
  handleEncoder(0);
}

// Interrupt for second encoder.
ISR(PCINT0_vect)
{
  handleEncoder(1);
}


void setup()
{
  // Enable serial port.
  Serial.begin(1000000);
  // Set encoder pins as input.
  DDRD &= ~(1 << encoderPinA[0]);
  DDRD &= ~(1 << encoderPinB[0]);
  DDRB &= ~(1 << encoderPinA[1]);
  DDRB &= ~(1 << encoderPinB[1]);
  // Setup interrupt for port B and D.
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << encoderPinA[0];
  PCMSK2 |= 1 << encoderPinB[0];
  PCMSK0 = 0x00;
  PCMSK0 |= 1 << encoderPinA[1];
  PCMSK0 |= 1 << encoderPinB[1];
  // Enalbe interrupt for port B and D.
  PCICR = 0b101;

  // Send ID message - since whenever doing an open() in Linux, the Arduino gets reset, this is the first message the master will read.
  Serial.write("MiAMSlave");
}

int potentiometerPosition = 0;

void loop() {
 // Read potentiometer position.
 potentiometerPosition = analogRead(A0);
 Serial.write(0xFF);
 Serial.write(0xFF);
 // Compute checksum
 uint8_t checksum = 0;
 for(int i = 0; i < 2; i++)
 {
  // Take encoderCount, use its 2s complement with 2^15
  int encoderComp = (1 << 15) - encoderCount[i];
  Serial.write((encoderComp >> 8) & 0xFF);
  Serial.write(encoderComp & 0xFF);
  checksum += (encoderComp >> 8) & 0xFF;
  checksum += encoderComp & 0xFF;
 }
 Serial.write((potentiometerPosition >> 8) & 0xFF);
 Serial.write(potentiometerPosition & 0xFF);
 checksum += (potentiometerPosition >> 8) & 0xFF;
 checksum += potentiometerPosition & 0xFF;
 Serial.write(checksum);
 delayMicroseconds(2000);
}
