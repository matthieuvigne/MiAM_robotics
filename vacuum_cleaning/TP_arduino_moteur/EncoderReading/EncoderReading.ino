/*
 * The goal of this code is to read data from a quadrature encoder.
 * 
 * Just like the signal generation example, two ways are shown:
 * 
 *  - the bad way: scanning the GIPO for a logic change. This takes a lot of processor time, and is very likely to miss transitions when the code is busy doing
 *  something else (e.g. sending data to a PC).
 *  - the good way: using interrupt to count in background, guaranteeing a looseless signal aquisition (granted that the interrupt function is well coded 
 *  and the number of interruptions reasonable).
 *  
 *  All ports on the Arduino can generate interrupts, but not all independantly. Here, we will use the interrupt on port P, which groups pins 0-7.
 *  Basically an interruption on this port is triggered when any interruption condition is met - but there is no way to know which condition it is
 *  (i.e. which pin triggered the interrupt). More importantly, you can't have the same interruption raised twice: you have to wait for the end
 *  of the interrupt function before being able to raise a new one. So having two independant interrupts on the same port is dangerous, because
 *  if both trigger at the same time things will go wrong.
 *  
 *  For details on the interruption capacity of the ATMega328P, see the datasheet, Ch. 11
 *  
 *  A quadrature encoder consists of two channels tha trigger in quadrature. This means we need two interrupts, but the interruption cannot happen
 *  at the same time. So it's fine to have them both on the same port, granted that the interruption is shorter than the time it takes the disk to rotate
 *  up to the next tick.
 *  
 */

// Here we define the pin numbers in the PIND bank - they happen to match arduino pinning on port D.
#define PINA 6
#define PINB 7

// To count, we need to store the previous value of port B - and obviously the counter.
bool oldBInterrupt = false;
int encoderCountInterrupt = 0;

bool oldBSoftware = false;
int encoderCountSoftware = 0;

// Software version also needs to keep old A to know when a pin has changed value.
bool oldASoftware = false;

// Port D corresponds to interrupt PCINT2 - so the name of the callback in Arduino is PCINT2_vect
// Thus, what this means is than when interruption PCINT2 is raised, this function will be called.
ISR(PCINT2_vect)
{
  // First, we read the status of channel A.
  // We don't use digitalRead, which is way too long, but instead directly read the value of the register.
  bool currentA = PIND & (1 << PINA);
  // The direction of the encoder is given by comparing the current A channel and the old value of the B channel.
  encoderCountInterrupt += (oldBInterrupt ^ currentA ? 1 : -1);
  // Finally, we update the old value of the B channel.
  oldBInterrupt =  PIND & (1 << PINB);
}


 
void setup() 
{
  // Setup serial communication with the PC.
  Serial.begin(115200);
  // Have pin 6 and 7 as input.
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  // Init variables.
  oldASoftware = digitalRead(PINA);
  oldBSoftware = digitalRead(PINB);
  oldBInterrupt = oldBSoftware;

  // Setup interrupt for port D, see p. 57 of the datashee.
  // PCMSK2 is a mask register: a 1 on bit i means pin i generates an interrupt, a 0 means no interrupt.
  // Here we set PINA and PINB as the only interrupt generators.
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << PINA;
  PCMSK2 |= 1 << PINB;
  // Finally, we enable interrupt generation for port D.
  PCICR = 0b100;
}

// Last time the serial value was sent, to have a constant update rate.
unsigned long lastTimeSerialSent;

void loop() 
{
  // Let's read both pins
  bool currentA = digitalRead(PINA);
  bool currentB = digitalRead(PINB);

  // If any of the two pins changed value, this means we need to increment the counter.
  if (currentA != oldASoftware || currentB != oldBSoftware)
  {
    // Same formula for encoder update
    encoderCountSoftware += (oldBSoftware ^ currentA ? 1 : -1);
    oldASoftware = currentA;
    oldBSoftware = currentB;
  }
  // Send the message to the PC: it is during this period that we are likely to miss an encoder tick...
  // To reduce the risk - and becase we don't need a very fast update - we do this only at 200Hz, the frequency we use on Gertrude.
  if (millis() - lastTimeSerialSent > 5)
  {
    lastTimeSerialSent = millis();
    Serial.print(encoderCountInterrupt);
    Serial.print(" ");
    Serial.println(encoderCountSoftware);
  }
}



