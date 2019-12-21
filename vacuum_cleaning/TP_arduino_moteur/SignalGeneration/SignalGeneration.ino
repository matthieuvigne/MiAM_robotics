/*
 * The goal of this code is to generate a PWM signal at 1kHz, with 10% duty cycle.
 * This means that the expected output signal is a high (1, i,e. 5V) for 100us and a low for 900us.
 * 
 * This is done in two different ways, to show the good way and the bad way to do it:
 * 
 *  - the bad way is to toogle the pin manually, using delays between each code. This can be applied to any random pin of the Arduino.
 *  While it's very easy to code, this gives very bad results, inaccurate but most importantly with a lot of jitter due to computing load.
 *  - the good way is to use the internal timers and signal generators of the microcontroller. Here I'm talking about the hardware
 *  
 *  timer, i.e. not the program clock. Some pins of the Arduino can thus be directly wired to the oscillator to generate an output signal,
 *  that can be customized using registers. The main point to understand is the following: there is NO software involved in generating this
 *  signal. Software is only used for configuration: after that this takes EXACTLY 0 PROCESSING LOAD. It's a hardware process (kind of like
 *  what we did on a FPGA) that does the trick. This is why we can do that on every pin: only those equipped with such a time can be used.
 *  
 *  Implementing the 'good' way, i.e. hardware PWM, is much harder for a beginner that the bad way. Yet as you will see, the results are
 *  uncomparable: thus, I strongly advise you not to fall for the apparent easy of sloppy implementation, as you might get a strong backlash...
 *  
 *  Since hardware PWM, by definition, depends on the hardware, the details are specific to each processor. Thus, to implement it, we need
 *  to read the manual of the Arduino Uno processor, the ATMega328P (https://www.sparkfun.com/datasheets/Components/SMD/ATMega328.pdf). 
 *  The code below is heavily commented, for more information refer to this manual, Chapter 13.
 *  
 *  The Arduino is equipped with 3 timers, TIMER0, TIMER1 and TIMER2. TIMER0 is used for the program clock, thus playing with it
 *  will have effect on function like delay() in the Arduino IDE (which basically relies on specific settings for this clock).
 *  TIMER0 and TIMER2 are both 8-bit counter: here we will use TIMER1, which is equipped with a 16-bit counter.
 *  
 */
 
void setup() 
{  
  // For software PWM, we will be using pin 8 in Arduino numbering (PB0) : this is the only configuration required.
  pinMode(8, OUTPUT);

  // For hardware PWM, we now need to do the full setup.

  // I will here rephrase Chapter 13 describing the working of TIMER1 and aossicated functions, talking only about the parts related to our use case.
  // All TIMERS can be used for several functions, here, we are interested in the waveform generator function. Two pins are associated outputs
  // for TIMER1: they are called OC1A and OC1B (likewise OC0A and OC0B are associated to TIMER0...). These pins can be configured to act as
  // signal generators based on this timer (otherwise they are normal GPIOs). Taking a look at the pinout on page 2, we see that
  // OC1A is located on pin PB1 - this corresponds to pin9 on the Arduino. So first, we will set this pin as output.
  pinMode(9, OUTPUT);

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
  TCCR1A = 0b10000010;
  TCCR1B = 0b00011001;

  // Now we need to set ICR1A for a 1kHz signal.
  // The frequency is given by CLK / ICR1A, so we have ICR1A = CLK/1kHz = 16000
  ICR1 = 15999;

  // Duty cycle of 10%: OCR1A is 10% of ICR1A
  OCR1A = 1600;

  // And that's it: now the signal is being generated, and there is no more code running !

  // Additonnaly, imagine you now want a signal at 10kHz, you just need to do:
  // ICR1A = 1600
  // OCR1A = 160
}


void loop() 
{
  // In the loop, we just do the naive implementation.
  digitalWrite(8, HIGH);
  delayMicroseconds(100);
  digitalWrite(8, LOW);
  delayMicroseconds(900);
  // So we generate our signal, but this takes our processor: what do we do if we actually want to run code ???
  // We could imagine something like:
  // 
  // digitalWrite(8, HIGH);
  // time = micros()
  // do_something_that_takes_less_than_100us()
  // delayMicroseconds(100 - (micros() - time));
  // 
  // This is already better, but it still doesn't scale to higher frequency / more extensive processing requirements (i.e. we have to make sure our function runs in less than the loop time...)
}

// As a quick reference, this is a nice summary also covering the other timers: https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328



