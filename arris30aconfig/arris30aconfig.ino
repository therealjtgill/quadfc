 /*
 * This connects all motor pulse outputs to the throttle input of the receiver.
 * This is for setting the minimum and maximum thresholds on the motor pulses.
 * 
 * Load the program onto the Arduino
 * Turn off the Arduino
 * Turn on the remote control
 * Set the throttle (channel 3) to its max value
 * Connect Arduino to battery power (not USB)
 * ESC's should beep
 * Move throttle down to lowest level
 * ESC's should beep again, indicating that max and min values have been set
 * Move throttle up/down and verify that all motor speeds increase accordingly
 */
#define CYCLELEN 4000 // Cycle length in microseconds
#define NUMRECEIVERCHANNELS 4

#define PRINTRXPULSES 0
#define PRINTMOTORPULSES 0
#define PRINTMOTORTIMERS 0
#define PRINTCYCLELENGTH 0

uint16_t current_time = 0;
uint16_t rx_timers[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};
int rx_channels_last[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};
int rx_pulses[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};

/////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////
void setup() {
  // Enable pin change interrupt by writing to this register on the Arduino.
  // Register bit 0: enable pin control interrupts on pins 0:7
  // Register bit 1: enable pin control interrupts on pins 8:14
  // Register bit 2: enable pin control interrupts on pins 16:23
  PCICR |= B00000001;

  // Only allow interrupts on particular pins within the pin 0:7 group
  // by masking some of the pins.
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  // Set data direction register for port D so that physical pins
  // 2, 3, 4, 5 can be used as output for the motors.
  DDRD |= B00111100;

  // Arm the speed controllers
  for (unsigned int i = 0; i < 5; ++i)
  {
    PORTD |= B00111100;
    delayMicroseconds(1000);
    PORTD &= B00111100;
    delayMicroseconds(5500);
  }

  Serial.begin(9600);

  for (unsigned int i = 0; i < 4; ++i)
  {
    rx_pulses[i] = 1000;
    rx_timers[i] = 1000;
  }
}

/////////////////////////////////////////////////
// Main Loop
/////////////////////////////////////////////////
void loop() {
  static double cycleStartTime = 0.;
  static double motorStartTime = 0.;
  static bool initialized = false;

  static double t = 0.;
  static double dt = 0.;

  static unsigned long motor_timers[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};
  static uint16_t motor_pulses[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};

  static unsigned long loopTime = 0;

  if (!initialized)
  {
    loopTime = 0;
    initialized = true;
    t = micros();
  }
  
  if (PRINTCYCLELENGTH)
  {
    Serial.print(t - cycleStartTime); Serial.print(" ");
  }

  dt = (t - cycleStartTime)/1e6;
  cycleStartTime = micros();
  
  if (PRINTRXPULSES)
  {
    Serial.print(rx_pulses[0]); Serial.print(" ");
    Serial.print(rx_pulses[1]); Serial.print(" ");
    Serial.print(rx_pulses[2]); Serial.print(" ");
    Serial.print(rx_pulses[3]); Serial.print(" ");
  }

  // Throttle = rx_pulses[2]
/*
  motor_pulses[0] = rx_pulses[2];
  motor_pulses[1] = rx_pulses[2];
  motor_pulses[2] = rx_pulses[2];
  motor_pulses[3] = rx_pulses[2];
*/
  if (rx_pulses[2] >= 1700)
  {
    motor_pulses[0] = 2000;
    motor_pulses[1] = 2000;
    motor_pulses[2] = 2000;
    motor_pulses[3] = 2000;
  }
  else if (rx_pulses[2] <= 1200)
  {
    motor_pulses[0] = 1000;
    motor_pulses[1] = 1000;
    motor_pulses[2] = 1000;
    motor_pulses[3] = 1000;
  }

  if (PRINTMOTORPULSES)
  {
    Serial.print(motor_pulses[0]); Serial.print(" ");
    Serial.print(motor_pulses[1]); Serial.print(" ");
    Serial.print(motor_pulses[2]); Serial.print(" ");
    Serial.print(motor_pulses[3]); Serial.print(" ");
  }

  // Turn all of motor pulses on.
  PORTD |= B00111100;

  motorStartTime = micros();

  motor_timers[0] = motor_pulses[0] + motorStartTime;
  motor_timers[1] = motor_pulses[1] + motorStartTime;
  motor_timers[2] = motor_pulses[2] + motorStartTime;
  motor_timers[3] = motor_pulses[3] + motorStartTime;

  // PORTD will become B00000000 within 2000us.
  while (PORTD >= 4)
  {
    loopTime = micros();
    if (motor_timers[0] <= loopTime)
    {
      PORTD &= B11111011;
    }
    if (motor_timers[1] <= loopTime)
    {
      PORTD &= B11110111;
    }
    if (motor_timers[2] <= loopTime)
    {
      PORTD &= B11101111;
    }
    if (motor_timers[3] <= loopTime)
    {
      PORTD &= B11011111;
    }
  }

  if (PRINTMOTORTIMERS)
  {
    Serial.print(motor_timers[0]); Serial.print(" ");
    Serial.print(motor_timers[1]); Serial.print(" ");
    Serial.print(motor_timers[2]); Serial.print(" ");
    Serial.print(motor_timers[3]); Serial.print(" ");
  }

  // Pause for the rest of the 4000us loop.
  while (cycleStartTime + CYCLELEN > micros());

  if (PRINTRXPULSES || PRINTMOTORPULSES || PRINTCYCLELENGTH || PRINTMOTORTIMERS)
  {
    Serial.println("");
  }
  t = micros();
}

/////////////////////////////////////////////////
// ISR
/////////////////////////////////////////////////
ISR(PCINT0_vect)
{
  //Loop over all channels, see what states the pins are in, and
  //see how long the pulse widths are if the state has transitioned
  //from 1 to 0.
  //PINB is a register containing the current states of physical
  //pins 8 - 13
  current_time = micros();
  
  if (rx_channels_last[0] == 0 && (PINB & B00000001))
  {
    rx_channels_last[0] = 1;
    rx_timers[0] = current_time;
  }
  else if (rx_channels_last[0] == 1 && !(PINB & B00000001))
  {
    rx_channels_last[0] = 0;
    rx_pulses[0] = current_time - rx_timers[0];
  }
  
  if (rx_channels_last[1] == 0 && (PINB & B00000010))
  {
    rx_channels_last[1] = 1;
    rx_timers[1] = current_time;
  }
  else if (rx_channels_last[1] == 1 && !(PINB & B00000010))
  {
    rx_channels_last[1] = 0;
    rx_pulses[1] = current_time - rx_timers[1];
  }

  if (rx_channels_last[2] == 0 && (PINB & B00000100))
  {
    rx_channels_last[2] = 1;
    rx_timers[2] = current_time;
  }
  else if (rx_channels_last[2] == 1 && !(PINB & B00000100))
  {
    rx_channels_last[2] = 0;
    rx_pulses[2] = current_time - rx_timers[2];
  }

  if (rx_channels_last[3] == 0 && (PINB & B00001000))
  {
    rx_channels_last[3] = 1;
    rx_timers[3] = current_time;
  }
  else if (rx_channels_last[3] == 1 && !(PINB & B00001000))
  {
    rx_channels_last[3] = 0;
    rx_pulses[3] = current_time - rx_timers[3];
  }
}
