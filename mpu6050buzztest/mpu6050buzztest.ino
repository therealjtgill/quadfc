#include <pid.hpp>
#include <propagateimu.hpp>
#include <remotestartup.hpp>
#include <Wire.h>
#include <EEPROM.h>
//#define CALCULATEANDSAVEGYROBIASES // Enable this to calculate gyro biases and save them to EEPROM.
#define CYCLELEN 4000 // Cycle length in microseconds
#define NUMRECEIVERCHANNELS 4

#define PRINTRXPULSES 0
#define PRINTSETPOINTS 0
#define PRINTIMUDEGINPUT 1
#define PRINTIMUDEGPSINPUT 0
#define PRINTIMUACCOUTPUT 0
#define PRINTIMUGYROUPDATE 0
#define PRINTPIDOUTPUT 0
#define PRINTPIDCONTROL 0
#define PRINTMOTORPULSES 0
#define PRINTMOTORTIMERS 0
#define PRINTCYCLELENGTH 0

int current_time = 0;
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

  Wire.begin();
  TWBR = 12;

  //Serial.println("This text should display");
  delay(800);

  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set accelerometer up for FS_SEL=2
  // Acceleration range of +/-8 g
  Wire.beginTransmission(0x68);
  Wire.write(0x1c);
  Wire.write(0x10);
  Wire.endTransmission(true);

  // Set gyroscope up for FS_SEL=1
  // Rate range of +/-500 degrees/second
  Wire.beginTransmission(0x68);
  Wire.write(0x1b);
  Wire.write(0x08);
  Wire.endTransmission(true);

  Serial.begin(9600);

  // Arm the speed controllers
  for (unsigned int i = 0; i < 5; ++i)
  {
    PORTD |= B00111100;
    delayMicroseconds(1000);
    PORTD &= B00111100;
    delayMicroseconds(5500);
  }

  for (unsigned int i = 0; i < 1000; ++i)
  {
    PORTD != B00111100;
    delayMicroseconds(1700);
    PORTD &= B00111100;
    delayMicroseconds(300);
  }

  for (unsigned int i = 0; i < 4; ++i)
  {
    rx_pulses[i] = 1000;
    rx_timers[i] = 1000;
  }
}

/////////////////////////////////////////////////
// interpolateLinear
/////////////////////////////////////////////////
float interpolateLinear(
  float input_min,
  float input_max,
  float output_min,
  float output_max,
  float input,
  bool clamp=true
)
{
  float m = ((output_max - output_min)/(input_max - input_min));
  if (clamp)
  {
    return max(
      min(
        m*input + output_min - m*input_min,
        output_max
      ),
      output_min
    );
  }
  return m*input + output_min - m*input_min;
}

/////////////////////////////////////////////////
// getImuData
/////////////////////////////////////////////////
void getImuData(float * acc_meas_out, float * gyro_meas_out)
{
  static unsigned int i = 0;
  i = 0;

  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(true);
  Wire.requestFrom(0x68, 14);

  while(Wire.available() < 14);

  for (i = 0; i < 3; ++i)
  {
    acc_meas_out[i] = (-1.0)*((float)((Wire.read() << 8) | Wire.read()))/4096.;
  }

  for (i = 0; i < 2; ++i)
  {
    Wire.read();
  }

  for (i = 0; i < 3; ++i)
  {
    gyro_meas_out[i] = (float)((Wire.read() << 8) | Wire.read())/65.5;
    if (i == 1)
    {
      gyro_meas_out[i] *= -1.0;
    }
  }
}

/////////////////////////////////////////////////
// calculateGyroBiases
/////////////////////////////////////////////////
void calculateGyroBiases(
  float * phi_rate_bias_out,
  float * theta_rate_bias_out,
  float * psi_rate_bias_out
)
{
#ifdef CALCULATEANDSAVEGYROBIASES
  
  float acc[3]  = {0., 0., 0.};
  float gyro[3] = {0., 0., 0.};

  *phi_rate_bias_out = 0.;
  *theta_rate_bias_out = 0.;
  *psi_rate_bias_out = 0.;
  unsigned int num_trials = 10000;
  for (unsigned int i = 0; i < num_trials; ++i)
  {
    getImuData(acc, gyro);
    *phi_rate_bias_out += gyro[0];
    *theta_rate_bias_out += gyro[1];
    *psi_rate_bias_out += gyro[2];
  }

  *phi_rate_bias_out /= static_cast<float>(num_trials);
  *theta_rate_bias_out /= static_cast<float>(num_trials);
  *psi_rate_bias_out /= static_cast<float>(num_trials);
  Serial.println("Saved to EEPROM.");
  int ee_address = 0;
  EEPROM.put(ee_address, *phi_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.put(ee_address, *theta_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.put(ee_address, *psi_rate_bias_out);
#else
  // These values are saved to EEPROM after many trials.
  Serial.println("Read from EEPROM:");
  int ee_address = 0;
  EEPROM.get(ee_address, *phi_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *theta_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *psi_rate_bias_out);
#endif
}

/////////////////////////////////////////////////
// Main Loop
/////////////////////////////////////////////////
void loop() {
  static double cycleStartTime = 0.;
  static double motorStartTime = 0.;
  static bool initialized = false;

  static float phi_meas = 0.;
  static float theta_meas = 0.;
  static float phi_rate_meas = 0.;
  static float theta_rate_meas = 0.;
  static float psi_rate_meas = 0.;

  static int16_t phi_ctl = 0.;
  static int16_t theta_ctl = 0.;
  static int16_t psi_rate_ctl = 0.;

  static float phi_set = 0.;
  static float theta_set = 0.;
  static float psi_rate_set = 0.;

  static float phi_rate_gyr_bias = 0.;
  static float theta_rate_gyr_bias = 0.;
  static float psi_rate_gyr_bias = 0.;

  static double t = 0.;
  static double dt = 0.;

  static int16_t throttle = 0;
  static unsigned long motor_timers[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};
  static uint16_t motor_pulses[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};

  static float acc_meas[3] = {0., 0., 0.};
  static float gyro_meas_degps[3] = {0., 0., 0.};

  static TuningMode currentTuneMode = 4;
  static TuningMode newTuneMode = 4;
  static double timeSinceLastModeChange = 0.;
  static double timeOfLastModeChange = 0.;
  static uint16_t motor_pulses_mask[NUMRECEIVERCHANNELS] = {0., 0., 0., 0.};

  static unsigned long loopTime = 0;

  if (!initialized)
  {
    calculateGyroBiases(&phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias);
    loopTime = 0;
    initialized = true;
    t = micros();
  }

  timeSinceLastModeChange = millis() - timeOfLastModeChange;
  newTuneMode = checkTuningMode(
    rx_pulses,
    currentTuneMode,
    timeSinceLastModeChange
  );

  if (newTuneMode != currentTuneMode)
  {
    timeOfLastModeChange = millis();
    switch(newTuneMode)
    {
      case MOTOR1:
        Serial.println("MOTOR1 Balancing");
        break;
      case MOTOR2:
        Serial.println("MOTOR2 Balancing");
        break;
      case MOTOR3:
        Serial.println("MOTOR3 Balancing");
        break;
      case MOTOR4:
        Serial.println("MOTOR4 Balancing");
        break;
      case TUNINGOFF:
        Serial.println("Tuning disabled");
        break;
      case ALLMOTORS:
        Serial.println("All motors");
        break;
      default:
        Serial.println("Unknown mode");
        break;
    }
    for (unsigned int i = 0; i < NUMRECEIVERCHANNELS; ++i)
    {
      motor_pulses_mask[i] = 0;
    }
    if (newTuneMode < 4)
    {
      motor_pulses_mask[newTuneMode] = 1;
    }
    if (newTuneMode == ALLMOTORS)
    {
      for (unsigned int i = 0; i < NUMRECEIVERCHANNELS; ++i)
      {
        motor_pulses_mask[i] = 1;
      }
    }
    currentTuneMode = newTuneMode;
  }

  if (PRINTCYCLELENGTH)
  {
    Serial.print(t - cycleStartTime); Serial.print(" ");
  }

  dt = (t - cycleStartTime)/1e6;
  cycleStartTime = micros();
//  updateAngleCalculations(
//    acc_meas, gyro_meas_degps,
//    &phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias, dt,
//    &phi_meas, &theta_meas, &psi_rate_meas
//  );
  updateAngleCalculations(
    acc_meas, gyro_meas_degps,
    &phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias, dt,
    &phi_meas, &theta_meas, &phi_rate_meas, &theta_rate_meas, &psi_rate_meas
  );

  if (PRINTIMUDEGINPUT)
  {
    Serial.print(phi_meas);      Serial.print(" ");
    Serial.print(theta_meas);    Serial.print(" ");
    Serial.print(psi_rate_meas); Serial.print(" ");
  }

  if (PRINTIMUDEGPSINPUT)
  {
    Serial.print(phi_rate_meas); Serial.print(" ");
    Serial.print(theta_rate_meas); Serial.print(" ");
    Serial.print(psi_rate_meas); Serial.print(" ");
  }

  if (PRINTRXPULSES)
  {
    Serial.print(rx_pulses[0]); Serial.print(" ");
    Serial.print(rx_pulses[1]); Serial.print(" ");
    Serial.print(rx_pulses[2]); Serial.print(" ");
    Serial.print(rx_pulses[3]); Serial.print(" ");
  }

  // Throttle = rx_pulses[2]
  motor_pulses[0] = rx_pulses[2]*motor_pulses_mask[0];
  motor_pulses[1] = rx_pulses[2]*motor_pulses_mask[1];
  motor_pulses[2] = rx_pulses[2]*motor_pulses_mask[2];
  motor_pulses[3] = rx_pulses[2]*motor_pulses_mask[3];
  
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

  // This call takes ~750us, motors must be turned on for at least 1000us, so
  // use 750us of that time to get gyro data instead of doing nothing.
  getImuData(acc_meas, gyro_meas_degps);

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

  if (
       PRINTRXPULSES
    || PRINTMOTORPULSES
    || PRINTIMUDEGINPUT
    || PRINTCYCLELENGTH
    || PRINTPIDOUTPUT
    || PRINTPIDCONTROL
    || PRINTSETPOINTS
    || PRINTMOTORTIMERS
    || PRINTIMUACCOUTPUT
    || PRINTIMUDEGPSINPUT
  )
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
