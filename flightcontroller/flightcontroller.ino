#include <pid.hpp>
#include <propagateimu.hpp>
#include <Wire.h>
#include <EEPROM.h>
#include <remotestartup.hpp>
//#define CALCULATEANDSAVEGYROBIASES // Enable this to calculate gyro biases and save them to EEPROM.
#define CYCLELEN 4000. // Cycle length in microseconds
#define NUMRECEIVERCHANNELS 4
#define MINTHROTTLE 1250

#define PRINTRXPULSES    0
#define PRINTSETPOINTS   0
#define PRINTIMUDEGINPUT 0
#define PRINTPIDCONTROL  0
#define PRINTMOTORPULSES 1
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
  TWBR = 12; // Some shit I saw on forum.arduino.cc/index.php?topic=58031.0

  Serial.println("This text should display");
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

  // Use built-in LED to signal safe/not-safe status.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
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
  static float m = 0.;
  m = ((output_max - output_min)/(input_max - input_min));
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
  Serial.println("Saved to EEPROM:");
  int ee_address = 0;
  EEPROM.put(ee_address, *phi_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.put(ee_address, *theta_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.put(ee_address, *psi_rate_bias_out);
  Serial.println(*phi_rate_bias_out);
  Serial.println(*theta_rate_bias_out);
  Serial.println(*psi_rate_bias_out);
#else

  //Serial.println("Read from EEPROM:");
  int ee_address = 0;
  EEPROM.get(ee_address, *phi_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *theta_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *psi_rate_bias_out);
#endif
}

/////////////////////////////////////////////////
// calculatePidControls
/////////////////////////////////////////////////
// x - measured value
// u - setpoint
// y - output
void calculatePidControls(
  const float & x_phi_deg,
  const float & x_theta_deg,
  const float & x_psi_degps,
  const float & u_phi_deg,
  const float & u_theta_deg,
  const float & u_psi_degps,
  int16_t & y_phi,
  int16_t & y_theta,
  int16_t & y_psi_rate
)
{
  static PID<float> phi_pid(1.2, 0.0, 15.0, 0., 0., -400., 400.);
  static PID<float> theta_pid(1.2, 0.0, 15.0, 0., 0., -400., 400.);
  static PID<float> psi_rate_pid(1.0/2., 0.00/8, 0.0/8.0, 0., 0., -400., 400.);
//
//  static float u_phi_deg = 0.;
//  static float u_theta_deg = 0.;
//  static float u_psi_degps = 0.;

  static float phi_filtered = 0.;
  static float theta_filtered = 0.;
  static float psi_rate_filtered = 0.;
//
//  u_phi_deg = (u_phi_rad)*(180./M_PI);
//  u_theta_deg = (u_theta_rad)*(180./M_PI);
//  u_psi_degps = (u_psi_radps)*(180./M_PI);

  phi_filtered = phi_pid.filter(x_phi_deg, u_phi_deg);
  theta_filtered = theta_pid.filter(x_theta_deg, u_theta_deg);
  psi_rate_filtered = psi_rate_pid.filter(x_psi_degps, u_psi_degps);

  y_phi = static_cast<int16_t>(phi_filtered);
  y_theta = static_cast<int16_t>(theta_filtered);
  y_psi_rate = static_cast<int16_t>(psi_rate_filtered);

}

/////////////////////////////////////////////////
// rxPulsesToSetPoints
/////////////////////////////////////////////////
void rxPulsesToSetPoints(
  float * u_phi_out,
  float * u_theta_out,
  float * u_psi_rate_out
)
{
  // After tinkering with the IMU, I decided that I don't want the drone to be able to
  // roll or pitch more than 15 degrees (from controller input).
  // rx_pulses is a global array, its size should always be 4.
  uint16_t limited_throttle = min(rx_pulses[2], 1750);
  if (limited_throttle >= MINTHROTTLE)
  {
    *u_phi_out      = -1.*interpolateLinear(1000, 2000, -15., 15., rx_pulses[1]);
    *u_theta_out    = -1.*interpolateLinear(1000, 2000, -15., 15., rx_pulses[0]);
    *u_psi_rate_out = interpolateLinear(1000, 2000, -60., 60., rx_pulses[3]);
    return;
  }
  else
  {
    *u_phi_out      = 0.;
    *u_theta_out    = 0.;
    *u_psi_rate_out = 0.;
    return;
  }
}

/////////////////////////////////////////////////
// calculateMotorMixtures
/////////////////////////////////////////////////
void calculateMotorMixtures(
  const int16_t & phi_ctl,
  const int16_t & theta_ctl,
  const int16_t & psi_rate_ctl,
  const int16_t & throttle,
  uint16_t * motor_timers_out
)
{
  static uint16_t tempMix = 0;

  // Limit the throttle output so that PID controller has room to make
  // corrections.
  uint16_t limited_throttle = min(throttle, 1750);
  //uint16_t limited_throttle = 1500;
  if (limited_throttle >= MINTHROTTLE)
  {
    tempMix = (limited_throttle + phi_ctl - theta_ctl - psi_rate_ctl);
    motor_timers_out[0] = clamp<uint16_t>(tempMix, 900, 1900);
    tempMix = (limited_throttle - phi_ctl - theta_ctl + psi_rate_ctl);
    motor_timers_out[1] = clamp<uint16_t>(tempMix, 900, 1900);
    tempMix = (limited_throttle - phi_ctl + theta_ctl - psi_rate_ctl);
    motor_timers_out[2] = clamp<uint16_t>(tempMix, 900, 1900);
    tempMix = (limited_throttle + phi_ctl + theta_ctl + psi_rate_ctl);
    motor_timers_out[3] = clamp<uint16_t>(tempMix, 900, 1900);
    return;
  }
  else
  {
    motor_timers_out[0] = 1000;
    motor_timers_out[1] = 1000;
    motor_timers_out[2] = 1000;
    motor_timers_out[3] = 1000;
    return;
  }
  
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

  //static double averageLoopTime = 0.;
  //static int loopCounter = 0;

  static int16_t throttle = 0;
  static unsigned long motor_timers[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};
  static uint16_t motor_pulses[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};

  static float acc_meas[3];
  static float gyro_meas_degps[3];

  static unsigned long loopTime = 0;
  static OperationMode flightMode = UNINITIALIZED;
  static OperationMode newFlightMode = UNINITIALIZED;
  static double modeTimeElapsed = 0.;
  static double modeStartTime = 0.;

  static bool firstFewLoops = true;
  static uint8_t tenLoopCount = 0;

  if (!initialized)
  {
    calculateGyroBiases(&phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias);
    loopTime = 0;
    initialized = true;
    t = micros();
  }

  if (PRINTCYCLELENGTH)
  {
    Serial.print(t - cycleStartTime); Serial.print(" ");
  }
/*
  averageLoopTime += t - cycleStartTime;
  ++loopCounter;
  if (loopCounter > 1000)
  {
    Serial.println(averageLoopTime/static_cast<double>(loopCounter));
    averageLoopTime = 0.;
    loopCounter = 0;
  }
*/
  dt = (t - cycleStartTime)/1e6;
  cycleStartTime = micros();

  modeTimeElapsed = millis() - modeStartTime;
  newFlightMode = checkOperationMode(rx_pulses, flightMode, modeTimeElapsed);
  if (newFlightMode != flightMode)
  {
    switch(newFlightMode)
    {
      case FLIGHT:
        digitalWrite(LED_BUILTIN, LOW);
        //Serial.println("FLIGHT");
        break;
      case NOFLIGHT:
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("NO FLIGHT, NEIN");
        break;
      case UNINITIALIZED:
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("UNINITIALIZED");
        break;
    }
    flightMode = newFlightMode;
    modeStartTime = millis();
  }

  updateAngleCalculations(
    acc_meas, gyro_meas_degps,
    &phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias, dt,
    &phi_meas, &theta_meas, &phi_rate_meas, &theta_rate_meas, &psi_rate_meas,
    firstFewLoops
  );

  if (PRINTIMUDEGINPUT)
  {
    Serial.print(phi_meas);      Serial.print(" ");
    Serial.print(theta_meas);    Serial.print(" ");
    Serial.print(psi_rate_meas); Serial.print(" ");
  }

  rxPulsesToSetPoints(
    &phi_set, &theta_set, &psi_rate_set
  );

  if (PRINTSETPOINTS)
  {
    Serial.print(phi_set); Serial.print(" ");
    Serial.print(theta_set); Serial.print(" ");
    Serial.print(psi_rate_set); Serial.print(" ");
  }

  // Don't want PID values to increase or decrease while we're not in flight mode.
  if (flightMode == FLIGHT)
  {
    // phi_set = 0.;
    // theta_set = 0.;
    // psi_rate_set = 0.;
    phi_set = 5*(phi_set - phi_meas);
    theta_set = 5*(theta_set - theta_meas);
//    Serial.print(phi_set); Serial.print(" ");
//    Serial.print(theta_set); Serial.print(" ");
//    Serial.println(psi_rate_set);
    calculatePidControls(
      phi_rate_meas, theta_rate_meas, psi_rate_meas,
      phi_set,  theta_set,  psi_rate_set,
      phi_ctl,  theta_ctl,  psi_rate_ctl
    );
  }

  if (PRINTPIDCONTROL)
  {
    Serial.print(phi_ctl); Serial.print(" ");
    Serial.print(theta_ctl); Serial.print(" ");
    Serial.print(psi_rate_ctl); Serial.print(" ");
  }

  // Throttle = rx_pulses[2]
  calculateMotorMixtures(
    phi_ctl, theta_ctl, psi_rate_ctl, rx_pulses[2], motor_pulses
  );

  // Pause for the rest of the 4000us loop.
  while (cycleStartTime + CYCLELEN - 50 >= micros());

  if (tenLoopCount < 10)
  {
    ++tenLoopCount;
  }
  else if (tenLoopCount >= 10 && firstFewLoops == true)
  {
    firstFewLoops = false;
  }

  if (
       PRINTRXPULSES
    || PRINTMOTORPULSES
    || PRINTIMUDEGINPUT
    || PRINTCYCLELENGTH
    || PRINTPIDCONTROL
    || PRINTSETPOINTS
  )
  {
    Serial.println("");
  }
  t = micros();

  if (flightMode != FLIGHT)
  {
    motor_pulses[0] = 0;
    motor_pulses[1] = 0;
    motor_pulses[2] = 0;
    motor_pulses[3] = 0;
  }

  if (PRINTMOTORPULSES)
  {
    Serial.print(motor_pulses[0]); Serial.print(" ");
    Serial.print(motor_pulses[1]); Serial.print(" ");
    Serial.print(motor_pulses[2]); Serial.print(" ");
    Serial.print(motor_pulses[3]); Serial.print(" ");
  }

  if (PRINTRXPULSES)
  {
    Serial.print(rx_pulses[0]); Serial.print(" ");
    Serial.print(rx_pulses[1]); Serial.print(" ");
    Serial.print(rx_pulses[2]); Serial.print(" ");
    Serial.print(rx_pulses[3]); Serial.print(" ");
  }

  // Turn all of motor pulses on.
  PORTD |= B00111100;

  motorStartTime = micros();

  // This call takes ~750us, motor pulses must last for at least 1000us, so use
  // 750us of that time to get gyro data instead of doing nothing.
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
