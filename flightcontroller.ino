#include "pid.hpp"
#include <Wire.h>
#define CYCLELEN 4000 // Cycle length in microseconds
#define NUMRECEIVERCHANNELS 4

#define PRINTRXPULSES 0
#define PRINTSETPOINTS 0
#define PRINTIMUDEGINPUT 0
#define PRINTIMURADINPUT 0
#define PRINTPIDOUTPUT 0
#define PRINTPIDCONTROL 0
#define PRINTMOTORPULSES 1
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

  Wire.begin();

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
    //PORTD |= B00111100;
    //delayMicroseconds(1000);
    //PORTD &= B00111100;
    //delayMicroseconds(5500);
  }

  for (unsigned int i = 0; i < 4; ++i)
  {
    rx_pulses[i] = 1000;
    rx_timers[i] = 1000;
  }
}

/////////////////////////////////////////////////
// clamp
/////////////////////////////////////////////////
uint16_t clamp(uint16_t & val, uint16_t min, uint16_t max)
{
  return max(min(val, max), min);
}

/////////////////////////////////////////////////
// interpolateLinear
/////////////////////////////////////////////////
float interpolateLinear(
  float input_min,
  float input_max,
  float output_min,
  float output_max,
  float input
)
{
  float m = ((output_max - output_min)/(input_max - input_min));
  return max(
    min(
      m*input + output_min - m*input_min,
      output_max
    ),
    output_min
  );
}

/////////////////////////////////////////////////
// getImuData
/////////////////////////////////////////////////
void getImuData(float * acc_meas_out, float * gyro_meas_out)
{
  unsigned int i = 0;
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(true);
  Wire.requestFrom(0x68, 14);

  while(Wire.available() < 8);
  
  for (i = 0; i < 3; ++i)
  {
    acc_meas_out[i] = (-1.0)*((float)((Wire.read() << 8) | Wire.read()))/4096.;
    //Serial.println(acc_meas_out[i]);
  }

  for (i = 0; i < 2; ++i)
  {
    Wire.read();
  }

  for (i = 0; i < 3; ++i)
  {
    gyro_meas_out[i] = (2.0)*((float)((Wire.read() << 8) | Wire.read()))/16.4;
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
  float acc[3]  = {0., 0., 0.};
  float gyro[3] = {0., 0., 0.};

  *phi_rate_bias_out = 0.;
  *theta_rate_bias_out = 0.;
  *psi_rate_bias_out = 0.;

  for (unsigned int i = 0; i < 1000; ++i)
  {
    getImuData(acc, gyro);
    *phi_rate_bias_out += gyro[0];
    *theta_rate_bias_out += gyro[1];
    *psi_rate_bias_out += gyro[2];
  }

  *phi_rate_bias_out /= 1000.;
  *theta_rate_bias_out /= 1000.;
  *psi_rate_bias_out /= 1000.;
}

/////////////////////////////////////////////////
// updateAngleCalculations
/////////////////////////////////////////////////
void updateAngleCalculations(
  const float * phi_rate_bias,
  const float * theta_rate_bias,
  const float * psi_rate_bias,
  const float dt,
  float * phi_deg_out,
  float * theta_deg_out,
  float * psi_rate_degps_out
)
{
  static float acc_meas[3]  = {0., 0., 0.};
  static float gyro_meas[3] = {0., 0., 0.};
  static float phi_acc      = 0.;
  static float theta_acc    = 0.;
  const static float alpha  = 0.97;

  float phi_prev = *phi_deg_out;
  float theta_prev = *theta_deg_out;

  getImuData(acc_meas, gyro_meas);

  *psi_rate_degps_out = gyro_meas[2] - *psi_rate_bias;
  
  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  *phi_deg_out = alpha*(phi_prev + (gyro_meas[0] - (*phi_rate_bias))*dt) + (1 - alpha)*(phi_acc);

  theta_acc = atan2(-1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;
  *theta_deg_out = alpha*(theta_prev + (gyro_meas[1] - (*theta_rate_bias))*dt) + (1 - alpha)*(theta_acc);
}

/////////////////////////////////////////////////
// calculatePidControls
/////////////////////////////////////////////////
// x - measured value
// u - setpoint
// y - output
void calculatePidControls(
  const float * x_phi_deg,
  const float * x_theta_deg,
  const float * x_psi_rate_degps,
  const float * u_phi,
  const float * u_theta,
  const float * u_psi_rate,
  int16_t * y_phi,
  int16_t * y_theta,
  int16_t * y_psi_rate
)
{
  static PID<float> phi_pid(3.0, 0., 0.1, 0., 0., -100., 100.);
  static PID<float> theta_pid(3.0, 0., 0.1, 0., 0., -100., 100.);
  static PID<float> psi_rate_pid(3.0, 0., 0.1, 0., 0., -100., 100.);

  // Output pulse length should be between 1000 and 2000.

  float x_phi_rad = (*x_phi_deg)*(M_PI/180.);
  float x_theta_rad = (*x_theta_deg)*(M_PI/180.);
  float x_psi_rate_radps = (*x_psi_rate_degps)*(M_PI/180.);

  float phi_filtered = phi_pid.filter(x_phi_rad, *u_phi);
  float theta_filtered = theta_pid.filter(x_theta_rad, *u_theta);
  float psi_rate_filtered = psi_rate_pid.filter(x_psi_rate_radps, *u_psi_rate);

  *y_phi = static_cast<int16_t>(
    interpolateLinear(
      -10.5,
      10.5,
      -400.,
      400.,
      phi_filtered
    )
  );
  *y_theta = static_cast<int16_t>(
    interpolateLinear(
      -10.5,
      10.5,
      -400.,
      400.,
      theta_filtered
    )
  );
  *y_psi_rate = static_cast<int16_t>(
    interpolateLinear(
      -10.5,
      10.5,
      -400.,
      400.,
      psi_rate_filtered
    )
  );

  if (PRINTPIDOUTPUT)
  {
    Serial.print(phi_filtered); Serial.print(" ");
    Serial.print(theta_filtered); Serial.print(" ");
    // The psi_rate measurement is super noisy, might need to add a
    // low-pass filter to it.
    //Serial.print(psi_rate_filtered); Serial.print(" ");
  }
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
  // rx_pulses is a global array. size should always be 4.
  *u_phi_out      = interpolateLinear(1000, 2000, -M_PI/4., M_PI/4., rx_pulses[1]);
  *u_theta_out    = interpolateLinear(1000, 2000, -M_PI/4., M_PI/4., rx_pulses[0]);
  *u_psi_rate_out = interpolateLinear(1000, 2000, -M_PI/2., M_PI/2., rx_pulses[3]);
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
  tempMix = (throttle + phi_ctl - theta_ctl - 0/*psi_rate_ctl*/);
  motor_timers_out[0] = clamp(tempMix, 900, 1800);
  tempMix = (throttle - phi_ctl - theta_ctl + 0/*psi_rate_ctl*/);
  motor_timers_out[1] = clamp(tempMix, 900, 1800);
  tempMix = (throttle - phi_ctl + theta_ctl - 0/*psi_rate_ctl*/);
  motor_timers_out[2] = clamp(tempMix, 900, 1800);
  tempMix = (throttle + phi_ctl + theta_ctl + 0/*psi_rate_ctl*/);
  motor_timers_out[3] = clamp(tempMix, 900, 1800);
}

/////////////////////////////////////////////////
// Main Loop
/////////////////////////////////////////////////
void loop() {
  static float cycleStartTime = 0.;
  static float motorStartTime = 0.;
  static bool initialized = false;

  static float phi_meas = 0.;
  static float theta_meas = 0.;
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

  static float t = 0.;
  static float dt = 0.;

  static int16_t throttle = 0;
  static uint16_t motor_timers[NUMRECEIVERCHANNELS] = {0, 0, 0, 0};

  static unsigned long loopTime = 0;

  if (!initialized)
  {
    calculateGyroBiases(&phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias);
    loopTime = 0;
    initialized = true;
    t = micros();
  }

  cycleStartTime = micros();
  dt = (motorStartTime - cycleStartTime)/1e6;
  updateAngleCalculations(
    &phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias, dt,
    &phi_meas, &theta_meas, &psi_rate_meas
  );

  if (PRINTIMUDEGINPUT)
  {
    Serial.print(phi_meas);      Serial.print(" ");
    Serial.print(theta_meas);    Serial.print(" ");
    //Serial.print(psi_rate_meas); Serial.print(" ");
  }

  if (PRINTIMURADINPUT)
  {
    Serial.print(phi_meas*M_PI/180.);      Serial.print(" ");
    Serial.print(theta_meas*M_PI/180.);    Serial.print(" ");
    //Serial.print(psi_rate_meas*M_PI/180.); Serial.print(" ");
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

  if (PRINTRXPULSES)
  {
    Serial.print(rx_pulses[0]); Serial.print(" ");
    Serial.print(rx_pulses[1]); Serial.print(" ");
    Serial.print(rx_pulses[2]); Serial.print(" ");
    Serial.print(rx_pulses[3]); Serial.print(" ");
  }

  calculatePidControls(
    &phi_meas, &theta_meas, &psi_rate_meas,
    &phi_set, &theta_set, &psi_rate_set,
    &phi_ctl, &theta_ctl, &psi_rate_ctl
  );

  if (PRINTPIDCONTROL)
  {
    Serial.print(phi_ctl); Serial.print(" ");
    Serial.print(theta_ctl); Serial.print(" ");
    //Serial.print(psi_rate_ctl); Serial.print(" ");
  }

  // Throttle = rx_pulses[2]
  calculateMotorMixtures(
    phi_ctl, theta_ctl, psi_rate_ctl, rx_pulses[2], motor_timers
  );

  if (PRINTMOTORPULSES)
  {
    Serial.print(motor_timers[0]); Serial.print(" ");
    Serial.print(motor_timers[1]); Serial.print(" ");
    Serial.print(motor_timers[2]); Serial.print(" ");
    Serial.print(motor_timers[3]); Serial.print(" ");
  }

  motorStartTime = micros();

  // Turn all of motor pulses on.
  PORTD |= B00111100;

  motor_timers[0] += motorStartTime;
  motor_timers[1] += motorStartTime;
  motor_timers[2] += motorStartTime;
  motor_timers[3] += motorStartTime;

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

  if (PRINTCYCLELENGTH)
  {
    Serial.print(micros() - cycleStartTime); Serial.print(" ");
  }

  // Pause for the rest of the 2500us loop.
  while (cycleStartTime + CYCLELEN > micros());

  if (PRINTRXPULSES || PRINTMOTORPULSES || PRINTIMUDEGINPUT || PRINTIMURADINPUT || PRINTCYCLELENGTH || PRINTPIDOUTPUT || PRINTPIDCONTROL || PRINTSETPOINTS)
  {
    Serial.println("");
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
