#include "pid.hpp"
#include "propagateimu.hpp"
#include <Wire.h>
#define CYCLELEN 4000 // Cycle length in microseconds
#define NUMRECEIVERCHANNELS 4

#define PRINTRXPULSES 0
#define PRINTSETPOINTS 0
#define PRINTIMUDEGINPUT 1
#define PRINTIMUGYROUPDATE 0
#define PRINTIMURADINPUT 0
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
  unsigned int i = 0;
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(true);
  Wire.requestFrom(0x68, 14);

  while(Wire.available() < 8);
  
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
    // The factor of 400 exists to make numerical integration of angular rates
    // from gyro roughly equal to accelerometer measurements (see mpu6050compfilt.ino).
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
  const float * phi_degps_bias,
  const float * theta_degps_bias,
  const float * psi_degps_bias,
  const float dt,
  float * phi_deg_out,   // Previous measurement of phi, will be updated
  float * theta_deg_out, // Previous measurement of theta, will be updated
  float * psi_degps_out  // Previous measurement of psi, will be updated
)
{
  static float acc_meas[3]      = {0., 0., 0.};
  static float gyro_meas_degps[3] = {0., 0., 0.};
  static float gyro_filt_degps[3] = {0., 0., 0.};
  static float gyro_filt_radps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  static float phi_gyro_rad     = 0.;
  static float theta_gyro_rad   = 0.;
  const static float alpha      = 0.9996;
  const static float beta       = 0.7;

  //static float phi_prev_deg = 0.;
  //static float theta_prev_deg = 0.;
  //static float psi_prev_degps = 0.;

  static float phi_gyro_temp_deg = 0.;
  static float theta_gyro_temp_deg = 0.;

  static float sin_omega_z_dt = 0.;

  //phi_prev_deg = *phi_deg_out;
  //theta_prev_deg = *theta_deg_out;
  //psi_prev_degps = *psi_degps_out;

  getImuData(acc_meas, gyro_meas_degps);

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

/*
  gyro_filt_radps[0] = gyro_filt_degps[0]*M_PI/180.;
  gyro_filt_radps[1] = gyro_filt_degps[1]*M_PI/180.;
  gyro_filt_radps[2] = gyro_filt_degps[2]*M_PI/180.;

  propagatePitchRoll(phi_prev_rad, theta_prev_rad, gyro_filt_radps, dt, phi_gyro_rad, theta_gyro_rad);
*/

  //*psi_degps_out = beta*(*psi_degps_out) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);
  *psi_degps_out = gyro_filt_degps[2];

  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  theta_acc = atan2(-1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;

  phi_gyro_temp_deg = *phi_deg_out + gyro_filt_degps[0]*dt;
  theta_gyro_temp_deg = *theta_deg_out + gyro_filt_degps[1]*dt;

  sin_omega_z_dt = sin(gyro_filt_degps[2]*dt*M_PI/180.);
  phi_gyro_temp_deg -= theta_gyro_temp_deg*sin_omega_z_dt;
  theta_gyro_temp_deg += phi_gyro_temp_deg*sin_omega_z_dt;

  //*phi_deg_out = alpha*(*phi_deg_out + (gyro_meas_degps[0] - (*phi_degps_bias))*dt) + (1 - alpha)*(phi_acc);
  //*phi_deg_out = alpha*(phi_gyro_rad*180./M_PI) + (1 - alpha)*(phi_acc);
  //*phi_deg_out = phi_gyro_rad*180./M_PI;
  *phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;

  //*theta_deg_out = alpha*(*theta_deg_out + (gyro_meas_degps[1] - (*theta_degps_bias))*dt) + (1 - alpha)*(theta_acc);
  //*theta_deg_out = alpha*(theta_gyro_rad*180./M_PI) + (1 - alpha)*(theta_acc);
  //*theta_deg_out = theta_gyro_rad*180./M_PI;
  *theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;
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

  static unsigned long loopTime = 0;

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

  dt = (t - cycleStartTime)/1e6;
  cycleStartTime = micros();
  updateAngleCalculations(
    &phi_rate_gyr_bias, &theta_rate_gyr_bias, &psi_rate_gyr_bias, dt,
    &phi_meas, &theta_meas, &psi_rate_meas
  );

  if (PRINTIMUDEGINPUT)
  {
    Serial.print(phi_meas);      Serial.print(" ");
    Serial.print(theta_meas);    Serial.print(" ");
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
  motor_pulses[0] = 0;//rx_pulses[2];
  motor_pulses[1] = 0;//rx_pulses[2];
  motor_pulses[2] = 0;//rx_pulses[2];
  motor_pulses[3] = 0;//rx_pulses[2];
  
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
  //while (cycleStartTime + CYCLELEN > micros());

  if (PRINTRXPULSES || PRINTMOTORPULSES || PRINTIMUDEGINPUT || PRINTIMURADINPUT || PRINTCYCLELENGTH || PRINTPIDOUTPUT || PRINTPIDCONTROL || PRINTSETPOINTS || PRINTMOTORTIMERS)
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