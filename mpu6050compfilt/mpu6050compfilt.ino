#include <Wire.h>
#include <math.h>
#include <propagateimu.hpp>

void setup(void)
{
  Wire.begin();
  TWBR = 12;

  Serial.println("Is this displaying?");
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
    gyro_meas_out[i] = (float)((Wire.read() << 8) | Wire.read())/65.5;
  }
}

void calculateGyroBiases(float * phi_bias, float * theta_bias, float * psi_bias)
{
  float acc[3] = {0., 0., 0.};
  float gyro[3] = {0., 0., 0.};

  *phi_bias = 0.;
  *theta_bias = 0.;
  *psi_bias = 0.;
  for (unsigned int i = 0; i < 1000; ++i)
  {
    getImuData(acc, gyro);
    *phi_bias += gyro[0];
    *theta_bias += gyro[1];
    *psi_bias += gyro[2];
  }

  *phi_bias /= 1000.;
  *theta_bias /= 1000.;
  *psi_bias /= 1000.;
}

void loop(void)
{
  static float acc[3];
  static float gyro[3];

  // Accelerometer
  //-----------------------
  static float phi_acc = 0.;
  static float theta_acc = 0.;
  //-----------------------

  // Gyroscope
  //-----------------------
  static float phi_gyr = 0.;
  static float phi_gyr_bias = 0.;

  static float theta_gyr = 0.;
  static float theta_gyr_bias = 0.;

  static float psi_gyr = 0.;
  static float psi_gyr_bias = 0.;
  //-----------------------

  // Fused measurements
  //-----------------------
  static float phi = 0.;
  static float theta = 0.;
  static float psi_rate = 0.;
  //-----------------------

  static double t = 0.0;
  static double dt = 0.01;
  static float alpha = 0.9996;

  static float gyro_rad[3] = {0., 0., 0.};
  static float phi_rad = 0.;
  static float theta_rad = 0.;

  static float phi_gyr_rad = 0.;
  static float theta_gyr_rad = 0.;

  static bool initialized = false;

  // Calibrate the gyro by collecting a series of measurements
  // and averaging them.
  if (!initialized)
  {
    calculateGyroBiases(&phi_gyr_bias, &theta_gyr_bias, &psi_gyr_bias);
    initialized = true;
    t = micros();
  }

  dt = (micros() - t)/1e6;
  t = micros();
  getImuData(acc, gyro);
  gyro_rad[0] = (0.7*gyro_rad[0] + 0.3*(gyro[0] - phi_gyr_bias))*M_PI/180.;
  gyro_rad[1] = (0.7*gyro_rad[1] + 0.3*(gyro[1] - theta_gyr_bias))*M_PI/180.;
  gyro_rad[2] = (0.7*gyro_rad[2] + 0.3*(gyro[2] - psi_gyr_bias))*M_PI/180.;

  phi_rad = phi*M_PI/180.;
  theta_rad = theta*M_PI/180.;
/*
  phi_acc = atan2(acc[1], acc[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc = phi_acc - 360;
  }
*/
  propagatePitchRoll(theta_rad, phi_rad, gyro_rad, dt, theta_gyr_rad, phi_gyr_rad);
  phi_acc = atan2(-1.*acc[1], sqrt(acc[1]*acc[1] + acc[2]*acc[2]))*180./M_PI;
  //phi_gyr += (gyro[0] - phi_gyr_bias)*dt;
  phi_gyr = phi_gyr_rad*180./M_PI;
  //phi      = alpha*(phi + (gyro[0] - phi_gyr_bias)*dt)   + (1 - alpha)*(phi_acc);
  phi = alpha*phi_gyr + (1 - alpha)*phi_acc;
  
//  theta_acc  = atan2(-1.*acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]))*180./M_PI;
  theta_acc = atan2(-1.*acc[0], acc[2])*180./M_PI + 180;
  if (theta_acc > 180.)
  {
    theta_acc -= 360.;
  }
  //theta_gyr += (gyro[1] - theta_gyr_bias)*dt;
  theta_gyr = theta_gyr_rad*180./M_PI;
  //theta      = alpha*(theta + (gyro[1] - theta_gyr_bias)*dt) + (1 - alpha)*(theta_acc);
  theta = alpha*theta_gyr + (1 - alpha)*theta_acc;

  psi_gyr += (gyro[2] - psi_gyr_bias)*dt;
  psi_rate = gyro[2] - psi_gyr_bias;

  //Serial.print(phi_acc);
  //Serial.print(" ");
  Serial.print(phi_gyr);
  //Serial.print(theta);
  //Serial.print(gyro[0] - phi_gyr_bias);
  Serial.print(" ");
  //Serial.print(gyro[1] - theta_gyr_bias);
  //Serial.print(theta_acc);
  Serial.print(theta_gyr);
  //Serial.print(phi);
  //Serial.print(" ");
  //Serial.print(M_PI/180.);
  //Serial.print(psi_rate);
  //Serial.print(" ");
  Serial.println("");
  
  while ((micros() - t) < 4000);
  //Serial.println(micros() - t);
}

