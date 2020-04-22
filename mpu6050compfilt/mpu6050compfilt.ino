#include <Wire.h>
#include <math.h>
#include <propagateimu.hpp>
#include <EEPROM.h>
//#define CALCULATEANDSAVEGYROBIASES

void setup(void)
{
  Wire.begin();
  //TWBR = 12;

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

  int16_t byte1 = 0;
  int16_t byte2 = 0;
  
  for (i = 0; i < 3; ++i)
  {
    byte1 = Wire.read() << 8;
    byte2 = Wire.read();
    acc_meas_out[i] = -1.*static_cast<float>(((byte1) | byte2))/4096.;
//    Serial.print(byte1 >> 8);
//    Serial.print(" ");
//    //Serial.print(byte2);
  }
//  Serial.println("");

  for (i = 0; i < 2; ++i)
  {
    Wire.read();
  }

  for (i = 0; i < 3; ++i)
  {
    byte1 = Wire.read() << 8;
    byte2 = Wire.read();
    gyro_meas_out[i] = static_cast<float>((byte1) | byte2)/65.5;
    if (i == 1)
    {
      gyro_meas_out[i] *= -1;
    }
//    Serial.print(byte1 >> 8);
//    Serial.print(" ");
//    Serial.print(byte2);
  }
//  Serial.println("");
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
  // These values are hard-coded after many trials.
//  *phi_rate_bias_out   = 1.193965;
//  *theta_rate_bias_out = 1.902589;
//  *psi_rate_bias_out   = 0.2791016;
  Serial.println("Read from EEPROM:");
  int ee_address = 0;
  EEPROM.get(ee_address, *phi_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *theta_rate_bias_out);
  ee_address += sizeof(float);
  EEPROM.get(ee_address, *psi_rate_bias_out);
#endif
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

  static float phi_prev_degps = 0.;
  static float theta_prev_degps = 0.;

  static bool initialized = false;

  // Calibrate the gyro by collecting a series of measurements
  // and averaging them.
  if (!initialized)
  {
    calculateGyroBiases(&phi_gyr_bias, &theta_gyr_bias, &psi_gyr_bias);
    initialized = true;
    t = micros();
    Serial.print(phi_gyr_bias, 9); Serial.print(" ");
    Serial.print(theta_gyr_bias, 9); Serial.print(" ");
    Serial.print(psi_gyr_bias, 9); Serial.println(" ");
  }

  dt = (micros() - t)/1e6;
  t = micros();
  getImuData(acc, gyro);
//  gyro_rad[0] = (0.7*gyro_rad[0] + 0.3*(gyro[0] - phi_gyr_bias))*M_PI/180.;
//  gyro_rad[1] = (0.7*gyro_rad[1] + 0.3*(gyro[1] - theta_gyr_bias))*M_PI/180.;
//  gyro_rad[2] = (0.7*gyro_rad[2] + 0.3*(gyro[2] - psi_gyr_bias))*M_PI/180.;
//
//  phi_rad = phi*M_PI/180.;
//  theta_rad = theta*M_PI/180.;

  //propagatePitchRoll(theta_rad, phi_rad, gyro_rad, dt, theta_gyr_rad, phi_gyr_rad);
  phi_acc = atan2(-1.*acc[1], sqrt(acc[1]*acc[1] + acc[2]*acc[2]))*180./M_PI;
  phi_gyr += (gyro[0] - phi_gyr_bias)*dt + 0.5*dt*(gyro[0] - phi_prev_degps);
  theta_gyr += (gyro[1] - theta_gyr_bias)*dt + 0.5*dt*(gyro[1] - theta_prev_degps);

  //phi_gyr -= theta_gyr*sin((gyro[2] - psi_gyr_bias)*dt*M_PI/180.);
  //theta_gyr += psi_gyr*sin((gyro[2] - psi_gyr_bias)*dt*M_PI/180.);
  static float phi_gyr_temp = 0.;
  static float theta_gyr_temp = 0.;

  //phi_gyr_temp = phi + (gyro[0] - phi_gyr_bias)*dt + 0.5*dt*(gyro[0] - phi_gyr_bias - phi_prev_degps);
  phi_gyr_temp = phi + (gyro[0] - phi_gyr_bias)*dt;
  //theta_gyr_temp = theta + (gyro[1] - theta_gyr_bias)*dt + 0.5*dt*(gyro[1] - theta_gyr_bias - theta_prev_degps);
  theta_gyr_temp = theta + (gyro[1] - theta_gyr_bias)*dt;

  phi_gyr_temp -= theta_gyr_temp*sin((gyro[2] - psi_gyr_bias)*dt*M_PI/180.);
  theta_gyr_temp += phi_gyr_temp*sin((gyro[2] - psi_gyr_bias)*dt*M_PI/180.);
  
  //phi_gyr = phi_gyr_rad*180./M_PI;
  //phi      = alpha*(phi + (gyro[0] - phi_gyr_bias)*dt + 0.5*dt*(gyro[0] - phi_prev_degps))   + (1 - alpha)*(phi_acc);
  phi      = alpha*(phi_gyr_temp)   + (1 - alpha)*(phi_acc);
  //phi = alpha*phi_gyr + (1 - alpha)*phi_acc;
  
  theta_acc = atan2(-1.*acc[0], acc[2])*180./M_PI + 180;
  if (theta_acc > 180.)
  {
    theta_acc -= 360.;
  }
  
  //theta_gyr = theta_gyr_rad*180./M_PI;
  //theta      = alpha*(theta + (gyro[1] - theta_gyr_bias)*dt + 0.5*dt*(gyro[1] - theta_prev_degps)) + (1 - alpha)*(theta_acc);
  theta      = alpha*(theta_gyr_temp) + (1 - alpha)*(theta_acc);
  //theta = alpha*theta_gyr + (1 - alpha)*theta_acc;

  psi_gyr += (gyro[2] - psi_gyr_bias)*dt;
  psi_rate = gyro[2] - psi_gyr_bias;

  phi_prev_degps = gyro[0] - phi_gyr_bias;
  theta_prev_degps = gyro[1] - theta_gyr_bias;

//  Serial.print(phi_acc);
//  Serial.print(acc[0]);
//  Serial.print(" ");
//  Serial.print(acc[1]);
//  Serial.print(" ");
//  Serial.print(acc[2]);
//  Serial.print(phi_gyr);
  Serial.print(theta);
  //Serial.print(gyro[0] - phi_gyr_bias);
  Serial.print(" ");
  //Serial.print(gyro[1] - theta_gyr_bias);
  //Serial.print(theta_acc);
  //Serial.print(theta_gyr);
  Serial.print(phi);
  Serial.print(" ");
  //Serial.print(M_PI/180.);
  Serial.print(psi_rate);
  //Serial.print(" ");
  Serial.println("");
  
  //Serial.println(micros() - t);
  while ((micros() - t) < 4000);
  //Serial.println(micros() - t);
}
