#ifndef PROPAGATEIMU
#define PROPAGATEIMU

#include <math.h>
#include <stdint.h>

// Forward declaration; actual definition is in *.ino file.
void getImuData(float * acc_meas_out, float * gyro_meas_out);

// omega_body is an array with three elements.
void propagatePitchRoll(
   const float & pitch_in,
   const float & roll_in,
   const float * omega_body,
   const float & dt,
   float & pitch_out,
   float & roll_out
)
{
   // The last row of the Rodrigues Rotation matrix.
   static float rod2[3] = {0., 0., 0.};
   static float omega_magnitude = 0.;
   static float omega_global_unit[3] = {0., 0., 0.};

   static float tanpitch_n = 0.;
   static float tanpitch_d = 0.;
   static float sinroll = 0.;

   // sin(omega_body_magnitude*dt)
   static float sa = 0.;
   // cos(omega_body_magnitude*dt)
   static float ca = 0.;

   static float sp = 0;
   static float cp = 0;
   static float sr = 0;
   static float cr = 0;

   omega_magnitude = sqrt(
      omega_body[0]*omega_body[0] +
      omega_body[1]*omega_body[1] +
      omega_body[2]*omega_body[2]
   );

   sp = sin(pitch_in);
   cp = cos(pitch_in);
   sr = sin(roll_in);
   cr = cos(roll_in);

   // The magnitude of omega is the angular speed of rotation. An approximate
   // delta-alpha is achieved by multiplying angular speed by dt.
   sa = sin(omega_magnitude*dt);
   ca = cos(omega_magnitude*dt);
   
   // Calculate angular velocity in global frame
   omega_global_unit[0] = (omega_body[0]*cp + omega_body[2]*sp)/omega_magnitude;
   omega_global_unit[1] = (omega_body[0]*sp*sr + omega_body[1]*cr - omega_body[2]*sr*cp)/omega_magnitude;
   omega_global_unit[2] = (-1.*omega_body[0]*sp*cr + omega_body[1]*sr + omega_body[2]*cp*cr)/omega_magnitude;

   //Calculate last row of Rodrigues rotation matrix
   rod2[0] = -1.*sa*omega_global_unit[1] + (1 - ca)*(omega_global_unit[0]*omega_global_unit[2]);
   rod2[1] =     sa*omega_global_unit[0] + (1 - ca)*(omega_global_unit[1]*omega_global_unit[2]);
   rod2[2] =                           1 + (1 - ca)*(omega_global_unit[2]*omega_global_unit[2] - 1);

   tanpitch_n = (rod2[2]*cr*sp - rod2[0]*cp - rod2[1]*sr*sp);
   tanpitch_d = (rod2[0]*sp - rod2[1]*sr*cp + rod2[2]*cp*cr);
   sinroll    = rod2[1]*cr + rod2[2]*sr;

   pitch_out = atan2(tanpitch_n, tanpitch_d);
   roll_out = asin(sinroll);
}

#ifndef USETRAPEZOIDAL
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
  static float gyro_prev_degps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  static float theta_gyro_rad   = 0.;
  const static float alpha      = 0.9996;
  const static float beta       = 0.5;

  static float phi_prev_deg = 0.;
  static float theta_prev_deg = 0.;
  //static float psi_prev_degps = 0.;

  // Keep the previous measurements of phi_dot and theta_dot for trapezoidal
  // integration.
  static float phi_prev_degps = 0.;
  static float theta_prev_degps = 0.;

  static float phi_gyro_temp_deg = 0.;
  static float theta_gyro_temp_deg = 0.;

  static float sin_omega_z_dt = 0.;

  phi_prev_deg = *phi_deg_out;
  theta_prev_deg = *theta_deg_out;
  //psi_prev_degps = *psi_degps_out;

  getImuData(acc_meas, gyro_meas_degps);

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  *psi_degps_out = gyro_filt_degps[2];

  // Calculate phi and theta according to accelerometer output.
  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  theta_acc = atan2(-1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;

  // Calculate phi and theta according to gyro output.
  //phi_gyro_temp_deg = phi_prev_deg + gyro_filt_degps[0]*dt;
  phi_gyro_temp_deg = phi_prev_deg + gyro_prev_degps[0]*dt + 0.5*dt*(gyro_filt_degps[0] - gyro_prev_degps[0]);
  //theta_gyro_temp_deg = theta_prev_deg + gyro_filt_degps[1]*dt;
  theta_gyro_temp_deg = theta_prev_deg + gyro_prev_degps[1]*dt + 0.5*dt*(gyro_filt_degps[1] - gyro_prev_degps[1]);

  //sin_omega_z_dt = sin((gyro_prev_degps[2] + 0.5*(gyro_filt_degps[2] - gyro_prev_degps[2]))*dt*M_PI/180.);
  sin_omega_z_dt = sin(gyro_prev_degps[2]*dt*M_PI/180.);
  phi_gyro_temp_deg -= theta_gyro_temp_deg*sin_omega_z_dt;
  theta_gyro_temp_deg += phi_gyro_temp_deg*sin_omega_z_dt;

  // Complementary filter of gyro and accelerometer.
  *phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;
  *theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;

  gyro_prev_degps[0] = gyro_filt_degps[0];
  gyro_prev_degps[1] = gyro_filt_degps[1];
  gyro_prev_degps[2] = gyro_filt_degps[2];
}

#else
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

  //*phi_deg_out = alpha*(*phi_deg_out + gyro_filt_degps[0]*dt) + (1 - alpha)*(phi_acc);
  //*phi_deg_out = alpha*(phi_gyro_rad*180./M_PI) + (1 - alpha)*(phi_acc);
  *phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;

  //*theta_deg_out = alpha*(*theta_deg_out + gyro_filt_degps[1]*dt) + (1 - alpha)*(theta_acc);
  //*theta_deg_out = alpha*(theta_gyro_rad*180./M_PI) + (1 - alpha)*(theta_acc);
  *theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;
}
#endif

#endif