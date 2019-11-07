#ifndef PROPAGATEIMU
#define PROPAGATEIMU

#include <math.h>
#include <stdint.h>

void getImuData(float * acc_meas_out, float * gyro_meas_out);

// Helper to convert from 2d array indices to linear array indices.
uint8_t ind(
   const uint8_t & i,
   const uint8_t & j
)
{
   static uint8_t max_i = 2;
   static uint8_t max_j = 2;
   return i*max_i + j;
}

// Size of R is always 9.
void pitchRollMatrix(
   const float & pitch,
   const float & roll,
   float * R
)
{
   static float sp = 0;
   static float cp = 0;
   static float sr = 0;
   static float cr = 0;

   sp = sin(pitch);
   cp = cos(pitch);
   sr = sin(roll);
   cr = cos(roll);
   R[ind(0, 0)] = cp;         R[ind(0, 1)] = 0;       R[ind(0, 2)] = sp;
   R[ind(1, 0)] = sr*sp;      R[ind(1, 1)] = cr;      R[ind(1, 2)] = -1.*sr*cp;
   R[ind(2, 0)] = -1.*cr*sp;  R[ind(2, 1)] = sr;      R[ind(2, 2)] = cp*cr;
}

// Only need the bottom row of this matrix; excluding implementation in favor
// of optimization.
//void rodriguesRotationMatrix() {}

// Size of R is always 9. This function may be superfluous given the
// implementation in propagatePitchRoll.
void extractPitchRoll(const float * R, float & pitch, float & roll)
{
   pitch = atan2(-1.*R[ind(2, 0)], R[ind(2, 2)]);
   roll  = asin(R[ind(2, 1)]);
}

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
  //static float gyro_filt_radps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  static float phi_gyro_rad     = 0.;
  static float theta_gyro_rad   = 0.;
  const static float alpha      = 0.9996;
  const static float beta       = 0.7;

  static float phi_prev_deg = 0.;
  static float theta_prev_deg = 0.;
  static float psi_prev_degps = 0.;

  static float phi_gyro_temp_deg = 0.;
  static float theta_gyro_temp_deg = 0.;

  static float sin_omega_z_dt = 0.;

  phi_prev_deg = *phi_deg_out;
  theta_prev_deg = *theta_deg_out;
  psi_prev_degps = *psi_degps_out;

  getImuData(acc_meas, gyro_meas_degps);

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  *psi_degps_out = gyro_filt_degps[2];

  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  theta_acc = atan2(-1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;
  //phi_acc = atan2(acc_meas[1], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;
  //theta_acc = atan2(-1.*acc_meas[0], acc_meas[2])*180./M_PI + 180;
  //if (theta_acc > 180.)
  //{
  //  theta_acc -= 360;
  //}

  phi_gyro_temp_deg = *phi_deg_out + gyro_filt_degps[0]*dt;
  theta_gyro_temp_deg = *theta_deg_out + gyro_filt_degps[1]*dt;

  sin_omega_z_dt = sin(gyro_filt_degps[2]*dt*M_PI/180.);
  phi_gyro_temp_deg -= theta_gyro_temp_deg*sin_omega_z_dt;
  theta_gyro_temp_deg += phi_gyro_temp_deg*sin_omega_z_dt;

  *phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;

  *theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;
}


#endif