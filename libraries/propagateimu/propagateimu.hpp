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

void quatMult(const float * a, const float * b, float * c)
{
  c[0] = a[0]*b[0] - (a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
  c[1] = a[0]*b[1] + b[0]*a[1] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] + b[0]*a[2] + a[3]*b[1] - a[1]*b[3];
  c[3] = a[0]*b[3] + b[0]*a[3] + a[1]*b[2] - a[2]*b[1];
}

// Need to rotate omega_body into lab frame, note that the current quaternion
// keeping track of the frame in lab frame has to be inverted.
//  omega_lab = (q_lab_to_body^-1) x omega_body x (q_lab_to_body)
void updateAngleCalculationsQuaternion(
  const float * acc_meas,
  const float * gyro_meas_degps,
  const float * phi_degps_bias,
  const float * theta_degps_bias,
  const float * psi_degps_bias,
  const float dt,
  float * phi_deg_out,     // Previous measurement of phi, will be updated
  float * theta_deg_out,   // Previous measurement of theta, will be updated
  float * phi_degps_out,   // Will be updated with filtered gyro output
  float * theta_degps_out, // Will be updated with filtered gyro output
  float * psi_degps_out,   // Previous measurement of psi rate, will be updated
  bool only_accel=false    // If true, will only return accel angle estimates
)
{
  static const float DEGTORAD = M_PI/180.;
  static const float RADTODEG = 180./M_PI;

  static float gyro_filt_degps[3] = {0., 0., 0.};
  static float gyro_prev_degps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  const static float alpha      = 0.9996;
  const static float beta       = 0.7;

  static float phi_prev_deg = 0.;
  static float theta_prev_deg = 0.;

  // Keep the previous measurements of phi_dot and theta_dot for trapezoidal
  // integration.
  static float phi_prev_degps = 0.;
  static float theta_prev_degps = 0.;

  static float phi_gyro_temp_deg = 0.;
  static float theta_gyro_temp_deg = 0.;

  static float sin_omega_z_dt = 0.;

  static float delta_omega_body_radps[3] = {0., 0., 0.};
  //static float delta_omega_global_radps[3] = {0., 0., 0.};
  static float q_body[4] = {1., 0., 0., 0.}; // Orientation quaternion in global frame.
  //static float v0, v1, v2, v3;
  //static float s0, s1, s2, s3;
  static float v[4] = {0., 0., 0., 0.};
  static float s[4] = {0., 0., 0., 0.};

  static float delta_omega_mag = 0.;
  static float delta_omega_mag_squared = 0.;
  static float cosdam = 0.;
  static float sindam = 0.;
  static float sindam_over_dam = 0.;

  static float pose_quat_mag_squared = 0.;
  static float correction = 0.;

  //1300us to complete entire function call

  phi_prev_deg = *phi_deg_out;
  theta_prev_deg = *theta_deg_out;
  //psi_prev_degps = *psi_degps_out;

  //1
  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  *phi_degps_out   = gyro_filt_degps[0];
  *theta_degps_out = gyro_filt_degps[1];
  *psi_degps_out   = gyro_filt_degps[2];

  delta_omega_body_radps[0] = (1.5*gyro_filt_degps[0] - 0.5*gyro_prev_degps[0])*DEGTORAD;
  delta_omega_body_radps[1] = (1.5*gyro_filt_degps[1] - 0.5*gyro_prev_degps[1])*DEGTORAD;
  delta_omega_body_radps[2] = (1.5*gyro_filt_degps[2] - 0.5*gyro_prev_degps[2])*DEGTORAD;
  
  delta_omega_mag = sqrt(
    delta_omega_body_radps[0]*delta_omega_body_radps[0] + 
    delta_omega_body_radps[1]*delta_omega_body_radps[1] + 
    delta_omega_body_radps[2]*delta_omega_body_radps[2]
  );

  //delta_omega_mag_squared = (
  //  delta_omega_body_arr[0]*delta_omega_body_arr[0] + 
  //  delta_omega_body_arr[1]*delta_omega_body_arr[1] + 
  //  delta_omega_body_arr[2]*delta_omega_body_arr[2]
  //)*DEGTORAD*DEGTORAD*0.25*dt*dt;
  //Serial.println(delta_omega_mag_squared, 16);
  //delta_omega_mag_squared = (delta_omega_mag_squared > 1e-8) ? delta_omega_mag_squared : 0.;
  //1 300us
  
  /*
  // The faster delta calculation.
  cosdam = 1.0 - delta_omega_mag_squared/2. + delta_omega_mag_squared*delta_omega_mag_squared/24;
  if (delta_omega_mag_squared == 0.)
  {
    sindam_over_dam = 0.;
  }
  else
  {
    sindam_over_dam = 1.0 - delta_omega_mag_squared/6. + delta_omega_mag_squared*delta_omega_mag_squared/120.;
  }
  
  v0 = cosdam;
  v1 = sindam_over_dam*0.5*dt*gyro_filt_degps[0]*DEGTORAD;
  v2 = sindam_over_dam*0.5*dt*gyro_filt_degps[1]*DEGTORAD;
  v3 = sindam_over_dam*0.5*dt*gyro_filt_degps[2]*DEGTORAD;
  */

  static double inv_delta_omega_mag = 0.;
  inv_delta_omega_mag = 1./delta_omega_mag;
  cosdam = cos(delta_omega_mag*0.5*dt);
  sindam = sin(delta_omega_mag*0.5*dt);

  v[0] = cosdam;
  v[1] = sindam*delta_omega_body_radps[0]*inv_delta_omega_mag;
  v[2] = sindam*delta_omega_body_radps[1]*inv_delta_omega_mag;
  v[3] = sindam*delta_omega_body_radps[2]*inv_delta_omega_mag;

  s[0] = q_body[0];
  s[1] = q_body[1];
  s[2] = q_body[2];
  s[3] = q_body[3];
  
  q_body[0] = s[0]*v[0] - (s[1]*v[1] + s[2]*v[2] + s[3]*v[3]);
  q_body[1] = s[0]*v[1] + v[0]*s[1] + s[2]*v[3] - s[3]*v[2];
  q_body[2] = s[0]*v[2] + v[0]*s[2] + s[3]*v[1] - s[1]*v[3];
  q_body[3] = s[0]*v[3] + v[0]*s[3] + s[1]*v[2] - s[2]*v[1];
  //3 272us
  
  //Serial.print(q_body[0]); Serial.print(" ");
  //Serial.print(q_body[1]); Serial.print(" ");
  //Serial.print(q_body[2]); Serial.print(" ");
  //Serial.print(q_body[3]); Serial.println(" ");

  static float accel_g2b_angle = 0.;
  static float accel_mag = 0.;
  static float n[3] = {0., 0., 0.};
  static float n_mag = 0.;
  accel_mag = sqrt(
    acc_meas[0]*acc_meas[0] +
    acc_meas[1]*acc_meas[1] +
    acc_meas[2]*acc_meas[2]
  );
  accel_g2b_angle = acos(-1.*acc_meas[2]/accel_mag);

  n[0] = -1.*acc_meas[1];
  n[1] = acc_meas[0];
  n[2] = 0.;
  n_mag = sqrt(
    n[0]*n[0] + n[1]*n[1]
  ) + 1e-8;

  static float delta_omega_acc[4] = {0., 0., 0., 0.};
  static float cosdoa = 0.;
  static float sindoa = 0.;
  cosdoa = cos((1 - alpha)*accel_g2b_angle*0.5);
  sindoa = sin((1 - alpha)*accel_g2b_angle*0.5);
  delta_omega_acc[0] = cosdoa;
  delta_omega_acc[1] = sindoa*n[0]/n_mag;
  delta_omega_acc[2] = sindoa*n[1]/n_mag;
  delta_omega_acc[3] = sindoa*n[2]/n_mag;

  s[0] = q_body[0];
  s[1] = q_body[1];
  s[2] = q_body[2];
  s[3] = q_body[3];
  
  v[0] = delta_omega_acc[0];
  v[1] = delta_omega_acc[1];
  v[2] = delta_omega_acc[2];
  v[3] = delta_omega_acc[3];

  //q_body[0] = s[0]*v[0] - (s[1]*v[1] + s[2]*v[2] + s[3]*v[3]);
  //q_body[1] = s[0]*v[1] + v[0]*s[1] + s[2]*v[3] - s[3]*v[2];
  //q_body[2] = s[0]*v[2] + v[0]*s[2] + s[3]*v[1] - s[1]*v[3];
  //q_body[3] = s[0]*v[3] + v[0]*s[3] + s[1]*v[2] - s[2]*v[1];

  pose_quat_mag_squared = 
    q_body[0]*q_body[0] +
    q_body[1]*q_body[1] +
    q_body[2]*q_body[2] +
    q_body[3]*q_body[3];

  if (pose_quat_mag_squared < 0.9801)
  {
    correction = 0.1*(1.0 - pose_quat_mag_squared)*dt;

    q_body[0] += correction*q_body[0];
    q_body[1] += correction*q_body[1];
    q_body[2] += correction*q_body[2];
    q_body[3] += correction*q_body[3];
  }

  if (only_accel)
  {
      // Calculate phi and theta according to accelerometer output.
    phi_acc = atan2(acc_meas[1], acc_meas[2])*RADTODEG + 180;
    if (phi_acc > 180.)
    {
      phi_acc -= 360;
    }
    
    theta_acc = atan2(
      1.*acc_meas[0],
      sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2])
    )*RADTODEG;

    *phi_deg_out = phi_acc;
    *theta_deg_out = theta_acc;

    gyro_prev_degps[0] = gyro_filt_degps[0];
    gyro_prev_degps[1] = gyro_filt_degps[1];
    gyro_prev_degps[2] = gyro_filt_degps[2];

    float costheta = cos(theta_acc*DEGTORAD/2.);
    float sintheta = sin(theta_acc*DEGTORAD/2.);

    float cosphi = cos(phi_acc*DEGTORAD/2.);
    float sinphi = sin(phi_acc*DEGTORAD/2.);

    q_body[0] =  costheta*cosphi;
    q_body[1] =  costheta*sinphi;
    q_body[2] =  sintheta*cosphi;
    q_body[3] =  -1.*sintheta*sinphi;

    return;
  }

  //5
  phi_gyro_temp_deg = atan2(
    2.*(q_body[2]*q_body[3] + q_body[1]*q_body[0]),
    1 - 2.*(q_body[1]*q_body[1] + q_body[2]*q_body[2])
  )*RADTODEG;
  //5 300us

  //Serial.print(2.*(q_body[2]*q_body[3] + q_body[1]*q_body[0])); Serial.print(" ");
  //Serial.print(1 - 2.*(q_body[1]*q_body[1] + q_body[2]*q_body[2])); Serial.println(" ");

  //6
  theta_gyro_temp_deg = asin(
    2.*(q_body[0]*q_body[2] - q_body[1]*q_body[3])
  )*RADTODEG;
  //6 132us
 
  // Complementary filter of gyro and accelerometer.
  //*phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;
  //*theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;

  *phi_deg_out = phi_gyro_temp_deg;
  *theta_deg_out = theta_gyro_temp_deg;

  gyro_prev_degps[0] = gyro_filt_degps[0];
  gyro_prev_degps[1] = gyro_filt_degps[1];
  gyro_prev_degps[2] = gyro_filt_degps[2];
}

/////////////////////////////////////////////////
// updateAngleCalculations
/////////////////////////////////////////////////
void updateAngleCalculations(
  const float * acc_meas,
  const float * gyro_meas_degps,
  const float * phi_degps_bias,
  const float * theta_degps_bias,
  const float * psi_degps_bias,
  const float dt,
  float * phi_deg_out,     // Previous measurement of phi, will be updated
  float * theta_deg_out,   // Previous measurement of theta, will be updated
  float * phi_degps_out,   // Will be updated with filtered gyro output
  float * theta_degps_out, // Will be updated with filtered gyro output
  float * psi_degps_out,   // Previous measurement of psi rate, will be updated
  bool only_accel=false    // If true, will only return accel angle estimates
)
{
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

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  *phi_degps_out   = gyro_filt_degps[0];
  *theta_degps_out = gyro_filt_degps[1];
  *psi_degps_out   = gyro_filt_degps[2];

  // Calculate phi and theta according to accelerometer output.
  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  theta_acc = atan2(-1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;

  // Serial.print(phi_acc); Serial.print(" ");
  // Serial.println(theta_acc);

  if (only_accel)
  {
    *phi_deg_out = phi_acc;
    *theta_deg_out = theta_acc;

    gyro_prev_degps[0] = gyro_filt_degps[0];
    gyro_prev_degps[1] = gyro_filt_degps[1];
    gyro_prev_degps[2] = gyro_filt_degps[2];
    return;
  }

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


#endif