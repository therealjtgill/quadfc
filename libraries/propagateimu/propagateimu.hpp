#ifndef PROPAGATEIMU
#define PROPAGATEIMU

#include <math.h>
#include <stdint.h>
#include "quaternion.hpp"

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
  static float gyro_filt_degps[3] = {0., 0., 0.};
  static float gyro_prev_degps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  static float theta_gyro_rad   = 0.;
  const static float alpha      = 0.9996;
  const static float beta       = 0.5;

  static float phi_prev_deg = 0.;
  static float theta_prev_deg = 0.;

  // Keep the previous measurements of phi_dot and theta_dot for trapezoidal
  // integration.
  static float phi_prev_degps = 0.;
  static float theta_prev_degps = 0.;

  static float phi_gyro_temp_deg = 0.;
  static float theta_gyro_temp_deg = 0.;

  static float sin_omega_z_dt = 0.;

  static Quaternion omega_body_quat_prev(1.0, 0., 0., 0.);
  static Quaternion omega_body_quat;
  static Quaternion omega_global_quat;
  static Quaternion delta_pose_global_quat;
  static Quaternion pose_global_quat(1., 0., 0., 0.);

  static double delta_alpha = 0.;

  static int i = 0;

  ++i;

  phi_prev_deg = *phi_deg_out;
  theta_prev_deg = *theta_deg_out;
  //psi_prev_degps = *psi_degps_out;

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  omega_body_quat_prev[0] = 0.;
  omega_body_quat_prev[1] = *phi_degps_out*M_PI/180.;
  omega_body_quat_prev[2] = *theta_degps_out*M_PI/180.;
  omega_body_quat_prev[3] = *psi_degps_out*M_PI/180.;

  *phi_degps_out   = gyro_filt_degps[0];
  *theta_degps_out = gyro_filt_degps[1];
  *psi_degps_out   = gyro_filt_degps[2];

  //delta_alpha = sqrt(
  //  gyro_filt_degps[0]*gyro_filt_degps[0] +
  //  gyro_filt_degps[1]*gyro_filt_degps[1] +
  //  gyro_filt_degps[2]*gyro_filt_degps[2]
  //)*dt*M_PI/180.;

  //static double cosda = cos(delta_alpha/2.);
  //static double sinda = sin(delta_alpha/2.);

  omega_body_quat[0] = 0.;
  omega_body_quat[1] = gyro_filt_degps[0]*M_PI/180.;
  omega_body_quat[2] = gyro_filt_degps[1]*M_PI/180.;
  omega_body_quat[3] = gyro_filt_degps[2]*M_PI/180.;
  //omega_body_quat[0] = cosda;
  //omega_body_quat[1] = sinda*gyro_filt_degps[0]/(delta_alpha*180/(dt*M_PI));
  //omega_body_quat[2] = sinda*gyro_filt_degps[1]/(delta_alpha*180/(dt*M_PI));
  //omega_body_quat[3] = sinda*gyro_filt_degps[2]/(delta_alpha*180/(dt*M_PI));
/*
  Serial.print(omega_body_quat[0]); Serial.print(" ");
  Serial.print(omega_body_quat[1]); Serial.print(" ");
  Serial.print(omega_body_quat[2]); Serial.print(" ");
  Serial.print(omega_body_quat[3]); Serial.println(" ");
*/
  //omega_global_quat = pose_global_quat*omega_body_quat*(~pose_global_quat);
  

  //Serial.println(delta_alpha);

  //delta_pose_global_quat[0] = cosda;
  //delta_pose_global_quat.vector() = sinda*omega_global_quat.vector()*dt/delta_alpha;
  //delta_pose_global_quat.vector() = sinda*omega_body_quat.vector()*dt*M_PI/(delta_alpha*180.);

  //pose_global_quat = pose_global_quat*delta_pose_global_quat;
  //pose_global_quat = delta_pose_global_quat*pose_global_quat;
  //pose_global_quat = omega_body_quat_prev*pose_global_quat*(~omega_body_quat_prev);
  //pose_global_quat = omega_body_quat_prev; //0.5*dt*(gyro_filt_degps[0] - gyro_prev_degps[0])
  pose_global_quat = pose_global_quat*exp(0.5*(dt*omega_body_quat + 0.5*dt*(omega_body_quat - omega_body_quat_prev))) + 0.1*(1.0 - pose_global_quat.magnitudeSquared())*pose_global_quat*dt;
  //pose_global_quat /= pose_global_quat.magnitude();

  // Calculate phi and theta according to accelerometer output.
  phi_acc = atan2(acc_meas[1], acc_meas[2])*180./M_PI + 180;
  if (phi_acc > 180.)
  {
    phi_acc -= 360;
  }
  theta_acc = atan2(1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*180./M_PI;

  // Serial.print(phi_acc); Serial.print(" ");
  // Serial.println(theta_acc);

  if (only_accel)
  {
    *phi_deg_out = phi_acc;
    *theta_deg_out = theta_acc;

    gyro_prev_degps[0] = gyro_filt_degps[0];
    gyro_prev_degps[1] = gyro_filt_degps[1];
    gyro_prev_degps[2] = gyro_filt_degps[2];

    // Get global quaternion estimate from accelerometer readings.
    // I'm using a 1-2-3 Tait-Bryan rotation matrix.
    // Pose quaternion needs to be initialized with this.
    // Halved for quaternion conversion.
    float costheta = cos(theta_acc*M_PI/360.);
    float sintheta = sin(theta_acc*M_PI/360.);

    float cosphi = cos(phi_acc*M_PI/360.);
    float sinphi = sin(phi_acc*M_PI/360.);

    omega_global_quat[0] =  costheta*cosphi;
    omega_global_quat[1] =  costheta*sinphi;
    omega_global_quat[2] =  sintheta*cosphi;
    omega_global_quat[3] =  sintheta*sinphi;

    return;
  }

  // Calculate phi and theta according to gyro output.
  /*
  phi_gyro_temp_deg = phi_prev_deg + gyro_prev_degps[0]*dt + 0.5*dt*(gyro_filt_degps[0] - gyro_prev_degps[0]);
  theta_gyro_temp_deg = theta_prev_deg + gyro_prev_degps[1]*dt + 0.5*dt*(gyro_filt_degps[1] - gyro_prev_degps[1]);

  sin_omega_z_dt = sin(gyro_prev_degps[2]*dt*M_PI/180.);
  phi_gyro_temp_deg -= theta_gyro_temp_deg*sin_omega_z_dt;
  theta_gyro_temp_deg += phi_gyro_temp_deg*sin_omega_z_dt;
  */
/*
  phi_gyro_temp_deg = atan2(
    -2.*(pose_global_quat[2]*pose_global_quat[3] - pose_global_quat[1]*pose_global_quat[0]),
    1 - 2.*(pose_global_quat[1]*pose_global_quat[1] + pose_global_quat[2]*pose_global_quat[2])
  )*180./M_PI;

  theta_gyro_temp_deg = asin(
    2.*(pose_global_quat[1]*pose_global_quat[3] + pose_global_quat[2]*pose_global_quat[0])
  )*180./M_PI;
*/

  phi_gyro_temp_deg = atan2(
    2.*(pose_global_quat[2]*pose_global_quat[3] + pose_global_quat[1]*pose_global_quat[0]),
    1 - 2.*(pose_global_quat[1]*pose_global_quat[1] + pose_global_quat[2]*pose_global_quat[2])
  )*180./M_PI;

  theta_gyro_temp_deg = asin(
    2.*(pose_global_quat[0]*pose_global_quat[2] - pose_global_quat[1]*pose_global_quat[3])
  )*180./M_PI;

  // Complementary filter of gyro and accelerometer.
  *phi_deg_out = alpha*phi_gyro_temp_deg + (1 - alpha)*phi_acc;
  *theta_deg_out = alpha*theta_gyro_temp_deg + (1 - alpha)*theta_acc;

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