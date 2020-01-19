#ifndef PROPAGATEIMU
#define PROPAGATEIMU

#include <math.h>
#include <stdint.h>

void quatMult(const float * a, const float * b, float * c)
{
  c[0] = a[0]*b[0] - (a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
  c[1] = a[0]*b[1] + b[0]*a[1] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] + b[0]*a[2] + a[3]*b[1] - a[1]*b[3];
  c[3] = a[0]*b[3] + b[0]*a[3] + a[1]*b[2] - a[2]*b[1];
}

void crossProduct(const float * a, const float * b, float * c)
{
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

void normalize(float * vector)
{
  static float magnitude;
  magnitude = sqrt(
    vector[0]*vector[0] + 
    vector[1]*vector[1] + 
    vector[2]*vector[2]
  );

  vector[0] /= magnitude + 1e-8;
  vector[1] /= magnitude + 1e-8;
  vector[2] /= magnitude + 1e-8;
}

void normalize(float * vector, float & magnitude)
{
  magnitude = sqrt(
    vector[0]*vector[0] + 
    vector[1]*vector[1] + 
    vector[2]*vector[2]
  );

  vector[0] /= magnitude + 1e-8;
  vector[1] /= magnitude + 1e-8;
  vector[2] /= magnitude + 1e-8;
}

// Assumes that k_hat is normalized.
void rodriguesRotation(const float * r, const float * k_hat, const float & theta, float * r_prime)
{
  static float costheta = 0.;
  static float sintheta = 0.;
  static float dotproduct = 0.;
  costheta = cos(theta);
  sintheta = sin(theta);
  dotproduct = r[0]*k_hat[0] + r[1]*k_hat[1] + r[2]*k_hat[2];

  r_prime[0] = r[0]*costheta + (k_hat[1]*r[2] - k_hat[2]*r[1])*sintheta + k_hat[0]*dotproduct*(1 - costheta);
  r_prime[1] = r[1]*costheta + (k_hat[2]*r[0] - k_hat[0]*r[2])*sintheta + k_hat[1]*dotproduct*(1 - costheta);
  r_prime[2] = r[2]*costheta + (k_hat[0]*r[1] - k_hat[1]*r[0])*sintheta + k_hat[2]*dotproduct*(1 - costheta);
}

// Assumes that axes of rotation are normalized before this function is called.
// 0 <= t <= 1.
void slerp(const float * angle1, const float * axis1, const float * angle2, const float *axis2,
  const float & t,
  float * interpAngle,
  float * interpAxis
)
{
  *interpAngle = (1. - t)*(*angle1) + (t)*(*angle2);

  static float alpha = 0.;
  alpha = (1. - t)*acos(
    axis1[0]*axis2[0] +
    axis1[1]*axis2[1] +
    axis1[2]*axis2[2]
  );

  static float k_hat[3] = {0., 0., 0.};
  static float k_mag = 0.;
  crossProduct(axis2, axis1, k_hat);
  normalize(k_hat, k_mag);
  if (k_mag < 1e-8)
  {
    interpAxis[0] = axis2[0];
    interpAxis[1] = axis2[1];
    interpAxis[2] = axis2[2];
    return;
  }

  rodriguesRotation(axis2, k_hat, alpha, interpAxis);
}

// Assumes that axes of rotation are normalized before this function is called.
// 0 <= t <= 1.
void slerp(const float * q1, const float * q2,
  const float & t,
  float * q_out
)
{
  static float costo2 = 0.;
  static float sinto2 = 0.;
  static float half_theta = 0.;
  costo2 = (
    q1[0]*q2[0] +
    q1[1]*q2[1] +
    q1[2]*q2[2] +
    q1[3]*q2[3]
  );
  if (costo2 > 0.995)
  {
    q_out[0] = q2[0];
    q_out[1] = q2[1];
    q_out[2] = q2[2];
    q_out[3] = q2[3];
    return;
  }

  sinto2 = sqrt(1. - costo2*costo2);
  half_theta = acos(costo2);

  static float a = sin((1. - t)*half_theta)/sinto2;
  static float b = sin((t)*half_theta)/sinto2;
  q_out[0] = q1[0]*a + q2[0]*b;
  q_out[1] = q1[1]*a + q2[1]*b;
  q_out[2] = q1[2]*a + q2[2]*b;
  q_out[3] = q1[3]*a + q2[3]*b;
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
  static float delta_q_omega[4] = {1., 0., 0., 0.};
  static float q_body[4] = {1., 0., 0., 0.}; // Orientation quaternion in body frame.

  phi_prev_deg = *phi_deg_out;
  theta_prev_deg = *theta_deg_out;
  //psi_prev_degps = *psi_degps_out;

  gyro_filt_degps[0] = beta*(gyro_filt_degps[0]) + (1 - beta)*(gyro_meas_degps[0] - *phi_degps_bias);
  gyro_filt_degps[1] = beta*(gyro_filt_degps[1]) + (1 - beta)*(gyro_meas_degps[1] - *theta_degps_bias);
  gyro_filt_degps[2] = beta*(gyro_filt_degps[2]) + (1 - beta)*(gyro_meas_degps[2] - *psi_degps_bias);

  *phi_degps_out   = gyro_filt_degps[0];
  *theta_degps_out = gyro_filt_degps[1];
  *psi_degps_out   = gyro_filt_degps[2];

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

  //delta_omega_body_radps[0] = (1.5*gyro_filt_degps[0] - 0.5*gyro_prev_degps[0])*DEGTORAD;
  //delta_omega_body_radps[1] = (1.5*gyro_filt_degps[1] - 0.5*gyro_prev_degps[1])*DEGTORAD;
  //delta_omega_body_radps[2] = (1.5*gyro_filt_degps[2] - 0.5*gyro_prev_degps[2])*DEGTORAD;

  delta_omega_body_radps[0] = (gyro_filt_degps[0])*DEGTORAD;
  delta_omega_body_radps[1] = (gyro_filt_degps[1])*DEGTORAD;
  delta_omega_body_radps[2] = (gyro_filt_degps[2])*DEGTORAD;
  
  static float delta_omega_mag = 0.;
  normalize(delta_omega_body_radps, delta_omega_mag);

  static float temp_q_body[4] = {0., 0., 0., 0.};
  static float cosdo = 0.;
  static float sindo = 0.;
  cosdo = cos(delta_omega_mag*dt/2.);
  sindo = sin(delta_omega_mag*dt/2.);
  delta_q_omega[0] = cosdo;
  delta_q_omega[1] = sindo*delta_omega_body_radps[0];
  delta_q_omega[2] = sindo*delta_omega_body_radps[1];
  delta_q_omega[3] = sindo*delta_omega_body_radps[2];

  quatMult(q_body, delta_q_omega, temp_q_body);
  static float temp_delta_beta = 0.;
  static float temp_axis[3] = {0., 0., 0.};
  static float temp_scale = 0.;
  temp_scale = sqrt(1.0 - temp_q_body[0]*temp_q_body[0]);
  temp_delta_beta = acos(temp_q_body[0])*2.; // Limited range...
  if (temp_scale < 1e-8)
  {
    temp_axis[0] = 0.;
    temp_axis[1] = 0.;
    temp_axis[2] = 0.;
  }
  else
  {
    temp_axis[0] = temp_q_body[1]/temp_scale;
    temp_axis[1] = temp_q_body[2]/temp_scale;
    temp_axis[2] = temp_q_body[3]/temp_scale;
  }

  //Serial.print(temp_q_body[0]); Serial.print(" ");
  //Serial.print(temp_q_body[1]); Serial.print(" ");
  //Serial.print(temp_q_body[2]); Serial.print(" ");
  //Serial.print(temp_q_body[3]); Serial.print(" ");
  //Serial.println(temp_q_body[0]*temp_q_body[0] + temp_q_body[1]*temp_q_body[1] + temp_q_body[2]*temp_q_body[2] + temp_q_body[3]*temp_q_body[3]);
  //Serial.print("tempscale: "); Serial.println(temp_scale);
  //Serial.print("tempaxissize "); Serial.println(temp_axis[0]*temp_axis[0] + temp_axis[1]*temp_axis[1] + temp_axis[2]*temp_axis[2]);

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
  normalize(n, n_mag);

  if (n_mag > 1e-16)
  {
    static float delta_beta_f = 0.;
    static float axis_f[3] = {0., 0., 0.};
    //slerp(&accel_g2b_angle, n, &temp_delta_beta, temp_axis, alpha, &delta_beta_f, axis_f);
    static float q1[4] = {0., 0., 0., 0.};
    static float q2[4] = {0., 0., 0., 0.};
    static float sintemp = 0.;
    sintemp = sin(accel_g2b_angle/2.);
    q1[0] = cos(accel_g2b_angle/2.);
    q1[1] = sintemp*n[0];
    q1[2] = sintemp*n[1];
    q1[3] = sintemp*n[2];

    sintemp = sin(temp_delta_beta/2.);
    q2[0] = cos(temp_delta_beta/2.);
    q2[1] = sintemp*temp_axis[0];
    q2[2] = sintemp*temp_axis[1];
    q2[3] = sintemp*temp_axis[2];
    slerp(q1, q2, alpha, q_body);

    //static float cosdbf = 0.;
    //static float sindbf = 0.;
    //cosdbf = cos(delta_beta_f/2.);
    //sindbf = sin(delta_beta_f/2.);
    //q_body[0] = cosdbf;
    //q_body[1] = sindbf*axis_f[0];
    //q_body[2] = sindbf*axis_f[1];
    //q_body[3] = sindbf*axis_f[2];

    //Serial.print(q_body[0]); Serial.print(" ");
    //Serial.print(q_body[1]); Serial.print(" ");
    //Serial.print(q_body[2]); Serial.print(" ");
    //Serial.print(q_body[3]); Serial.println(" ");
  }
  else
  {
    q_body[0] = temp_q_body[0];
    q_body[1] = temp_q_body[1];
    q_body[2] = temp_q_body[2];
  }

  static float pose_quat_mag_squared = 0.;
  pose_quat_mag_squared = (
    q_body[0]*q_body[0] +
    q_body[1]*q_body[1] +
    q_body[2]*q_body[2] +
    q_body[3]*q_body[3]
  );

  if (pose_quat_mag_squared < 0.9801)
  {
    static float correction = 0.;
    correction = 0.1*(1.0 - pose_quat_mag_squared)*dt;

    q_body[0] += correction*q_body[0];
    q_body[1] += correction*q_body[1];
    q_body[2] += correction*q_body[2];
    q_body[3] += correction*q_body[3];
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