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

float ApproxAtan2(float y, float x)
{
    const float n1 = 0.97239411f;
    const float n2 = -0.19194795f; 
    const float PI_2 = M_PI/2.;
    //const float PI = M_PI;   
    float result = 0.0f;
    if (x != 0.0f)
    {
        const union { float flVal; uint32_t nVal; } tYSign = { y };
        const union { float flVal; uint32_t nVal; } tXSign = { x };
        if (fabsf(x) >= fabsf(y))
        {
            union { float flVal; uint32_t nVal; } tOffset = { PI };
            // Add or subtract PI based on y's sign.
            tOffset.nVal |= tYSign.nVal & 0x80000000u;
            // No offset if x is positive, so multiply by 0 or based on x's sign.
            tOffset.nVal *= tXSign.nVal >> 31;
            result = tOffset.flVal;
            const float z = y / x;
            result += (n1 + n2 * z * z) * z;
        }
        else // Use atan(y/x) = pi/2 - atan(x/y) if |y/x| > 1.
        {
            union { float flVal; uint32_t nVal; } tOffset = { PI_2 };
            // Add or subtract PI/2 based on y's sign.
            tOffset.nVal |= tYSign.nVal & 0x80000000u;            
            result = tOffset.flVal;
            const float z = x / y;
            result -= (n1 + n2 * z * z) * z;            
        }
    }
    else if (y > 0.0f)
    {
        result = PI_2;
    }
    else if (y < 0.0f)
    {
        result = -PI_2;
    }
    return result;
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
  static const float DEGTORAD = M_PI/180.;
  static const float RADTODEG = 180./M_PI;

  static float gyro_filt_degps[3] = {0., 0., 0.};
  static float gyro_prev_degps[3] = {0., 0., 0.};
  static float phi_acc          = 0.;
  static float theta_acc        = 0.;
  static float theta_gyro_rad   = 0.;
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

  static float delta_omega_arr[3] = {0., 0., 0.};
  static float pose_global_arr[4] = {1., 0., 0., 0.};
  static float v0, v1, v2, v3;
  static float s0, s1, s2, s3;

  static float delta_omega_mag = 0.;
  static float delta_omega_mag_squared = 0.;
  static float cosdam = 0.;
  static float sindam = 0.;
  static float sindam_over_dam = 0.;

  static float pose_quat_mag_squared = 0.;
  static float correction = 0.;
  
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

  delta_omega_arr[0] = 1.5*gyro_filt_degps[0] - 0.5*gyro_prev_degps[0];
  delta_omega_arr[1] = 1.5*gyro_filt_degps[1] - 0.5*gyro_prev_degps[1];
  delta_omega_arr[2] = 1.5*gyro_filt_degps[2] - 0.5*gyro_prev_degps[2];
  
  //delta_omega_mag = sqrt(
  //  delta_omega_arr[0]*delta_omega_arr[0] + 
  //  delta_omega_arr[1]*delta_omega_arr[1] + 
  //  delta_omega_arr[2]*delta_omega_arr[2]
  //)*DEGTORAD*0.5*dt;

  delta_omega_mag_squared = (
    delta_omega_arr[0]*delta_omega_arr[0] + 
    delta_omega_arr[1]*delta_omega_arr[1] + 
    delta_omega_arr[2]*delta_omega_arr[2]
  )*DEGTORAD*DEGTORAD*0.25*dt*dt;
  //1 300us

  //1.5
  //cosdam = cos(delta_omega_mag);
  //sindam = sin(delta_omega_mag);
  
  //1.5 200us

  //2
  //v0 = cosdam;
  //v1 = sindam*delta_omega_arr[0]*0.5*dt*DEGTORAD/delta_omega_mag;
  //v2 = sindam*delta_omega_arr[1]*0.5*dt*DEGTORAD/delta_omega_mag;
  //v3 = sindam*delta_omega_arr[2]*0.5*dt*DEGTORAD/delta_omega_mag;
  //2 200us; removing division decreases this to 120us
  
  cosdam = 1.0 - delta_omega_mag_squared/2. + delta_omega_mag_squared*delta_omega_mag_squared/24;
  sindam_over_dam = 1.0 - delta_omega_mag_squared/6. + delta_omega_mag_squared*delta_omega_mag_squared/120.;
  v0 = cosdam;
  v1 = sindam_over_dam*0.5*dt*gyro_filt_degps[0]*DEGTORAD;
  v2 = sindam_over_dam*0.5*dt*gyro_filt_degps[1]*DEGTORAD;
  v3 = sindam_over_dam*0.5*dt*gyro_filt_degps[2]*DEGTORAD;
  
  
  //3
  s0 = pose_global_arr[0];
  s1 = pose_global_arr[1];
  s2 = pose_global_arr[2];
  s3 = pose_global_arr[3];
  
  pose_global_arr[0] = s0*v0 - (s1*v1 + s2*v2 + s3*v3);
  pose_global_arr[1] = s0*v1 + v0*s1 + s2*v3 - s3*v2;
  pose_global_arr[2] = s0*v2 + v0*s2 + s3*v1 - s1*v3;
  pose_global_arr[3] = s0*v3 + v0*s3 + s1*v2 - s2*v1;
  //3 272us
  
  //4 
  pose_quat_mag_squared = 
    pose_global_arr[0]*pose_global_arr[0] +
    pose_global_arr[1]*pose_global_arr[1] +
    pose_global_arr[2]*pose_global_arr[2] +
    pose_global_arr[3]*pose_global_arr[3];
  //4 64us
  
  if (pose_quat_mag_squared < 0.9801)
  {
    correction = 0.1*(1.0 - pose_quat_mag_squared)*dt;

    pose_global_arr[0] += correction*pose_global_arr[0];
    pose_global_arr[1] += correction*pose_global_arr[1];
    pose_global_arr[2] += correction*pose_global_arr[2];
    pose_global_arr[3] += correction*pose_global_arr[3];
  }

  if (only_accel)
  {
      // Calculate phi and theta according to accelerometer output.
    phi_acc = atan2(acc_meas[1], acc_meas[2])*RADTODEG + 180;
    if (phi_acc > 180.)
    {
      phi_acc -= 360;
    }
    
    theta_acc = atan2(1.*acc_meas[0], sqrt(acc_meas[1]*acc_meas[1] + acc_meas[2]*acc_meas[2]))*RADTODEG;

    *phi_deg_out = phi_acc;
    *theta_deg_out = theta_acc;

    gyro_prev_degps[0] = gyro_filt_degps[0];
    gyro_prev_degps[1] = gyro_filt_degps[1];
    gyro_prev_degps[2] = gyro_filt_degps[2];

    float costheta = cos(theta_acc*M_PI/360.);
    float sintheta = sin(theta_acc*M_PI/360.);

    float cosphi = cos(phi_acc*M_PI/360.);
    float sinphi = sin(phi_acc*M_PI/360.);

    pose_global_arr[0] =  costheta*cosphi;
    pose_global_arr[1] =  costheta*sinphi;
    pose_global_arr[2] =  sintheta*cosphi;
    pose_global_arr[3] =  sintheta*sinphi;

    return;
  }

  //5
  phi_gyro_temp_deg = atan2(
    2.*(pose_global_arr[2]*pose_global_arr[3] + pose_global_arr[1]*pose_global_arr[0]),
    1 - 2.*(pose_global_arr[1]*pose_global_arr[1] + pose_global_arr[2]*pose_global_arr[2])
  )*RADTODEG - dt*-0.0104633889;
  //5 300us

  //6
  theta_gyro_temp_deg = asin(
    2.*(pose_global_arr[0]*pose_global_arr[2] - pose_global_arr[1]*pose_global_arr[3])
  )*RADTODEG - dt*0.0203988087;
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