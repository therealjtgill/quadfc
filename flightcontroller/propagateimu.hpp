#ifndef PROPAGATEIMU
#define PROPAGATEIMU

#include <math.h>
#include <stdint.h>

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

   sp = sin(pitch_in);
   cp = cos(pitch_in);
   //cp = sqrt(1 - sp*sp);
   sr = sin(roll_in);
   cr = cos(roll_in);
   //cr = sqrt(1 - sr*sr);

   omega_magnitude = sqrt(
      omega_body[0]*omega_body[0] +
      omega_body[1]*omega_body[1] +
      omega_body[2]*omega_body[2]
   );

   // The magnitude of omega is the angular speed of rotation. An approximate
   // delta-alpha is achieved by multiplying angular speed by dt.
   sa = sin(omega_magnitude*dt);
   ca = cos(omega_magnitude*dt);
   
   omega_global_unit[0] = (omega_body[0]*cp + omega_body[2]*sp)/omega_magnitude;
   omega_global_unit[1] = (omega_body[0]*sp*sr + omega_body[1]*cr - omega_body[2]*sr*cp)/omega_magnitude;
   omega_global_unit[2] = (-1.*omega_body[0]*sp*cr + omega_body[1]*sr + omega_body[2]*cp*cr)/omega_magnitude;
   
   rod2[0] = -1.*sa*omega_global_unit[1]  + (1. - ca)*(omega_global_unit[0]*omega_global_unit[2]);
   rod2[1] =     sa*omega_global_unit[0]  + (1. - ca)*(omega_global_unit[1]*omega_global_unit[2]);
   rod2[2] =                           1. + (1. - ca)*(omega_global_unit[2]*omega_global_unit[2] - 1.);

   tanpitch_n = rod2[2]*cr*sp - rod2[0]*cp - rod2[1]*sr*sp;
   tanpitch_d = rod2[0]*sp - rod2[1]*sr*cp + rod2[2]*cp*cr;
   sinroll    = rod2[1]*cr + rod2[2]*sr;

   pitch_out = atan2(tanpitch_n, tanpitch_d);
   roll_out = asin(sinroll);
}

#endif
