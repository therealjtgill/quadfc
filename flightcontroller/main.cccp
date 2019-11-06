#include "propagateimu.hpp"
#include <iostream>

int main(void)
{
   float dt = 1./250.;
   float omega_body[3] = {0., 0., M_PI};
   float pitch_prev = M_PI/6;
   float roll_prev = M_PI/4;
   float pitch_curr = 0;
   float roll_curr = 0;
   for (unsigned int i = 0; i < 500; ++i)
   {
      propagatePitchRoll(pitch_prev, roll_prev, omega_body, dt, pitch_curr, roll_curr);
      std::cout << pitch_curr*180./M_PI << " " << roll_curr*180./M_PI << std::endl;
      pitch_prev = pitch_curr;
      roll_prev = roll_curr;
   }
   
   return 0;
}