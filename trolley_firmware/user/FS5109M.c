#include "FS5109M.h"

unsigned int servo_angleToPPM(float angle_deg)
{
  if (angle_deg > max_angle)
    angle_deg = max_angle;
  
  return (unsigned int)(ppm_min + angle_deg / deg_per_ppm);
}

float servo_ppmToAngle(unsigned int ppm)
{
  if (ppm > ppm_max)
    ppm = ppm_max;
  
  return (ppm - ppm_min) * deg_per_ppm;
}