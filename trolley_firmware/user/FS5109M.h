#ifndef _FS5109M_H
#define _FS5109M_H

static const unsigned int ppm_min = 50;
static const unsigned int ppm_max = 250;
static const unsigned int max_angle = 180;
static const float deg_per_ppm = 0.9;

// Angles in degrees, [0..180]
unsigned int servo_angleToPPM(float angle_deg);
float servo_ppmToAngle(unsigned int ppm);

#endif