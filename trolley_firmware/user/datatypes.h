#ifndef _DATATYPES_H
#define _DATATYPES_H

// TODO: save lid_params to nonvilatile memory to restore correct lid angle after power fails !
typedef struct
{
  float target_angle;
  float current_angle;
  unsigned int speed;
} LidParams;

#endif