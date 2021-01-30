#ifndef _MB_REGS_H
#define _MB_REGS_H

//Input registers
#define mbIreg_ST_LED            0
#define mbIregUartErr            1

//Holdings registers
#define mbHreg_ST_LED            0
#define mbHreg_LidAngle          1      // Lid angle in [degrees], controls PPM1 & PPM2
#define mbHreg_LidSpeed          2      // lid speed in [degrees per second]. ~350 max for FS5109M
#define mbHreg_PPM3              3



#endif