#ifndef __FIR_H__
#define __FIR_H__
#include <defs.h>

#define N 11
#define data_length 64

int tap[N] = {0,-10,-9,23,56,63,56,23,-9,-10,0};
int inputbuffer[N];
int inputsignal[N] = {1,2,3,4,5,6,7,8,9,10,11};
int outputsignal[N];
int y[data_length];

#endif
