#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include "DWA_path_recover.h"
#include "tools.h"

void DWA_path_recover(float *x, float *y, float *vx, float *vy, float accel_x, float accel_y, float max_velo){
    *vx = *vx + accel_x*MICON_TIME_STEP;
    *vy = *vy + accel_y*MICON_TIME_STEP;
    float v = sqrt(*vx * *vx + *vy * *vy);
    if(v > max_velo){
        *vx = *vx/v*max_velo;
        *vy = *vy/v*max_velo;
    }
    *x = *x + *vx*MICON_TIME_STEP;
    *y = *y + *vy*MICON_TIME_STEP;
}