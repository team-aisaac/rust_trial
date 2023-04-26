#ifndef _DWA_PATH_RECOVER_H_
#define _DWA_PATH_RECOVER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    //マイコンからの情報
    int32_t x;
    int32_t y;
    int32_t theta;
    int32_t Vx;
    int32_t Vy;
    int32_t omega;
    //戦略PCからの情報
    int32_t targetX;
    int32_t targetY;
    int32_t targetTheta;
    int32_t midle_targetX;
    int32_t midle_targetY;
} dwa_robot_path;

void DWA_path_recover(float *x, float *y, float *vx, float *vy, float accel_x, float accel_y, float max_velo);
#ifdef __cplusplus
}
#endif
#endif // _DWA_PATH_RECOVER_H_