#ifndef _WRAP_KICK_H_
#define _WRAP_KICK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "tools.h"

typedef struct{         //mm or mm/s
    //ボールの情報
    int32_t w_ball_x;
    int32_t w_ball_y;
    int32_t w_ball_vx;
    int32_t w_ball_vy; 
    int32_t target_ball_x;
    int32_t target_ball_y;
    int32_t r_ball_x;
    int32_t r_ball_y;
} wrap_kick_ball;

typedef struct{         //mm or 0.001 deg or 0.001 deg/s
    //ロボットの情報
    int32_t w_robot_x;
    int32_t w_robot_y;
    int32_t w_robot_vx;
    int32_t w_robot_vy;
    int32_t w_robot_theta;
    int32_t w_robot_omega;
} wrap_kick_robot;

typedef struct{       //単位なし or 0.001 deg or mm or 0.001 deg/s
    //回り込み動作を行うためのpid制御を行うために必要な情報
    float ob_unit_vec_circumferential_x;
    float ob_unit_vec_circumferential_y;
    float ob_unit_vec_radius_x;
    float ob_unit_vec_radius_y; 
    int32_t robot_goal_theta;
    int32_t cul_circumferential_error;
    int32_t cul_radius_error;
    int32_t ball_kick_cin_max_velo_theta;
    int32_t goal_x;
    int32_t goal_y;
    bool robot_xy_con_flag;         //目標点と近い場合true
} wrap_motion_pid;

//ボールを蹴るための回り込み動作を行うための目標値を決定する関数
/*
void wrap_kick(int32_t w_robot_x, int32_t w_robot_y, int32_t w_robot_theta, int32_t w_robot_omega, int32_t w_ball_x, int32_t w_ball_y, int32_t w_ball_vx, int32_t w_ball_vy, 
    int32_t target_ball_x, int32_t target_ball_y, int32_t r_ball_x, int32_t r_ball_y, int32_t* robot_goal_x, int32_t* robot_goal_y, int32_t* robot_goal_theta, int32_t *ball_kick_cin_max_velo_theta, 
    bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick);
*/
void wrap_kick2(wrap_kick_robot robot, wrap_kick_ball ball, wrap_motion_pid *pid, bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick, bool *ball_kick_con, 
    bool *dribble_active);
//ボールに対する回り込み動作時にボールに触れてよいかを判定する(触れる場合はドリブラーによって保持することを想定する)
bool wrap_con_ball_touch_check(wrap_kick_robot robot, wrap_kick_ball ball, bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick);
//ボールを蹴る判定を行う関数
bool ball_kick_check(wrap_kick_robot robot, wrap_kick_ball ball, int32_t ball_target_allowable_erroe);
#ifdef __cplusplus
}
#endif
#endif // _DWA_H_