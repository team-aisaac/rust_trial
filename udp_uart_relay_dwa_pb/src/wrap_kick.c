#define _USE_MATH_DEFINES
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "robot_controller/tools.h"
#include "robot_controller/ball_wrap_kick_include/wrap_kick.h"
//ros2デバッグ用のヘッダファイル
#include "/opt/ros/foxy/include/rcutils/logging_macros.h"
#include "/opt/ros/foxy/include/rcutils/logging.h"

/*
void wrap_kick(int32_t w_robot_x, int32_t w_robot_y, int32_t w_robot_theta, int32_t w_robot_omega, int32_t w_ball_x, int32_t w_ball_y, int32_t w_ball_vx, int32_t w_ball_vy, 
    int32_t target_ball_x, int32_t target_ball_y, int32_t r_ball_x, int32_t r_ball_y, int32_t* robot_goal_x, int32_t* robot_goal_y, int32_t* robot_goal_theta, int32_t *ball_kick_cin_max_velo_theta, 
    bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick){
    //RCUTILS_LOG_INFO("wrap ball kick");
    float w_robot_rad = angle_range_corrector(w_robot_theta/1000 * DEG_TO_RAD); //rad
    float w_robot_rad_omega = -w_robot_omega/1000 * DEG_TO_RAD;  //rad/s
    float x_omega_offset = ROBOT_KICK_MIN_X * (1 - cosf(RASPI_TIME_STEP*w_robot_rad_omega));    
    float y_omega_offset = ROBOT_KICK_MIN_X * sinf(RASPI_TIME_STEP*w_robot_rad_omega);
    float r_goal_x = r_ball_x - ROBOT_KICK_MIN_X; //現在のロボット座標系でのロボットの目標値のx座標
    if(!wrap_con_ball_touch_check(w_robot_theta, w_ball_x, w_ball_y, target_ball_x, target_ball_y, free_kick_flag, ball_target_allowable_error, ball_kick)){
        r_goal_x = r_ball_x - ROBOT_NOT_TOUCH_MIN_X;
    }
    float r_goal_y = r_ball_y - ROBOT_WRAP_KICK_MAGIC_NUMBER*w_robot_omega/(*ball_kick_cin_max_velo_theta); //現在のロボット座標系でのロボットの目標値のy座標
    float w_goal_x, w_goal_y;   //world座標系でのロボットの目標値
    r_to_w_coordinate_chang(&w_goal_x, &w_goal_y, r_goal_x + x_omega_offset, r_goal_y + y_omega_offset, (float) w_robot_x, (float) w_robot_y, w_robot_rad);
    *robot_goal_x = w_goal_x + RASPI_TIME_STEP*w_ball_vx;
    *robot_goal_y = w_goal_y + RASPI_TIME_STEP*w_ball_vy;
    //ロボットの目標角度の計算
    if(target_ball_x - w_ball_x !=0 || target_ball_y - w_ball_y !=0){
        float w_goal_rad = atan2f(target_ball_y - w_ball_y, target_ball_x - w_ball_x);
        *robot_goal_theta = 1000*w_goal_rad*RAD_TO_DEG;
    }
    if(!free_kick_flag){
        *ball_kick_cin_max_velo_theta = 140*1000;
    }
}*/

void wrap_kick2(wrap_kick_robot robot, wrap_kick_ball ball, wrap_motion_pid *pid, bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick, bool *ball_kick_con, 
    bool *dribble_active){
    float w_robot_rad = angle_range_corrector(robot.w_robot_theta/1000 * DEG_TO_RAD); //rad
    float ball_goal_ball_distance = norm((float)ball.w_ball_x, (float)ball.w_ball_y, (float)ball.target_ball_x, (float)ball.target_ball_y);
    float robot_ball_distance = norm((float)ball.r_ball_x, (float)ball.r_ball_y, 0, 0);
    if(robot_ball_distance == 0 || ball_goal_ball_distance == 0){
        return;
    }
    float robot_ball_con_distance = ROBOT_KICK_MIN_X;
    if(!wrap_con_ball_touch_check(robot, ball, free_kick_flag, ball_target_allowable_error, ball_kick)){
        robot_ball_con_distance = ROBOT_NOT_TOUCH_MIN_X;
    }
    //ロボットがボールを蹴る際の最終目標地点
    float robot_kick_goal_x = ball.w_ball_x + robot_ball_con_distance*(ball.w_ball_x - ball.target_ball_x)/ball_goal_ball_distance;
    float robot_kick_goal_y = ball.w_ball_y + robot_ball_con_distance*(ball.w_ball_y - ball.target_ball_y)/ball_goal_ball_distance;
    float robot_robot_target_distance = norm(robot_kick_goal_x, robot_kick_goal_y, robot.w_robot_x, robot.w_robot_y);
    if(robot_robot_target_distance <= 50){
        pid->robot_xy_con_flag = true;
        pid->goal_x = robot_kick_goal_x;
        pid->goal_y = robot_kick_goal_y;
        if(ball.target_ball_x - ball.w_ball_x !=0 || ball.target_ball_y - ball.w_ball_y !=0){
            float w_goal_rad = atan2f(ball.target_ball_y - ball.w_ball_y, ball.target_ball_x - ball.w_ball_x);
            pid->robot_goal_theta = 1000*w_goal_rad*RAD_TO_DEG;
        }
    }
    else{
        pid->robot_xy_con_flag = false;
        float different_rad 
            = modifid_acosf(((robot_kick_goal_x-ball.w_ball_x)*(robot.w_robot_x-ball.w_ball_x) + (robot_kick_goal_y-ball.w_ball_y)*(robot.w_robot_y-ball.w_ball_y))/(robot_ball_distance*robot_ball_con_distance));
        pid->cul_circumferential_error = robot_ball_con_distance*different_rad;
        pid->cul_radius_error = robot_ball_con_distance - robot_ball_distance;
        float unit_vec_radius_x = (robot.w_robot_x - ball.w_ball_x)/robot_ball_distance;
        float unit_vec_radius_y = (robot.w_robot_y - ball.w_ball_y)/robot_ball_distance;
        pid->ob_unit_vec_radius_x = (robot.w_robot_x - ball.w_ball_x)/robot_ball_distance;
        pid->ob_unit_vec_radius_y = (robot.w_robot_y - ball.w_ball_y)/robot_ball_distance;
        float rotate_flag = (robot_kick_goal_y - robot.w_robot_y)*(ball.w_ball_x - robot.w_robot_x) - (robot_kick_goal_x - robot.w_robot_x)*(ball.w_ball_y - robot.w_robot_y);
        float robot_wrap_omega = 0;
        float robot_wrap_d_theta, unit_vec_circumferential_x, unit_vec_circumferential_y;
        float robot_v = norm(robot.w_robot_vx, robot.w_robot_vy, 0, 0);
        if(0<rotate_flag){
            if(-ROBOT_WRAP_DRIBBLE_NON_ACTIVE < ball.r_ball_y){
                *dribble_active = false;
            }
            unit_vec_circumferential_x = pid->ob_unit_vec_radius_y;
            unit_vec_circumferential_y = -pid->ob_unit_vec_radius_x;
            if(robot_v!=0){
                robot_wrap_omega = (robot.w_robot_vx*unit_vec_circumferential_x + robot.w_robot_vy*unit_vec_circumferential_y)/robot_ball_con_distance;
                robot_wrap_d_theta = angle_range_corrector(RASPI_TIME_STEP*robot_wrap_omega);
                if(0 < robot_wrap_omega){
                    robot_wrap_omega = 0;
                }
            }
        }
        else{
            if(ball.r_ball_y < ROBOT_WRAP_DRIBBLE_NON_ACTIVE){
                *dribble_active = false;
            }
            unit_vec_circumferential_x = -pid->ob_unit_vec_radius_y;
            unit_vec_circumferential_y = pid->ob_unit_vec_radius_x;
            if(robot_v!=0){
                robot_wrap_omega = (robot.w_robot_vx*unit_vec_circumferential_x + robot.w_robot_vy*unit_vec_circumferential_y)/robot_ball_con_distance;
                robot_wrap_d_theta = angle_range_corrector(RASPI_TIME_STEP*robot_wrap_omega);
                if(robot_wrap_omega < 0){
                    robot_wrap_omega = 0;
                }
            }
        }
        pid->ob_unit_vec_circumferential_x = unit_vec_circumferential_x*cosf(robot_wrap_d_theta) - unit_vec_circumferential_y*sinf(robot_wrap_d_theta);
        pid->ob_unit_vec_circumferential_y = unit_vec_circumferential_x*sinf(robot_wrap_d_theta) + unit_vec_circumferential_y*cosf(robot_wrap_d_theta);
        pid->ob_unit_vec_radius_x = unit_vec_radius_x*cosf(robot_wrap_d_theta) - unit_vec_radius_y*sinf(robot_wrap_d_theta);
        pid->ob_unit_vec_radius_y = unit_vec_radius_x*sinf(robot_wrap_d_theta) + unit_vec_radius_y*cosf(robot_wrap_d_theta);
        pid->robot_goal_theta = robot.w_robot_theta + 1000*atan2f(ball.r_ball_y,ball.r_ball_x)*RAD_TO_DEG + 10*1000*robot_wrap_omega*RASPI_TIME_STEP*RAD_TO_DEG;
    }
    //ボールを蹴る判定
    if(ball_kick){
        *ball_kick_con = ball_kick_check(robot, ball, ball_target_allowable_error);
    }
    //RCUTILS_LOG_INFO("rotate x : %lf, rotate y : %lf", pid->ob_unit_vec_circumferential_x, pid->ob_unit_vec_circumferential_y);
}

bool wrap_con_ball_touch_check(wrap_kick_robot robot, wrap_kick_ball ball, bool free_kick_flag, int32_t ball_target_allowable_error, bool ball_kick){
    if(!free_kick_flag){
        return true;
    }
    else{
        if(!ball_kick){//フリーキックの指定でボールを蹴らないときはフェイント
            return false;
        }
        else if(1000 <= ball_target_allowable_error){ //ボールをクリアする
            return true;
        }

        float ball_ball_target_distance = norm((float)ball.target_ball_x, (float)ball.target_ball_y, (float)ball.w_ball_x, (float)ball.w_ball_y);
        if(ball_ball_target_distance == 0){
            return false;
        }
        float threshold_ball_kick_rad = modifid_asinf(ball_target_allowable_error/ball_ball_target_distance);   //ロボットがボールを蹴る際の角度の許容誤差(rad)
        float robot_rad = angle_range_corrector((float)robot.w_robot_theta / 1000 * DEG_TO_RAD);             //ロボットの角度(rad)
        //ボールからボール目標値へのベクトルとロボットが向いている向きのベクトルが成す角(rad)
        float ball_ball_target_vectot_robot_vctor_rad 
            = modifid_acosf(((ball.target_ball_x - ball.w_ball_x)*cosf(robot_rad) + (ball.target_ball_y - ball.w_ball_y)*sinf(robot_rad))/ball_ball_target_distance);
        if(ball_ball_target_vectot_robot_vctor_rad < threshold_ball_kick_rad){
            return true;
        }
    }
    return false;
}

bool ball_kick_check(wrap_kick_robot robot, wrap_kick_ball ball, int32_t ball_target_allowable_erroe){
        //RCUTILS_LOG_INFO("ball kick ckeck");
        //ボールが蹴れるx,y座標（ロボット座標系）を判定
        if((0 < ball.r_ball_x && ball.r_ball_x < ROBOT_KICK_ENABLE_X + BALL_DIAMETER/2) && 
            (-ROBOT_KICK_ENABLE_Y < ball.r_ball_y && ball.r_ball_y < ROBOT_KICK_ENABLE_Y)){
                //RCUTILS_LOG_INFO("ball kick position");
                //ball_target_allowable_erroe -> ボールを蹴るターゲットからの許容誤差の半径 (mm)
                //例えば精密シュートをするのか大雑把なパスをするのか、クリア(1000<ball_target_allowable_erro)をするのか
                //ball_target_allowable_erroe = BALL_TARGET_ALLOWABLE_ERROR;
                if(1000 <= ball_target_allowable_erroe){ //ボールをクリアする
                    //RCUTILS_LOG_INFO("clear");
                    return true;
                }
                float ball_ball_target_distance = norm((float)ball.target_ball_x, (float)ball.target_ball_y, (float)ball.w_ball_x, (float)ball.w_ball_y);
                if(ball_ball_target_distance == 0){
                    return false;
                }
                float threshold_ball_kick_rad = modifid_asinf(ball_target_allowable_erroe/ball_ball_target_distance);   //ロボットがボールを蹴る際の角度の許容誤差(rad)
                float robot_rad = angle_range_corrector((float) robot.w_robot_theta / 1000 * DEG_TO_RAD);             //ロボットの角度(rad)
                //ボールからボール目標値へのベクトルとロボットが向いている向きのベクトルが成す角(rad)
                float ball_ball_target_vectot_robot_vctor_rad 
                    = modifid_acosf(((ball.target_ball_x - ball.w_ball_x)*cosf(robot_rad) + (ball.target_ball_y - ball.w_ball_y)*sinf(robot_rad))/ball_ball_target_distance);
                //RCUTILS_LOG_INFO("threshold : %f", threshold_ball_kick_rad*RAD_TO_DEG);
                RCUTILS_LOG_INFO("diff angle : %f", ball_ball_target_vectot_robot_vctor_rad*RAD_TO_DEG);
                if(ball_ball_target_vectot_robot_vctor_rad < threshold_ball_kick_rad){
                    //RCUTILS_LOG_INFO("ball kick");
                    return true;
                }
            }
        return false;
}