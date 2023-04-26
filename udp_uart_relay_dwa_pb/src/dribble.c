#define _USE_MATH_DEFINES
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "robot_controller/tools.h"
#include "robot_controller/dribble_include/dribble.h"
#include "robot_controller/ball_wrap_kick_include/wrap_kick.h"
//#include "robot_controller/ball_wrap_kick_include/wrap_kick.h"
//ros2デバッグ用のヘッダファイル
#include "/opt/ros/foxy/include/rcutils/logging_macros.h"
#include "/opt/ros/foxy/include/rcutils/logging.h"

bool dribble_wrap_motion(wrap_kick_robot robot, wrap_kick_ball ball, wrap_motion_pid *pid, int32_t ball_target_allowable_error, bool dribble_ball_move_flag, bool *dribble_active){
    float w_robot_rad = angle_range_corrector(robot.w_robot_theta/1000 * DEG_TO_RAD); //rad
    float ball_goal_ball_distance = norm((float)ball.w_ball_x, (float)ball.w_ball_y, (float)ball.target_ball_x, (float)ball.target_ball_y);
    float robot_ball_distance = norm((float)ball.r_ball_x, (float)ball.r_ball_y, 0, 0);
    if(robot_ball_distance == 0 || ball_goal_ball_distance == 0){
        return;
    }
    float robot_ball_con_distance = ROBOT_KICK_MIN_X;
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

    //ロボットの目標角度の計算
    if(ball.target_ball_x - ball.w_ball_x !=0 || ball.target_ball_y - ball.w_ball_y !=0){
        float w_goal_rad = atan2f(ball.target_ball_y - ball.w_ball_y, ball.target_ball_x - ball.w_ball_x);
        //*robot_goal_theta = 1000*w_goal_rad*RAD_TO_DEG;
        int32_t diff_angle_deg = (1000*w_goal_rad*RAD_TO_DEG - robot.w_robot_theta)/1000;
        if(diff_angle_deg != 0){
            if(fabs(diff_angle_deg) < ROBOT_DRIBBLE_START_DEG){
                return true;
            }
            else if(dribble_ball_move_flag/* && fabs(diff_angle_deg) < 45*/){
                return true;
            }
            return false;
        }
        else{
            return true;
        }
    }
    return true;
}

int32_t dribble_move_deg(int32_t r_ball_y, int32_t robot_goal_theta){
    int32_t robot_move_deg = r_ball_y*ROBOT_DRIBBLE_ABJUST_DEG;
    if(ROBOT_DRIBBLE_START_DEG/2 < robot_move_deg){
        robot_move_deg = ROBOT_DRIBBLE_START_DEG/2;
    }
    else if(robot_move_deg < -ROBOT_DRIBBLE_START_DEG/2){
        robot_move_deg = -ROBOT_DRIBBLE_START_DEG/2;
    }
    //RCUTILS_LOG_INFO("abjust deg : %d", robot_move_deg);
    return robot_goal_theta - robot_move_deg*1000;
}

