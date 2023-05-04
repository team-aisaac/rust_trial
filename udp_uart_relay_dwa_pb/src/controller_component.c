// #include <vector>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "wrap_kick.h"
#include "tools.h"
#include "dribble.h"
#include "dwa2.h"

#include "controller_component.h"

void decide_next_goal_xy(State goal_pose, State *midle_goal_pose, State *next_goal_pose, bool prohidited_zone_ignore, bool *midle_target_flag, 
    const unsigned int robot_id, TrackedRobot my_robot, bool team_is_yellow_, bool *trape_controle_flag, micon_trape_con *trape_c)
{
  dwa_robot.targetX = goal_pose.x*1000;   //単位変換(m ->mm)
  dwa_robot.targetY = goal_pose.y*1000;   //単位変換(m ->mm)
  dwa_robot.targetTheta = goal_pose.theta;
  dwa_robot.midle_targetX = midle_goal_pose->x*1000; //単位変換(m ->mm)
  dwa_robot.midle_targetY = midle_goal_pose->y*1000; //単位変換(m ->mm)
  float robot_virtual_goal_distance = hypot(dwa_robot.x - my_robot.pos.x*1000, dwa_robot.y - my_robot.pos.y*1000);
  
  if( ROBOT_POSITION_RESET_DISTANCE < robot_virtual_goal_distance ){
    dwa_robot.x = my_robot.pos.x*1000;
    dwa_robot.y = my_robot.pos.y*1000;
    dwa_robot.theta = my_robot.orientation;
    dwa_robot.Vx = my_robot.vel[0].x*1000;
    dwa_robot.Vy = my_robot.vel[0].y*1000;
    dwa_robot.omega = my_robot.vel_angular[0];
  }
  
  int32_t numOfObstacle = 0;
  int32_t obs_x[2*ROBOT_NUM], obs_y[2*ROBOT_NUM], obs_vx[2*ROBOT_NUM], obs_vy[2*ROBOT_NUM], obs_ax[2*ROBOT_NUM], obs_ay[2*ROBOT_NUM];
  // TrackedRobot obs_robot;
  // int32_t k = 0;

  // 障害物の情報を取得
  // 自機以外の最大15台のロボットの座標と速度を取得する
  // for(uint32_t j = 0; j < ROBOT_NUM; j++){
  //     if (Controller::parser_.extract_robot(j, team_is_yellow_, obs_robot) && j != robot_id){
  //     numOfObstacle++;
  //     obs_x[k] = obs_robot.pos.x*1000;       //単位変換(m ->mm)
  //     obs_y[k] = obs_robot.pos.y*1000;       //単位変換(m ->mm)
  //     obs_vx[k] = obs_robot.vel[0].x*1000;   //単位変換(m/s ->mm/s)
  //     obs_vy[k] = obs_robot.vel[0].y*1000;   //単位変換(m/s ->mm/s)
  //     obs_ax[k] = 0*1000;                       //単位変換(m/s^2 ->mm/s^2)
  //     obs_ay[k] = 0*1000;                       //単位変換(m/s^2 ->mm/s^2)
  //     k++;
  //     }
  //     if (Controller::parser_.extract_robot(j, !team_is_yellow_, obs_robot)){
  //     numOfObstacle++;
  //     obs_x[k] = obs_robot.pos.x*1000;       //単位変換(m ->mm)
  //     obs_y[k] = obs_robot.pos.y*1000;       //単位変換(m ->mm)
  //     obs_vx[k] = obs_robot.vel[0].x*1000;   //単位変換(m/s ->mm/s)
  //     obs_vy[k] = obs_robot.vel[0].y*1000;   //単位変換(m/s ->mm/s)
  //     obs_ax[k] = 0*1000;                       //単位変換(m/s^2 ->mm/s^2)
  //     obs_ay[k] = 0*1000;                       //単位変換(m/s^2 ->mm/s^2)
  //     k++;
  //     }
  // }

  // DWAの出力のための変数
  bool is_enable;
  bool path_enable;
  int32_t output_vx;
  int32_t output_vy;
  int32_t output_omega;
  int32_t output_ax;
  int32_t output_ay;
  execDWA(dwa_robot.x, dwa_robot.y, dwa_robot.theta, dwa_robot.Vx, dwa_robot.Vy, dwa_robot.omega, 
        &dwa_robot.targetX, &dwa_robot.targetY, dwa_robot.targetTheta, &dwa_robot.midle_targetX, &dwa_robot.midle_targetY,
        numOfObstacle, obs_x, obs_y, obs_vx, obs_vy, obs_ax, obs_ay, prohidited_zone_ignore, 
        midle_target_flag, &is_enable, &path_enable, &output_vx, &output_vy, &output_omega, &output_ax, &output_ay);
  float next_goal_pose_x = dwa_robot.x;
  float next_goal_pose_y = dwa_robot.y;
  float robot_vertual_vel_x = dwa_robot.Vx;
  float robot_vertual_vel_y = dwa_robot.Vy;
  if(is_enable == 1 && path_enable == 1){
    //DWAを行う
    for(uint16_t i = 0; i < RASPI_TIME_STEP/MICON_TIME_STEP; i++){
      DWA_path_recover(&next_goal_pose_x, &next_goal_pose_y, &robot_vertual_vel_x, &robot_vertual_vel_y, 
                        (float) output_ax, (float) output_ay, (float) ROBOT_MAX_VEL);
      float robot_dwa_goal_distance = hypot(next_goal_pose_x - my_robot.pos.x*1000, next_goal_pose_y - my_robot.pos.y*1000);
      if(DWA_ROBOTXY_VIRUALXY_DISTANCE_CHECK < robot_dwa_goal_distance){
        //RCLCPP_INFO(rclcpp::get_logger("dwa"),"dwa_overdwa_overdwa_overdwa_overdwa_overdwa_overdwa_overdwa_over");
        break;
      }
    }
    dwa_robot.x = next_goal_pose_x;
    dwa_robot.y = next_goal_pose_y;
    dwa_robot.theta = my_robot.orientation;
    dwa_robot.Vx = robot_vertual_vel_x;
    dwa_robot.Vy = robot_vertual_vel_y;
    dwa_robot.omega = my_robot.vel_angular[0];
    next_goal_pose->x = next_goal_pose_x;
    next_goal_pose->y = next_goal_pose_y;
    trape_controle_flag[robot_id] = 0;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"dwa");
  }
  else if(is_enable == 1 && path_enable == 0){
    //DWAから台形制御に移行する際に変数を設定する関数
    trape_controle_flag[robot_id] = micon_trapezoidal_DWA_change(dwa_robot.x, dwa_robot.y, dwa_robot.Vx, dwa_robot.Vy, 
                            trape_c, dwa_robot.targetX, dwa_robot.targetY, trape_controle_flag, 
                            is_enable, path_enable);
    //台形制御を行う
    for(uint16_t i = 0; i < RASPI_TIME_STEP/MICON_TIME_STEP; i++){
      micon_trapezoidal_control(dwa_robot.targetX, dwa_robot.targetY, trape_c);
      }
    //ロボットの位置と仮想目標値に大きなズレが発生した場合に補正する
    micon_trapezoidal_robotXY_vertualXY_distance_check(trape_c, dwa_robot.x, dwa_robot.y);
    dwa_robot.x = trape_c->virtual_x;
    dwa_robot.y = trape_c->virtual_y;
    dwa_robot.theta = my_robot.orientation;
    dwa_robot.Vx = trape_c->unit_vector_x*trape_c->velocity;
    dwa_robot.Vy = trape_c->unit_vector_y*trape_c->velocity;
    dwa_robot.omega = my_robot.vel_angular[0];
    next_goal_pose->x = trape_c->virtual_x;
    next_goal_pose->y = trape_c->virtual_y;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"trap");
  }
  else if(is_enable == 0){
    trape_controle_flag[robot_id] = 0;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"corission");
    double verocity = hypot(my_robot.vel[0].x, my_robot.vel[0].y);
    dwa_robot.x = my_robot.pos.x*1000 + RASPI_TIME_STEP*my_robot.vel[0].x*1000 - 1/2*RASPI_TIME_STEP*RASPI_TIME_STEP*ROBOT_MAX_ACCEL*(my_robot.vel[0].x/verocity);
    dwa_robot.y = my_robot.pos.y*1000 + RASPI_TIME_STEP*my_robot.vel[0].y*1000 - 1/2*RASPI_TIME_STEP*RASPI_TIME_STEP*ROBOT_MAX_ACCEL*(my_robot.vel[0].y/verocity);
    dwa_robot.theta = my_robot.orientation;
    dwa_robot.Vx = my_robot.vel[0].x*1000 - ROBOT_MAX_ACCEL*RASPI_TIME_STEP;
    dwa_robot.Vy = my_robot.vel[0].y*1000 - ROBOT_MAX_ACCEL*RASPI_TIME_STEP;
    dwa_robot.omega = my_robot.vel_angular[0];
    next_goal_pose->x = dwa_robot.x;
    next_goal_pose->y = dwa_robot.y;
  }
  next_goal_pose->x = next_goal_pose->x/1000;   //単位変換(mm -> m)
  next_goal_pose->y = next_goal_pose->y/1000;   //単位変換(mm -> m)
}
