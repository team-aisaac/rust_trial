#include <vector>
#include <cmath>
#include <cstdint>

#include "wrap_kick.h"
#include "tools.h"
#include "dribble.h"
#include "dwa2.h"

#include "controller_component.h"

void robot_wrap_kick(State &next_goal_pose, TrackedBall ball, State r_ball, State ball_goal, TrackedRobot my_robot, double &circumferential_error,double &radius_error, double &goal_theta,
    std::vector<bool> &ball_kick_con_flag, const unsigned int robot_id, float &kick_con_max_velocity_theta, bool free_kick_flag, int32_t ball_target_allowable_error, 
    bool ball_kick, bool &ball_kick_con, float &ob_unit_vec_circumferential_x, float &ob_unit_vec_circumferential_y, float &ob_unit_vec_radius_x, float &ob_unit_vec_radius_y,
    bool &wrap_kick_xy_flag, bool &dribble_active) {
  wrap_kick_robot wrap_robot;
  wrap_kick_ball wrap_ball;
  wrap_motion_pid pid;
  wrap_robot.w_robot_x = my_robot.pos.x*1000;          //m->mm
  wrap_robot.w_robot_y = my_robot.pos.y*1000;          //m->mm
  wrap_robot.w_robot_theta = 1000 * RAD_TO_DEG * my_robot.orientation; // 0.001 deg
  wrap_robot.w_robot_vx = my_robot.vel[0].x*1000;     //m/s->mm/s
  wrap_robot.w_robot_vy = my_robot.vel[0].y*1000;     //m/s->mm/s
  wrap_robot.w_robot_omega = 1000 * RAD_TO_DEG * my_robot.vel_angular[0]; // 0.001 deg/s
  wrap_ball.w_ball_x = ball.pos.x * 1000;             //m->mm
  wrap_ball.w_ball_y = ball.pos.y * 1000;             //m->mm
  wrap_ball.w_ball_vx = ball.vel[0].x * 1000;       //m/s->mm/s
  wrap_ball.w_ball_vy = ball.vel[0].y * 1000;       //m/s->mm/s
  wrap_ball.r_ball_x = r_ball.x * 1000;             //m->mm
  wrap_ball.r_ball_y = r_ball.y * 1000;             //m->mm
  wrap_ball.target_ball_x = ball_goal.x*1000;         //m->mm
  wrap_ball.target_ball_y = ball_goal.y*1000;         //m->mm
  //int32_t cul_circumferential_error, cul_radius_error, robot_goal_theta; //mm
  pid.ball_kick_cin_max_velo_theta = kick_con_max_velocity_theta*RAD_TO_DEG*1000;     //1000 des/s (deg/sを1000倍した値, 初期値は最大値)
  float ball_robot_distance = std::hypot(wrap_ball.r_ball_x - ROBOT_KICK_MIN_X, wrap_ball.r_ball_y);
  //RCLCPP_INFO(rclcpp::get_logger("dwa"),"distance : %f", ball_robot_distance);
  if(WRAP_KICK_CONTROL_CHANGE_DISTANCE < ball_robot_distance){
    ball_kick_con_flag[robot_id] = 0;
  }else{
    ball_kick_con_flag[robot_id] = 1;
    wrap_kick2(wrap_robot, wrap_ball, &pid, free_kick_flag, ball_target_allowable_error, ball_kick, &ball_kick_con, &dribble_active);
    ob_unit_vec_circumferential_x = pid.ob_unit_vec_circumferential_x;
    ob_unit_vec_circumferential_y = pid.ob_unit_vec_circumferential_y;
    ob_unit_vec_radius_x = pid.ob_unit_vec_radius_x;
    ob_unit_vec_radius_y = pid.ob_unit_vec_radius_y;
    circumferential_error = (double)pid.cul_circumferential_error/1000;   //単位変換(mm -> m)
    radius_error = (double)pid.cul_radius_error/1000;                     //単位変換(mm -> m)
    //RCLCPP_INFO(rclcpp::get_logger("kick"),"distance : %d", pid.cul_circumferential_error);
    next_goal_pose.x = (double)pid.goal_x/1000;
    next_goal_pose.y = (double)pid.goal_y/1000;
    wrap_kick_xy_flag = pid.robot_xy_con_flag;
    goal_theta = pid.robot_goal_theta/1000*DEG_TO_RAD;
  }
}

void dribble(State dribble_goal, TrackedBall ball, State r_ball, TrackedRobot my_robot, State &next_goal_pose, std::vector<bool> &dribble_con_flag, 
    const unsigned int robot_id, int32_t dribble_complete_distance, std::vector<micon_trape_con> &dribble_trape_c, std::vector<bool> &dribble_ball_move_flag, 
    double &circumferential_error,double &radius_error, float &ob_unit_vec_circumferential_x, float &ob_unit_vec_circumferential_y, float &ob_unit_vec_radius_x, 
    float &ob_unit_vec_radius_y, bool &dribble_active) {
  double ball_dribble_goal_distance = 1000 * std::hypot(dribble_goal.x - ball.pos.x, dribble_goal.y - ball.pos.y);
  double ball_robot_distance = std::hypot(1000 * r_ball.x - ROBOT_KICK_MIN_X, 1000 * r_ball.y);
  if(ball_dribble_goal_distance < dribble_complete_distance || ROBOT_DRIBBLE_DISTANCE < ball_robot_distance){
    //RCLCPP_INFO(rclcpp::get_logger("dribble"),"dribble finish");
    dribble_con_flag[robot_id] = 0;
    dribble_ball_move_flag[robot_id] = 0;
    return;
  }
  wrap_kick_robot drible_robot;
  wrap_kick_ball dribble_ball;
  wrap_motion_pid pid;
  dribble_con_flag[robot_id] = 1;
  drible_robot.w_robot_x = my_robot.pos.x*1000;          //m->mm
  drible_robot.w_robot_y = my_robot.pos.y*1000;          //m->mm
  drible_robot.w_robot_theta = 1000 * RAD_TO_DEG * my_robot.orientation; // 0.001 deg
  drible_robot.w_robot_vx = my_robot.vel[0].x*1000;      //m/s->mm/s
  drible_robot.w_robot_vy = my_robot.vel[0].y*1000;      //m/s->mm/s
  drible_robot.w_robot_omega = 1000 * RAD_TO_DEG * my_robot.vel_angular[0]; // 0.001 deg/s
  dribble_ball.w_ball_x = ball.pos.x * 1000;             //m->mm
  dribble_ball.w_ball_y = ball.pos.y * 1000;             //m->mm
  dribble_ball.w_ball_vx = ball.vel[0].x * 1000;       //m/s->mm/s
  dribble_ball.w_ball_vy = ball.vel[0].y * 1000;       //m/s->mm/s
  dribble_ball.r_ball_x = r_ball.x * 1000;             //m->mm
  dribble_ball.r_ball_y = r_ball.y * 1000;             //m->mm
  dribble_ball.target_ball_x = dribble_goal.x * 1000; //m->mm
  dribble_ball.target_ball_y = dribble_goal.y * 1000; //m->mm
  bool pre_dribble_ball_move_flag = dribble_ball_move_flag[robot_id];
  dribble_ball_move_flag[robot_id] = dribble_wrap_motion(drible_robot, dribble_ball, &pid, dribble_complete_distance, dribble_ball_move_flag[robot_id], &dribble_active);
  next_goal_pose.x = pid.goal_x/1000;   
  next_goal_pose.y = pid.goal_y/1000;
  ob_unit_vec_circumferential_x = pid.ob_unit_vec_circumferential_x;
  ob_unit_vec_circumferential_y = pid.ob_unit_vec_circumferential_y;
  ob_unit_vec_radius_x = pid.ob_unit_vec_radius_x;
  ob_unit_vec_radius_y = pid.ob_unit_vec_radius_y;
  circumferential_error = (double)pid.cul_circumferential_error/1000;   //単位変換(mm -> m)
  radius_error = (double)pid.cul_radius_error/1000;                     //単位変換(mm -> m)
  if(dribble_ball_move_flag[robot_id]){
    //RCLCPP_INFO(rclcpp::get_logger("dribble"),"trape con");
    dribble_active = true;
    if(pre_dribble_ball_move_flag == 0){
      //RCLCPP_INFO(rclcpp::get_logger("dribble"),"trape con start");
      micon_trapezoidal_DWA_change(drible_robot.w_robot_x, drible_robot.w_robot_y, drible_robot.w_robot_vx, drible_robot.w_robot_vy, &dribble_trape_c[robot_id], 
        dribble_ball.target_ball_x, dribble_ball.target_ball_y, pre_dribble_ball_move_flag, 1, 0);
    }
    for(uint16_t i = 0; i < RASPI_TIME_STEP/MICON_TIME_STEP; i++){
      micon_trapezoidal_dribble_control(dribble_ball.target_ball_x, dribble_ball.target_ball_y, &dribble_trape_c[robot_id], dribble_ball.w_ball_x, dribble_ball.w_ball_y);
      pid.robot_goal_theta = dribble_move_deg(dribble_ball.r_ball_y, pid.robot_goal_theta);
    }
    next_goal_pose.x = dribble_trape_c[robot_id].virtual_x/1000;    //単位変換(mm -> m)
    next_goal_pose.y = dribble_trape_c[robot_id].virtual_y/1000;    //単位変換(mm -> m)
  }   
  next_goal_pose.theta = pid.robot_goal_theta/1000*DEG_TO_RAD;
}

void decide_next_goal_xy(State goal_pose, State &midle_goal_pose, State &next_goal_pose, bool prohidited_zone_ignore, bool &midle_target_flag, 
    const unsigned int robot_id, TrackedRobot my_robot, bool team_is_yellow_, std::vector<bool> &trape_controle_flag, std::vector<micon_trape_con> &trape_c)
{
  dwa_robot[robot_id].targetX = goal_pose.x*1000;   //単位変換(m ->mm)
  dwa_robot[robot_id].targetY = goal_pose.y*1000;   //単位変換(m ->mm)
  dwa_robot[robot_id].targetTheta = goal_pose.theta;
  dwa_robot[robot_id].midle_targetX = midle_goal_pose.x*1000; //単位変換(m ->mm)
  dwa_robot[robot_id].midle_targetY = midle_goal_pose.y*1000; //単位変換(m ->mm)
  float robot_virtual_goal_distance = std::hypot(dwa_robot[robot_id].x - my_robot.pos.x*1000, dwa_robot[robot_id].y - my_robot.pos.y*1000);
  if(ROBOT_POSITION_RESET_DISTANCE < robot_virtual_goal_distance){
    dwa_robot[robot_id].x = my_robot.pos.x*1000;
    dwa_robot[robot_id].y = my_robot.pos.y*1000;
    dwa_robot[robot_id].theta = my_robot.orientation;
    dwa_robot[robot_id].Vx = my_robot.vel[0].x*1000;
    dwa_robot[robot_id].Vy = my_robot.vel[0].y*1000;
    dwa_robot[robot_id].omega = my_robot.vel_angular[0];
  }
  
  int32_t numOfObstacle = 0;
  int32_t obs_x[2*ROBOT_NUM], obs_y[2*ROBOT_NUM], obs_vx[2*ROBOT_NUM], obs_vy[2*ROBOT_NUM], obs_ax[2*ROBOT_NUM], obs_ay[2*ROBOT_NUM];
  TrackedRobot obs_robot;
  int32_t k = 0;
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
  execDWA(dwa_robot[robot_id].x, dwa_robot[robot_id].y, dwa_robot[robot_id].theta, dwa_robot[robot_id].Vx, dwa_robot[robot_id].Vy, dwa_robot[robot_id].omega, 
        &dwa_robot[robot_id].targetX, &dwa_robot[robot_id].targetY, dwa_robot[robot_id].targetTheta, &dwa_robot[robot_id].midle_targetX, &dwa_robot[robot_id].midle_targetY,
        numOfObstacle, obs_x, obs_y, obs_vx, obs_vy, obs_ax, obs_ay, prohidited_zone_ignore, 
        &midle_target_flag, &is_enable, &path_enable, &output_vx, &output_vy, &output_omega, &output_ax, &output_ay);
  float next_goal_pose_x = dwa_robot[robot_id].x;
  float next_goal_pose_y = dwa_robot[robot_id].y;
  float robot_vertual_vel_x = dwa_robot[robot_id].Vx;
  float robot_vertual_vel_y = dwa_robot[robot_id].Vy;
  if(is_enable == 1 && path_enable == 1){
    //DWAを行う
    for(uint16_t i = 0; i < RASPI_TIME_STEP/MICON_TIME_STEP; i++){
      DWA_path_recover(&next_goal_pose_x, &next_goal_pose_y, &robot_vertual_vel_x, &robot_vertual_vel_y, 
                        (float) output_ax, (float) output_ay, (float) ROBOT_MAX_VEL);
      float robot_dwa_goal_distance = std::hypot(next_goal_pose_x - my_robot.pos.x*1000, next_goal_pose_y - my_robot.pos.y*1000);
      if(DWA_ROBOTXY_VIRUALXY_DISTANCE_CHECK < robot_dwa_goal_distance){
        //RCLCPP_INFO(rclcpp::get_logger("dwa"),"dwa_overdwa_overdwa_overdwa_overdwa_overdwa_overdwa_overdwa_over");
        break;
      }
    }
    dwa_robot[robot_id].x = next_goal_pose_x;
    dwa_robot[robot_id].y = next_goal_pose_y;
    dwa_robot[robot_id].theta = my_robot.orientation;
    dwa_robot[robot_id].Vx = robot_vertual_vel_x;
    dwa_robot[robot_id].Vy = robot_vertual_vel_y;
    dwa_robot[robot_id].omega = my_robot.vel_angular[0];
    next_goal_pose.x = next_goal_pose_x;
    next_goal_pose.y = next_goal_pose_y;
    trape_controle_flag[robot_id] = 0;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"dwa");
  }
  else if(is_enable == 1 && path_enable == 0){
    //DWAから台形制御に移行する際に変数を設定する関数
    trape_controle_flag[robot_id] = micon_trapezoidal_DWA_change(dwa_robot[robot_id].x, dwa_robot[robot_id].y, dwa_robot[robot_id].Vx, dwa_robot[robot_id].Vy, 
                            &trape_c[robot_id], dwa_robot[robot_id].targetX, dwa_robot[robot_id].targetY, trape_controle_flag[robot_id], 
                            is_enable, path_enable);
    //台形制御を行う
    for(uint16_t i = 0; i < RASPI_TIME_STEP/MICON_TIME_STEP; i++){
      micon_trapezoidal_control(dwa_robot[robot_id].targetX, dwa_robot[robot_id].targetY, &trape_c[robot_id]);
      }
    //ロボットの位置と仮想目標値に大きなズレが発生した場合に補正する
    micon_trapezoidal_robotXY_vertualXY_distance_check(&trape_c[robot_id], dwa_robot[robot_id].x, dwa_robot[robot_id].y);
    dwa_robot[robot_id].x = trape_c[robot_id].virtual_x;
    dwa_robot[robot_id].y = trape_c[robot_id].virtual_y;
    dwa_robot[robot_id].theta = my_robot.orientation;
    dwa_robot[robot_id].Vx = trape_c[robot_id].unit_vector_x*trape_c[robot_id].velocity;
    dwa_robot[robot_id].Vy = trape_c[robot_id].unit_vector_y*trape_c[robot_id].velocity;
    dwa_robot[robot_id].omega = my_robot.vel_angular[0];
    next_goal_pose.x = trape_c[robot_id].virtual_x;
    next_goal_pose.y = trape_c[robot_id].virtual_y;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"trap");
  }
  else if(is_enable == 0){
    trape_controle_flag[robot_id] = 0;
    //RCLCPP_INFO(rclcpp::get_logger("dwa"),"corission");
    double verocity = std::hypot(my_robot.vel[0].x, my_robot.vel[0].y);
    dwa_robot[robot_id].x = my_robot.pos.x*1000 + RASPI_TIME_STEP*my_robot.vel[0].x*1000 - 1/2*RASPI_TIME_STEP*RASPI_TIME_STEP*ROBOT_MAX_ACCEL*(my_robot.vel[0].x/verocity);
    dwa_robot[robot_id].y = my_robot.pos.y*1000 + RASPI_TIME_STEP*my_robot.vel[0].y*1000 - 1/2*RASPI_TIME_STEP*RASPI_TIME_STEP*ROBOT_MAX_ACCEL*(my_robot.vel[0].y/verocity);
    dwa_robot[robot_id].theta = my_robot.orientation;
    dwa_robot[robot_id].Vx = my_robot.vel[0].x*1000 - ROBOT_MAX_ACCEL*RASPI_TIME_STEP;
    dwa_robot[robot_id].Vy = my_robot.vel[0].y*1000 - ROBOT_MAX_ACCEL*RASPI_TIME_STEP;
    dwa_robot[robot_id].omega = my_robot.vel_angular[0];
    next_goal_pose.x = dwa_robot[robot_id].x;
    next_goal_pose.y = dwa_robot[robot_id].y;
  }
  next_goal_pose.x = next_goal_pose.x/1000;   //単位変換(mm -> m)
  next_goal_pose.y = next_goal_pose.y/1000;   //単位変換(mm -> m)
}
