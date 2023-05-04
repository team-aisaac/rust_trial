#pragma once

// #include <vector>
#include <stdint.h>
#include <stdbool.h>

#include "TrapezoidalControl.h"
#include "DWA_path_recover.h"

// https://github.com/SSL-Roots/consai_ros2/blob/b5256af08073560abad72e4371d8e2e2619b56ad/consai_msgs/msg/State2D.msg#L2
typedef struct State {
  double x;
  double y;
  double theta;
} State;

// https://github.com/SSL-Roots/consai_ros2/blob/8cd53f868982c5923e995b86eaf1bf69de257710/robocup_ssl_msgs/msg/detection_tracked/Vector3.msg
typedef struct {
  float x;
  float y;
  float theta;
} Vector3;

// https://github.com/SSL-Roots/consai_ros2/blob/8cd53f868982c5923e995b86eaf1bf69de257710/robocup_ssl_msgs/msg/detection_tracked/Vector2.msg
typedef struct {
  float x;
  float y;
} Vector2;

// https://github.com/SSL-Roots/consai_ros2/blob/8cd53f868982c5923e995b86eaf1bf69de257710/robocup_ssl_msgs/msg/detection_tracked/TrackedBall.msg
typedef struct {
  Vector3 pos;
  Vector3 vel[1];
  float visibility;
} TrackedBall;

// https://github.com/SSL-Roots/consai_ros2/blob/8cd53f868982c5923e995b86eaf1bf69de257710/robocup_ssl_msgs/msg/detection_tracked/RobotId.msg
typedef struct {
//   enum TEAM_COLOR {
//     TEAM_COLOR_UNKNOWN =0,
//     TEAM_COLOR_YELLOW,
//     TEAM_COLOR_BLUE
//   };
  uint32_t id;
  uint32_t team_color;
} RobotId;

// https://github.com/SSL-Roots/consai_ros2/blob/8cd53f868982c5923e995b86eaf1bf69de257710/robocup_ssl_msgs/msg/detection_tracked/TrackedRobot.msg
typedef struct {
  RobotId robot_id;
  Vector2 pos;
  float orientation;
  Vector2 vel[1];
  float vel_angular[1];
  float visibility[1];
} TrackedRobot;

void decide_next_goal_xy(
  State goal_pose,
  State *midle_goal_pose,
  State *next_goal_pose,
  bool prohidited_zone_ignore,
  bool *midle_target_flag, 
  const unsigned int robot_id,
  TrackedRobot my_robot,
  bool team_is_yellow_,
  bool *trape_controle_flag,
  micon_trape_con *trape_c);

const int ROBOT_NUM = 16;
dwa_robot_path dwa_robot;

