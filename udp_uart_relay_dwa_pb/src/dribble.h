#ifndef _DRIBBLE_H_
#define _DRIBBLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "TrapezoidalControl.h"
#include "wrap_kick.h"

bool dribble_wrap_motion(wrap_kick_robot robot, wrap_kick_ball ball, wrap_motion_pid *pid, int32_t ball_target_allowable_error, bool dribble_ball_move_flag, bool *dribble_active);

int32_t dribble_move_deg(int32_t r_ball_y, int32_t robot_goal_theta);

#ifdef __cplusplus
}
#endif
#endif // _DRIBBLE_H_