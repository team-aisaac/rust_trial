#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<math.h>
#include "RaspiTrapezoidalControl.h"
#include "tools.h"

#include <time.h>

//int test_time_step = 0;

bool trapezoidal_DWA_change(int32_t x, int32_t y, int32_t vx, int32_t vy, trape_con *trape_c, int32_t targetX, int32_t targetY, float ax, float ay){
    float distance;
    float velocity;
    if((x-targetX)*(x-targetX) + (y-targetY)*(y-targetY) != 0){
        distance = sqrt((x-targetX)*(x-targetX) + (y-targetY)*(y-targetY));
    }
    else{
        distance = 0;
    }
    if(vx*vx + vy*vy != 0){
        velocity = sqrt(vx*vx + vy*vy);
    }
    else{
        velocity = 0;
    }
    float dis_vel_cos = 1;
    if(distance != 0 && velocity != 0){
        dis_vel_cos = ((targetX-x)*vx + (targetY-y)*vy) / (distance*velocity);
    }
    //台形制御の初期条件
    trape_c->velocity = velocity * dis_vel_cos;
    trape_c->virtual_x = x;
    trape_c->virtual_y = y;
    //trape_c->accel = sqrt(ax*ax + ay*ay) * dis_vel_cos;
    trape_c->accel = 0;
    if(velocity < 20){ //ロボットの速度がかなり遅い場合
        trape_c->velocity = 0;
        trape_c->virtual_x = x;
        trape_c->virtual_y = y;
    }
    if(trape_c->accel < 0){
        trape_c->accel = 0;
    }
    if(distance != 0){
        trape_c->unit_vector_x = (targetX - x)/distance;
        trape_c->unit_vector_y = (targetY - y)/distance;
    }
    else{
        trape_c->unit_vector_x = 0;
        trape_c->unit_vector_y = 0;
    }
    if(distance < 1000){
        trape_c->accel = 0;
    }
}

int8_t jerk_flag_check(float distance, float braking_distance){
    if(distance <= braking_distance){
        return -1;
    }
    else{
        return 1;
    }
}

//台形制御
void trapezoidal_control(int32_t targetX, int32_t targetY, trape_con *trape_c){
    if((targetX - trape_c->virtual_x) * (targetX - trape_c->virtual_x) + (targetY - trape_c->virtual_y) * (targetY - trape_c->virtual_y) == 0){
        trape_c->virtual_x = trape_c->virtual_x + trape_c->velocity * trape_c->unit_vector_x * TRAP_TIME_STEP;
        trape_c->virtual_y = trape_c->virtual_y + trape_c->velocity * trape_c->unit_vector_y * TRAP_TIME_STEP;
        return;
    }
    float distance = sqrt((targetX - trape_c->virtual_x) * (targetX - trape_c->virtual_x) + (targetY - trape_c->virtual_y) * (targetY - trape_c->virtual_y));
    //通り過ぎた場合
    if((trape_c->unit_vector_x * (targetX - trape_c->virtual_x) + trape_c->unit_vector_y * (targetY - trape_c->virtual_y)) / distance < 0){
        trape_c->velocity = -trape_c->velocity;
        trape_c->accel = -trape_c->accel;
    }
    //目標値の計算と単位ベクトルの算出
    trape_c->unit_vector_x = (targetX - trape_c->virtual_x) / distance;
    trape_c->unit_vector_y = (targetY - trape_c->virtual_y) / distance;
    //jerkの判定
    int8_t jerk_flag = 0; //-1:jerk--, 1:jerk++, 0:jerk = jerk
    //速度の最大値に近づいている場合
    if((trape_c->max_velocity - trape_c->velocity) <= (trape_c->accel*trape_c->accel / (2*trape_c->jerk))){
        jerk_flag = -1;
    }
    //目標から離れていく方向に速度を持っている場合
    else if(trape_c->velocity < 0 && trape_c->accel < 0){
        jerk_flag = 50;
        //printf("1 %d\n", jerk_flag);
    }
    else if(trape_c->velocity < 0 && 0 <= trape_c->accel){
        float in_sqrt = trape_c->accel*trape_c->accel + 2*trape_c->jerk*trape_c->velocity;
        if(in_sqrt < 0){
            jerk_flag = 50;
            //printf("2, %d\n", jerk_flag);
        }
        else{
            float a_t = sqrt(in_sqrt);
            float t_v0 = (trape_c->accel - a_t)/trape_c->jerk;
            float negative_braking_distance = -trape_c->jerk*t_v0*t_v0*t_v0/6 + trape_c->accel*t_v0*t_v0/2 + trape_c->velocity*t_v0;
            float b = (1 + sqrt(2))*a_t/trape_c->jerk;
            float t = (a_t + b * trape_c->jerk)/(2 * trape_c->jerk);
            float positive_braking_distance = -trape_c->jerk*t*t*t/6 + trape_c->accel*t*t/2 + trape_c->velocity*t + trape_c->jerk*(b-t)*(b-t)*(b-t)/2;
            float braking_distance = positive_braking_distance - negative_braking_distance;
            jerk_flag = jerk_flag_check(distance, braking_distance);
            if(jerk_flag == 1){
                jerk_flag = 50;
            }
            //printf("3 %d\n", jerk_flag);
        }
    }
    //制動距離と目標点までの距離の差から躍度を算出する場合
    else{
        float b1 = 2*trape_c->accel* trape_c->accel;
        float b2 = 4 * trape_c->jerk * trape_c->velocity;
        float b = (trape_c->accel + sqrt(b1 + b2)) / trape_c->jerk;
        float a_check = (trape_c->accel - b * trape_c->jerk) / 2;
        if(-trape_c->max_accel < a_check){
            float t = (trape_c->accel + b * trape_c->jerk)/(2 * trape_c->jerk);
            float braking_distance = -trape_c->jerk*t*t*t/6 + trape_c->accel*t*t/2 + trape_c->velocity*t + trape_c->jerk*(b-t)*(b-t)*(b-t)/6;
            jerk_flag = jerk_flag_check(distance, braking_distance+1);
            //printf("1,%f,%f\n",distance,braking_distance);
        }
        else if(a_check <= -trape_c->max_accel){
            float t = (trape_c->accel + trape_c->max_accel) / trape_c->jerk;
            float v1 = -trape_c->jerk*t*t/2 + trape_c->accel*t + trape_c->velocity;
            float v2 = trape_c->max_accel*trape_c->max_accel / (2*trape_c->jerk);
            float distance1 = -trape_c->jerk*t*t*t/6 + trape_c->accel*t*t/2 + trape_c->velocity*t;
            float distance2 = trape_c->max_accel * trape_c->max_accel * trape_c->max_accel / (6 * trape_c->jerk * trape_c->jerk);
            float braking_distance = distance1 + (v1 + v2)*(v1 - v2) / (2 * trape_c->max_accel) + distance2;
            jerk_flag = jerk_flag_check(distance, braking_distance+1);
            //printf("2,%f,%f\n",distance, braking_distance);
        }
        float t = (trape_c->accel+b*trape_c->jerk)/(2*trape_c->jerk);
        if(t <= 0){
            jerk_flag = 1;
            //printf("3\n");
        }
    }
    //printf("jerk %d\n", jerk_flag);
    //速度･加速度･目標値の算出
    trape_c->accel = trape_c->accel + jerk_flag * trape_c->jerk * TRAP_TIME_STEP;
    if(trape_c->max_accel < trape_c->accel) {
        trape_c->accel = trape_c->max_accel;
    }
    else if(trape_c->accel < -trape_c->max_accel){
        trape_c->accel = -trape_c->max_accel;
    }
    trape_c->velocity = trape_c->velocity + trape_c->accel * TRAP_TIME_STEP;
    if(trape_c->max_velocity < trape_c->velocity){
        trape_c->velocity = trape_c->max_velocity;
    }
    else if(trape_c->velocity < -trape_c->max_velocity){
        trape_c->velocity = -trape_c->max_velocity;
    }
    trape_c->virtual_x = trape_c->virtual_x + trape_c->velocity * trape_c->unit_vector_x * TRAP_TIME_STEP;
    trape_c->virtual_y = trape_c->virtual_y + trape_c->velocity * trape_c->unit_vector_y * TRAP_TIME_STEP;
}

void trapezoidal_init(trape_con *trape_c){
    trape_c->jerk = ROBOT_MAX_JARK;
    trape_c->accel = 0;
    trape_c->max_accel = ROBOT_MAX_ACCEL;
    trape_c->velocity = 0;
    trape_c->max_velocity = ROBOT_MAX_VEL;
    trape_c->virtual_x = 0;
    trape_c->virtual_y = 0;
}
/*
int main(void) {
    printf("start.\n");
    //テスト時のロボットの初期状態
    trap_robot_state test_robot_state;
    test_robot_state.x = 0;
    test_robot_state.y = 0;
    test_robot_state.theta = 0;
    test_robot_state.Vx = 0;
    test_robot_state.Vy = 0;
    test_robot_state.omega = 0;
    test_robot_state.targetX = -3000;
    test_robot_state.targetY = -5000;
    test_robot_state.targetTheta = 0;

    int32_t num_Obs = 0;
    int32_t Obs_X[0];
    int32_t Obs_y[0];
    int32_t Obs_VX[0];
    int32_t Obs_VY[0];
    /*
    int32_t num_Obs = 2;
    int32_t Obs_X[2];
    int32_t Obs_y[2];
    int32_t Obs_VX[2];
    int32_t Obs_VY[2];
    Obs_X[0] = 500;
    Obs_y[0] = 1000;
    Obs_VX[1] = 3000;
    Obs_VY[1] = 2000;
    */
/*
    bool enable;
    bool velorAccel;
    trape_con trape_c;
    trapezoidal_init(&trape_c);
    printf("jerk %f,x %f, y %f,v %f,a %f\n",trape_c.jerk,trape_c.accel,trape_c.max_accel,trape_c.velocity,trape_c.max_velocity);

    printf("Trapezoidal\n");
    clock_t start = clock();
    for(test_time_step = 0; test_time_step<5000; test_time_step++){
        int32_t output_vx;
        int32_t output_vy;
        int32_t output_omega;
        int32_t output_ax;
        int32_t output_ay;
        float virtual_x;
        float virtual_y;
        trapezoidal_control(test_robot_state.targetX, test_robot_state.targetY, &trape_c);
        //test_robot_state.Vx = test_robot_state.Vx + output_x * TIME_STEP;
        //test_robot_state.Vy = test_robot_state.Vy + output_y * TIME_STEP;
        //test_robot_state.x = test_robot_state.x + test_robot_state.Vx * TIME_STEP;
        //test_robot_state.y = test_robot_state.y + test_robot_state.Vy * TIME_STEP;
        test_robot_state.x = trape_c.virtual_x;
        test_robot_state.y = trape_c.virtual_y;
        float v = trape_c.velocity;
        //printf("%d,%f\n",test_time_step, v);
        printf("%d,%d,%d,%f,%f\n",test_time_step, test_robot_state.x, test_robot_state.y,v,trape_c.accel);
    }
    clock_t end = clock();

    printf("%.2f(sec))\n",(double)(end-start)/CLOCKS_PER_SEC);
}
*/