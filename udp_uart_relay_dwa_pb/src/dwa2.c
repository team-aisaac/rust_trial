#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include <unistd.h>
#include<math.h>
#include "dwa.h"
#include "Target_abjust.c"
#include"RaspiTrapezoidalControl.c"

#include <time.h>

float robot_max_velo = 3000; //ロボットの最大速度(mm/s)
float robot_max_accel = 2000; //ロボットの最大速度(mm/s^2)

float x_penalty_line_puls;
float x_penalty_line_minus;
float y_penalty_line_puls;
float y_penalty_line_minus;
float x_outside_line_puls;
float x_outside_line_minus;
float y_outside_line_puls;
float y_outside_line_minus;

//基本関数
//最大値を算出
float max_value(float data[], uint16_t size){
    float max = data[0];
    for(int i = 1; i < size; i++){
        if(max < data[i]){
            max = data[i];
        }
    }
    return max;
}
//最小値を算出
float min_value(float data[], uint16_t size){
    float min = data[0];
    for(int i = 1; i < size; i++){
        if(data[i] < min){
            min = data[i];
        }
    }
    return min;
}
//正規化
void min_max_normalize(float *data, uint16_t size){
    float max = max_value(data, size);
    float min = min_value(data, size);
    if(max - min == 0){
        for(int i = 0; i<size; i++){
            data[i] = 0;
        }
    }
    else{
        for(int i = 0; i<size; i++){
            data[i] = (data[i]-min)/(max - min);
        }
    }
}
//角度範囲内に再計算(rad)
float angle_range_corrector(float angle){
    if(angle > M_PI){
        while(angle > M_PI){
            angle -=  2 * M_PI;
        }
    }
    else if(angle < -M_PI){
        while(angle < -M_PI){
            angle += 2 * M_PI;
        }
    }
    return angle;
}
//角度範囲内に再計算(0-360deg)
float angle_range_corrector_deg(float angle){
    if(angle < 0){
        while(angle < 0){
            angle = angle + 360;
        }
    }
    else if(360 < angle){
        while(360 < angle){
            angle = angle - 360;
        }
    }
    return angle;
}
//角度が範囲内かを判定する(範囲内:1, 範囲外:0, 単位:rad)
bool angele_check(float check_angle, float angle_max, float angle_min){
    check_angle = angle_range_corrector(check_angle);
    angle_min = angle_range_corrector(angle_min);
    angle_max = angle_range_corrector(angle_max);
    if(angle_min <= angle_max){
        if(angle_min <= check_angle && check_angle <= angle_max){
            return 1;
        }
    }
    else{
        if(angle_max < 0){
            angle_max += 2 * M_PI;
        }
        if(angle_min < 0){
            angle_min += 2 * M_PI;
        }
        if(check_angle < 0){
            check_angle += 2 * M_PI;
        }
        if(angle_min <= check_angle && check_angle <= angle_max){
            return 1;
        }
    }
    return 0;
}
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.acosfの引数などに使用)
float modifid_acosf(float value){
    if(1.0 < value){
        return acosf(1.0);
    }
    if(value < -1.0){
        return acosf(-1.0);
    }
    return acosf(value);
}
//ロボットの軌道予測(long path用)
void predict_robot_long_path(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, 
                            path_long_prediction *long_prediction, float accel_x, float accel_y, uint32_t long_path_predict_step){
    float temp_x = x;
    float temp_y = y;
    float vx = Vx;
    float vy = Vy;
    for(int i = 0; i < long_path_predict_step; i++){
        temp_x = vx * TIME_STEP + 1/2*accel_x * TIME_STEP * TIME_STEP + temp_x;
        temp_y = vy * TIME_STEP + 1/2*accel_y * TIME_STEP * TIME_STEP + temp_y;
        vx = vx + accel_x * TIME_STEP;
        vy = vy + accel_y * TIME_STEP;
        float v = sqrt(vx*vx + vy*vy);
        if(v > robot_max_velo){
            vx = vx/v*robot_max_velo;
            vy = vy/v*robot_max_velo;
        }
        long_prediction->x[i] = temp_x;
        long_prediction->y[i] = temp_y;
        long_prediction->theta[i] = theta;
        long_prediction->vx[i] = vx;
        long_prediction->vy[i] = vy;
        long_prediction->omega[i] = omega;
    }
}
//ロボットの軌道予測(fine path用)
void predict_robot_fine_path(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, 
                            path_fine_prediction *fine_prediction, float accel_x, float accel_y, uint32_t fine_path_predict_step){
    float temp_x = x;
    float temp_y = y;
    float vx = Vx;
    float vy = Vy;
    for(int i = 0; i < fine_path_predict_step; i++){
        temp_x = vx * TIME_STEP + 1/2*accel_x * TIME_STEP * TIME_STEP + temp_x;
        temp_y = vy * TIME_STEP + 1/2*accel_y * TIME_STEP * TIME_STEP + temp_y;
        vx = vx + accel_x * TIME_STEP;
        vy = vy + accel_y * TIME_STEP;
        float v = sqrt(vx*vx + vy*vy);
        if(v > robot_max_velo){
            vx = vx/v*robot_max_velo;
            vy = vy/v*robot_max_velo;
        }
        fine_prediction->x[i] = temp_x;
        fine_prediction->y[i] = temp_y;
        fine_prediction->theta[i] = theta;
        fine_prediction->vx[i] = vx;
        fine_prediction->vy[i] = vy;
        fine_prediction->omega[i] = omega;
    }
}

//ロボットと目標点までの距離を計算する
float cal_robot_target_distance(int32_t x, int32_t y, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, bool midle_target_flag){
    float distance;
    if(midle_target_flag == 1 && (midle_targetX-x)*(midle_targetX-x) + (midle_targetY-y)*(midle_targetY-y) != 0){
        distance = sqrt((midle_targetX-x)*(midle_targetX-x) + (midle_targetY-y)*(midle_targetY-y));
    }
    else if(midle_target_flag == 0 && (targetX-x)*(targetX-x) + (targetY-y)*(targetY-y) != 0){
        distance = sqrt((targetX-x)*(targetX-x) + (targetY-y)*(targetY-y));
    }
    else{
        distance = 0;
    }
    return distance;
}
//DWAで探索する時間とステップ数を算出する
void dwa_predict_step_cal(uint32_t *long_path_predict_step, uint32_t *fine_path_predict_step, float distance){
    float k = 0.9;
    float long_path_predict_sec = k * distance / 1000;
    if(2 < long_path_predict_sec){
        long_path_predict_sec = 2;
    }
    float fine_path_predict_sec = long_path_predict_sec/2;
    if(long_path_predict_sec < 0.5){
        long_path_predict_sec = 0.5;
    }
    if(fine_path_predict_sec < 0.1){
        fine_path_predict_sec = 0.1;
    }
    *long_path_predict_step = long_path_predict_sec / TIME_STEP;
    *fine_path_predict_step = fine_path_predict_sec / TIME_STEP;
}
//障害物との衝突を回避するためにDWAで探索する時間とステップを算出する
uint32_t dwa_cllision_avoid_step_cal(){
    uint32_t predict_step = COLLISION_AVOID_PREDICT_SECOND/TIME_STEP;
    return predict_step;
}

//DWA
void execDWA(   int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t *targetX, int32_t *targetY, int32_t targetTheta, int32_t *midle_targetX, int32_t *midle_targetY,
                int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[], int32_t ObstacleVX[], int32_t ObstacleVY[], int32_t ObstacleAX[], int32_t ObstacleAY[], bool prohidited_zone_ignore, bool *midle_target_flag,
                bool *is_enable, bool *path_enable, bool *prohidited_zone_start, int32_t *output_vx, int32_t *output_vy, int32_t *output_omega, int32_t *output_ax, int32_t *output_ay){
    //フィールドデータの代入
    //field_data();
    field_data(&x_penalty_line_puls, &x_penalty_line_minus, &y_penalty_line_puls, &y_penalty_line_minus, &x_outside_line_puls,
                &x_outside_line_minus, &y_outside_line_puls, &y_outside_line_minus);
    //フィールド外の目標値やペナルティゾーン内の目標値を適正な箇所に設置し直す
    *midle_target_flag = target_abjust(x, y, targetX, targetY, midle_targetX, midle_targetY, prohidited_zone_ignore, prohidited_zone_start);
    //現在距離と目標値から予測時間を算出
    float distance = cal_robot_target_distance(x, y, *targetX, *targetY, *midle_targetX, *midle_targetY, *midle_target_flag);
    uint32_t long_path_predict_step;    //long pathの予想ステップ数
    uint32_t fine_path_predict_step;    //fine pathの予想ステップ数
    dwa_predict_step_cal(&long_path_predict_step, &fine_path_predict_step, distance);   //DWAの予想ステップ数を決定
    uint32_t obs_predict_step = dwa_cllision_avoid_step_cal();
    //DWAを実行するための変数設定
    path_long_prediction *long_prediction = (path_long_prediction*)calloc(1000, sizeof(path_long_prediction)); //解像度粗目で製作したpathを入れる構造体
    path_long_prediction long_opt_path;         //解像度粗目で製作したpathの中で最適なpathを入れる構造体
    path_fine_prediction *fine_prediction = (path_fine_prediction*)calloc(10000, sizeof(path_fine_prediction));  //解像度細かめで製作したpathを入れる構造体
    path_fine_prediction fine_opt_path;         //解像度細かめで製作したpathの中で最適なpathを入れる構造体
    
    //計算するべき障害物を計算して構造体に代入
    int16_t obs_size = nearest_obs_mun(x, y, numOfObstacle, ObstacleX, ObstacleY);   //計算するべき障害物の数
    obs *obs_imfo = (obs*)calloc(obs_size, sizeof(obs));
    cul_obs(x, y, numOfObstacle, ObstacleX, ObstacleY, ObstacleVX, ObstacleVY, ObstacleAX, ObstacleAY, obs_size, obs_imfo);
        
    //360度全方向の加速度でPath作成(解像度粗目)
    uint16_t long_path_size; //解像度粗目で製作したpathの数を入れる変数 
    //解像度粗目でpathを製作
    make_path_long( x, y, theta, Vx, Vy, omega, targetTheta,
                    long_prediction, robot_max_accel/2, robot_max_accel/2, DELTA_BIG_ACCEL,
                    ONE_ROUND_DEG/2, ONE_ROUND_DEG/2, DELTA_BIG_DEG, &long_path_size, obs_predict_step);
    //pathがペナルティゾーンに入っていないかを確認
    field_out_penalty_zone_long_path_check(long_prediction, long_path_size, long_path_predict_step, *prohidited_zone_start, prohidited_zone_ignore);
    //中間目標点が設置されている時のロボットの速度制限(long path用)
    midle_target_velo_limit_long_path(x, y, Vx, Vy, *targetX, *targetY, *midle_targetX, *midle_targetY, long_prediction, long_path_size, long_path_predict_step, *midle_target_flag);
    //360度全方向の加速度でPath評価(解像度粗目)&その時の加速度の方向を抽出
    eval_long_path( x, y, theta, Vx, Vy, omega, *targetX, *targetY, targetTheta, *midle_targetX, *midle_targetY,
                    long_prediction, long_path_size,
                    obs_size, obs_imfo, obs_predict_step, *midle_target_flag,
                    &long_opt_path, long_path_predict_step);
    //最高評価となった角度付近で再度Path作成(解像度細かめ, )
    uint16_t fine_path_size; //解像度細かめで製作したpathの数を入れる変数
    make_path_fine( x, y, theta, Vx, Vy, omega, targetTheta,
                    fine_prediction, long_opt_path.accel, DELTA_BIG_ACCEL*2, DELTA_FINE_ACCEL,
                    long_opt_path.accel_derection, DELTA_BIG_DEG, DELTA_FINE_DEG, &fine_path_size, obs_predict_step);
    //pathがペナルティゾーンに入っていないかを確認
    field_out_penalty_zone_fine_path_check(fine_prediction, fine_path_size, fine_path_predict_step, *prohidited_zone_start, prohidited_zone_ignore);
    //中間目標点が設置されている時のロボットの速度制限(fine path用)
    midle_target_velo_limit_fine_path(x, y, Vx, Vy, *targetX, *targetY, *midle_targetX, *midle_targetY, fine_prediction, fine_path_size, fine_path_predict_step, *midle_target_flag);
    //最高評価となった角度付近で再度Path評価(解像度細かめ)
    eval_fine_path( x, y, theta, Vx, Vy, omega, *targetX, *targetY, targetTheta, *midle_targetX, *midle_targetY,
                    fine_prediction, fine_path_size,
                    obs_size, obs_imfo, obs_predict_step, *midle_target_flag,
                    &fine_opt_path, fine_path_predict_step);
    //制動距離を算出して壁に激突しないか判定する
    bool ollision_flag = 0;
    avoid_field_wall_ollision(x, y, theta, Vx, Vy, omega, &fine_opt_path, fine_path_predict_step, &ollision_flag);

    //最適と計算されたpathを格納
    *output_vx = fine_opt_path.vx[0];
    *output_vy = fine_opt_path.vy[0];
    float angle = angle_range_corrector(fine_opt_path.accel_derection/180*M_PI);
    *output_ax = fine_opt_path.accel*cosf(angle);
    *output_ay = fine_opt_path.accel*sinf(angle);
    *output_omega = 0;
    *is_enable = DWA_enable_check(long_opt_path.availability_flag, fine_opt_path.availability_flag);
    *path_enable = path_enable_check(x, y, Vx, Vy, *targetX, *targetY, *output_vx, *output_vy, *output_ax, *output_ay, obs_imfo, obs_size, long_path_predict_step, prohidited_zone_ignore,
                                    *midle_target_flag, ollision_flag, *path_enable, *prohidited_zone_start);
    
    //メモリの解放
    free(obs_imfo);
    free(long_prediction);
    free(fine_prediction);
}
//DWAで全体方位をざっくり計算するために経路を検索する関数(long path用)
void make_path_long(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetTheta,
                    path_long_prediction *long_prediction, float midle_accel_value, float accel_range, float accel_resolution,
                    float midle_deg_value, float deg_range, float deg_resolution, uint16_t *long_path_num ,uint32_t long_path_predict_step){
    int path_num = 0;
    for(float accel = accel_resolution; accel <= midle_accel_value + accel_range; accel = accel + accel_resolution){
        for(float accel_derection = midle_deg_value - deg_range; accel_derection < midle_deg_value + deg_range; accel_derection = accel_derection + deg_resolution){
            float accel_x = accel*cosf(accel_derection/180*M_PI);
            float accel_y = accel*sinf(accel_derection/180*M_PI);
            long_prediction[path_num].accel = accel;
            long_prediction[path_num].accel_derection = accel_derection;
            
            predict_robot_long_path(x, y, theta, Vx, Vy, omega, &long_prediction[path_num], accel_x, accel_y, long_path_predict_step);
            
            path_num++;
        }
    }
    *long_path_num = path_num;
}
//DWAで全体方位を計算した後に、詳細を検討するために検索する関数
void make_path_fine(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetTheta,
                    path_fine_prediction *fine_prediction, float midle_accel_value, float accel_range, float accel_resolution,
                    float midle_deg_value, float deg_range, float deg_resolution, uint16_t *fine_path_num, uint32_t fine_path_predict_step){
    int path_num = 0;
    float accel_min = midle_accel_value - accel_range;
    float accel_max = midle_accel_value + accel_range;
    if(accel_min < 100){
        accel_min = 100;
    }
    if(robot_max_accel < accel_max){
        accel_max = robot_max_accel;
    }
    
    for(float accel = accel_min; accel <= accel_max; accel = accel + accel_resolution){
        for(float accel_derection = midle_deg_value - deg_range; accel_derection <= midle_deg_value + deg_range; accel_derection = accel_derection + deg_resolution){
                for(int i = 0; i<2; i++){
                    float angle_diff = 180*i;
                    float angle_deg = accel_derection + angle_diff;
                    angle_deg = angle_range_corrector_deg(angle_deg);
                    float angle = angle_deg*M_PI/180;
                    angle = angle_range_corrector(angle);
                    float accel_x = accel*cosf(angle);
                    float accel_y = accel*sinf(angle);
                    fine_prediction[path_num].accel = accel;
                    fine_prediction[path_num].accel_derection = angle_deg;

                    predict_robot_fine_path(x, y, theta, Vx, Vy, omega, &fine_prediction[path_num], accel_x, accel_y, fine_path_predict_step);
                    
                    path_num++;
            }
        }
    }
    *fine_path_num = path_num;
}
//DWAで探索された経路の中でも最適経路を評価するための関数(long path用)
void eval_long_path( int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetX, int32_t targetY, int32_t targetTheta, int32_t midle_targetX, int32_t midle_targetY,
                path_long_prediction *long_prediction, uint16_t path_size, int16_t obs_size, obs obs_imfo[], uint32_t obs_avoid_step_size, bool midle_target_flag,
                path_long_prediction *long_opt_path, uint32_t long_path_predict_step){
    //評価結果を格納する配列
    float score_heading_angle[path_size];
    float score_heading_velos[path_size];
    float score_obstacles[path_size];
    //最適化パスがあるかの確認を行うためのフラグ
    int16_t long_opt_path_num = -1;

    //評価計算
    if(midle_target_flag == 0){
        for(int i = 0; i<path_size; i++){
            score_heading_angle[i] = heading_angle(long_prediction[i].x, long_prediction[i].y, long_prediction[i].theta, long_prediction[i].vx, long_prediction[i].vy, long_prediction[i].omega, long_path_predict_step-1, targetX, targetY);
            score_heading_velos[i] = heading_velo(long_prediction[i].x, long_prediction[i].y, long_prediction[i].theta, long_prediction[i].vx, long_prediction[i].vy, long_prediction[i].omega, long_path_predict_step-1, targetX, targetY);
            score_obstacles[i] = obstacle(long_prediction[i].x, long_prediction[i].y, obs_avoid_step_size-1, obs_size, &long_prediction[i].availability_flag, obs_imfo, targetX, targetY);
        }

        //正規化
        min_max_normalize(score_heading_angle, path_size);
        min_max_normalize(score_heading_velos, path_size);
        min_max_normalize(score_obstacles, path_size);

        //scoreが最高値となるpathを算出
        float score = 0;
        int8_t first_flag = 0;
        for(int i = 0; i<path_size; i++){
            if(long_prediction[i].availability_flag == 0){//障害物と衝突していないpathを抽出
                float temp_score = 0;
                temp_score =    LONG_WEIGHT_ANGLE*score_heading_angle[i] + 
                                LONG_WEIGHT_VELO*score_heading_velos[i] + 
                                LONG_WEIGHT_OBS*score_obstacles[i];
                
                if(first_flag == 0){
                    score = temp_score;
                    long_opt_path_num = i;
                    first_flag++;
                }
                
                if(temp_score > score){
                    score = temp_score;
                    long_opt_path_num = i;
                }
            }
        //全てのpathが選択できない場合
        }
    }
    else{
        for(int i = 0; i<path_size; i++){
            long_path_midle_target_score( long_prediction[i].x, long_prediction[i].y, long_prediction[i].theta, long_prediction[i].vx, long_prediction[i].vy, long_prediction[i].omega, 
                                long_path_predict_step-1, midle_targetX, midle_targetY, &score_heading_angle[i], &score_heading_velos[i]);
            score_obstacles[i] = obstacle(long_prediction[i].x, long_prediction[i].y, long_path_predict_step-1, obs_size, &long_prediction[i].availability_flag, obs_imfo, targetX, targetY);
        }
        //正規化
        min_max_normalize(score_heading_angle, path_size);
        min_max_normalize(score_heading_velos, path_size);
        min_max_normalize(score_obstacles, path_size);

        //scoreが最高値となるpathを算出
        float score = 0;
        int8_t first_flag = 0;
        for(int i = 0; i<path_size; i++){
            if(long_prediction[i].availability_flag == 0){//障害物と衝突していないpathを抽出
                float temp_score = 0;
                temp_score =    LONG_WEIGHT_ANGLE_MIDLE_TARGET*score_heading_angle[i] + 
                                LONG_WEIGHT_VELO_MIDLE_TARGET*score_heading_velos[i] + 
                                LONG_WEIGHT_OBS_MIDLE_TARGET*score_obstacles[i];
                
                if(first_flag == 0){
                    score = temp_score;
                    long_opt_path_num = i;
                    first_flag++;
                }
                
                if(temp_score > score){
                    score = temp_score;
                    long_opt_path_num = i;
                }
            }
        //全てのpathが選択できない場合
        }
    }
    if(long_opt_path_num == -1){
        long_opt_path->availability_flag = 1;
        return;
    }

    //最適と評価されたpathを代入
    *long_opt_path = long_prediction[long_opt_path_num];
}
//DWAで探索された経路の中でも最適経路を評価するための関数(fine path用)
void eval_fine_path( int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetX, int32_t targetY, int32_t targetTheta, int32_t midle_targetX, int32_t midle_targetY,
                path_fine_prediction *fine_prediction, uint16_t path_size,
                int16_t obs_size, obs obs_imfo[], uint32_t obs_avoid_step_size, bool midle_target_flag,
                path_fine_prediction *fine_opt_path, uint32_t fine_path_predict_step){
    //評価結果を格納する配列
    float score_heading_angle[path_size];
    float score_heading_velos[path_size];
    float score_f_target_heading_angle[path_size];
    float score_f_target_heading_velos[path_size];
    float score_obstacles[path_size];
    //最適化パスがあるかの確認を行うためのフラグ
    int16_t fine_opt_path_num = -1;
    
    //評価計算
    if(midle_target_flag == 0){
        for(int i = 0; i<path_size; i++){
            score_heading_angle[i] = heading_angle(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1, targetX, targetY);
            score_heading_velos[i] = heading_velo(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1,  targetX, targetY);
            score_obstacles[i] = obstacle(fine_prediction[i].x, fine_prediction[i].y, obs_avoid_step_size-1, obs_size, &fine_prediction[i].availability_flag, obs_imfo, targetX, targetY);
        }
        
        //正規化
        min_max_normalize(score_heading_angle, path_size);
        min_max_normalize(score_heading_velos, path_size);
        min_max_normalize(score_obstacles, path_size);

        //scoreが最高値となるpathを算出
        float score = 0;
        int8_t first_flag = 0;
        for(int i = 0; i<path_size; i++){
            if(fine_prediction[i].availability_flag == 0){//障害物と衝突していないpathを抽出
                float temp_score = 0;
                temp_score =    FINE_WEIGHT_ANGLE*score_heading_angle[i] + 
                                FINE_WEIGHT_VELO*score_heading_velos[i] + 
                                FINE_WEIGHT_OBS*score_obstacles[i];
                
                if(first_flag == 0){
                    score = temp_score;
                    fine_opt_path_num = i;
                    first_flag++;
                }
                
                if(temp_score > score){
                    score = temp_score;
                    fine_opt_path_num = i;
                }
            }
        //全てのpathが選択できない場合
        }
    }
    else{
        for(int i = 0; i<path_size; i++){
            score_heading_angle[i] = heading_angle(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1, midle_targetX, midle_targetY);
            score_heading_velos[i] = heading_velo(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1,  midle_targetX, midle_targetY);
            score_obstacles[i] = obstacle(fine_prediction[i].x, fine_prediction[i].y, fine_path_predict_step-1, obs_size, &fine_prediction[i].availability_flag, obs_imfo, targetX, targetY);
            score_f_target_heading_angle[i] = heading_angle(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1, targetX, targetY);
            score_f_target_heading_velos[i] = heading_velo(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1,  targetX, targetY);
            /*
            fine_path_midle_target_score( fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, 
                                fine_path_predict_step-1, midle_targetX, midle_targetY, &score_heading_angle[i], &score_heading_velos[i]);
            score_f_target_heading_angle[i] = heading_angle(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1, targetX, targetY);
            score_f_target_heading_velos[i] = heading_velo(fine_prediction[i].x, fine_prediction[i].y, fine_prediction[i].theta, fine_prediction[i].vx, fine_prediction[i].vy, fine_prediction[i].omega, fine_path_predict_step-1,  targetX, targetY);
            score_obstacles[i] = obstacle(fine_prediction[i].x, fine_prediction[i].y, fine_path_predict_step-1, obs_size, &fine_prediction[i].availability_flag, obs_imfo);
            */
        }
        
        //正規化
        min_max_normalize(score_heading_angle, path_size);
        min_max_normalize(score_heading_velos, path_size);
        min_max_normalize(score_f_target_heading_angle, path_size);
        min_max_normalize(score_f_target_heading_velos, path_size);
        min_max_normalize(score_obstacles, path_size);


        //scoreが最高値となるpathを算出
        float score = 0;
        int8_t first_flag = 0;
        for(int i = 0; i<path_size; i++){
            if(fine_prediction[i].availability_flag == 0){//障害物と衝突していないpathを抽出
                float temp_score = 0;
                temp_score =    FINE_WEIGHT_ANGLE*score_heading_angle[i] + 
                                FINE_WEIGHT_VELO*score_heading_velos[i] + 
                                FINE_WEIGHT_ANGLE_MIDLE_F_TARGET*score_f_target_heading_angle[i] + 
                                FINE_WEIGHT_VELO_MIDLE_F_TARGET*score_f_target_heading_velos[i] +
                                FINE_WEIGHT_OBS*score_obstacles[i];
                
                if(first_flag == 0){
                    score = temp_score;
                    fine_opt_path_num = i;
                    first_flag++;
                }
                
                if(temp_score > score){
                    score = temp_score;
                    fine_opt_path_num = i;
                }
            }
        }
    }

    //全てのpathが選択できない場合
    if(fine_opt_path_num == -1){
        fine_opt_path->availability_flag = 1;
        return;
    }

    //最適と評価されたpathを代入
    *fine_opt_path = fine_prediction[fine_opt_path_num];
}
//経路が目標値の方向を向いているかを評価
float heading_angle(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], uint16_t size, int32_t targetX, int32_t targetY){
    float last_x = path_x[size];
    float last_y = path_y[size];
    float first_x = path_x[0];
    float first_y = path_y[0];
    float last_th = path_th[size];
    float last_vx = path_vx[size];
    float last_vy = path_vy[size];
    float last_omega = path_omega[size];
    
    //score計算
    //角度計算
    float angle_to_goal = atan2f((float)targetY - last_y, (float)targetX - last_x);
    //ロボットの進んでいる方向
    float robot_drection = atan2f(last_vy, last_vx);
    float score_angle = angle_to_goal - robot_drection;
    //角度を-PIからPIの範囲にクリップ
    score_angle = fabs(angle_range_corrector(score_angle));
    //最大と最小をひっくり返す
    score_angle = M_PI - score_angle;

    return score_angle;
}
//経路が目標値方向の速度を持っているか評価
float heading_velo(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], uint16_t size, int32_t targetX, int32_t targetY){
    float last_x = path_x[size];
    float last_y = path_y[size];
    float last_th = path_th[size];
    float last_vx = path_vx[size];
    float last_vy = path_vy[size];
    float last_omega = path_omega[size];

    //角度計算
    float angle_to_goal = atan2f((float)targetY - last_y, (float)targetX - last_x);
    //ロボットの進んでいる方向
    float robot_drection = atan2f(last_vy, last_vx);
    
    //score計算
    float d_angle = angle_to_goal - robot_drection;

    //角度を-PIからPIの範囲にクリップ
    d_angle = fabs(angle_range_corrector(d_angle));

    //score計算
    float score_heading_velo = sqrt(last_vx*last_vx + last_vy*last_vy)*cosf(d_angle);

    return score_heading_velo;
}
//中間目標点の時の評価方法(long path用)
void long_path_midle_target_score(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], uint16_t size, int32_t targetX, int32_t targetY, float *angle_score, float *velo_score){
    float temp_score_angle[size + 1];
    float temp_score_heading_velo[size + 1];
    float normal_temp_score_angle[size + 1];
    float normal_temp_score_heading_velo[size + 1];
    for(int i = 0; i < size + 1 ; i++){
        //角度計算
        float angle_to_goal = atan2f((float)targetY - path_y[i], (float)targetX - path_x[i]);
        //ロボットの進んでいる方向
        float robot_drection = atan2f(path_vy[i], path_vx[i]);
        float score_angle = angle_to_goal - robot_drection;
        //角度を-PIからPIの範囲にクリップ
        score_angle = fabs(angle_range_corrector(score_angle));
        //最大と最小をひっくり返す
        temp_score_angle[i] = M_PI - score_angle;
        normal_temp_score_angle[i] = temp_score_angle[i];
        temp_score_heading_velo[i] = sqrt(path_vx[i]*path_vx[i] + path_vy[i]*path_vy[i])*cosf(score_angle);
        normal_temp_score_heading_velo[i] = temp_score_heading_velo[i];
    }
    //正規化
    min_max_normalize(normal_temp_score_angle, size+1);
    min_max_normalize(normal_temp_score_heading_velo, size+1);
    //評価が最大となる番号を探索する
    float score = 0;
    for(int i = 0; i < size + 1; i++){
        if(score < LONG_WEIGHT_ANGLE_MIDLE_TARGET * normal_temp_score_angle[i] + LONG_WEIGHT_VELO_MIDLE_TARGET * normal_temp_score_heading_velo[i]){
            score = LONG_WEIGHT_ANGLE_MIDLE_TARGET * normal_temp_score_angle[i] + LONG_WEIGHT_VELO_MIDLE_TARGET * normal_temp_score_heading_velo[i];
            *angle_score = temp_score_angle[i];
            *velo_score = temp_score_heading_velo[i];
        }
    }
}
//中間目標点の時の評価方法(long path用)
void fine_path_midle_target_score(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], uint16_t size, int32_t targetX, int32_t targetY, float *angle_score, float *velo_score){
    float temp_score_angle[size + 1];
    float temp_score_heading_velo[size + 1];
    float normal_temp_score_angle[size + 1];
    float normal_temp_score_heading_velo[size + 1];
    for(int i = 0; i < size + 1 ; i++){
        //角度計算
        float angle_to_goal = atan2f((float)targetY - path_y[i], (float)targetX - path_x[i]);
        //ロボットの進んでいる方向
        float robot_drection = atan2f(path_vy[i], path_vx[i]);
        float score_angle = angle_to_goal - robot_drection;
        //角度を-PIからPIの範囲にクリップ
        score_angle = fabs(angle_range_corrector(score_angle));
        //最大と最小をひっくり返す
        temp_score_angle[i] = M_PI - score_angle;
        normal_temp_score_angle[i] = temp_score_angle[i];
        temp_score_heading_velo[i] = sqrt(path_vx[i]*path_vx[i] + path_vy[i]*path_vy[i])*cosf(score_angle);
        normal_temp_score_heading_velo[i] = temp_score_heading_velo[i];
    }
    //正規化
    min_max_normalize(normal_temp_score_angle, size+1);
    min_max_normalize(normal_temp_score_heading_velo, size+1);
    //評価が最大となる番号を探索する
    float score = 0;
    for(int i = 0; i < size + 1 ; i++){
        if(score < FINE_WEIGHT_ANGLE_MIDLE_TARGET * normal_temp_score_angle[i] + FINE_WEIGHT_VELO_MIDLE_TARGET * normal_temp_score_heading_velo[i]){
            score = FINE_WEIGHT_ANGLE_MIDLE_TARGET * normal_temp_score_angle[i] + FINE_WEIGHT_VELO_MIDLE_TARGET * normal_temp_score_heading_velo[i];
            *angle_score = temp_score_angle[i];
            *velo_score = temp_score_heading_velo[i];
        }
    }
}
//計算する障害物を特定する
int16_t nearest_obs_mun(int32_t x, int32_t y, int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[]){
    //障害物がない場合計算しない
    if(numOfObstacle <= 0){
        return 0;
    }
    //計算すべき障害物の抽出
    int16_t obs_num = 0;
    for(int i = 0; i <numOfObstacle; i++){
        float temp_dis_to_obs = sqrt((x-ObstacleX[i])*(x-ObstacleX[i]) + (y-ObstacleY[i])*(x-ObstacleY[i]));
        if(temp_dis_to_obs < Area_DIS_TO_ODS){
            obs_num++;
        }
    }
    return obs_num;
}
//計算するべき障害物の情報をobs構造体に代入する
void cul_obs(int32_t x, int32_t y, int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[], int32_t ObstacleVX[], int32_t ObstacleVY[], int32_t ObstacleAX[], int32_t ObstacleAY[], int16_t obs_size, obs *obs_imfo){
    if(obs_size == 0){
        return;
    }
    int16_t cal_obs_num = 0;
    for(int i = 0; i <numOfObstacle; i++){
        float temp_dis_to_obs = sqrt((x-ObstacleX[i])*(x-ObstacleX[i]) + (y-ObstacleY[i])*(x-ObstacleY[i]));
        if(temp_dis_to_obs < Area_DIS_TO_ODS){
            obs_imfo[cal_obs_num].x = ObstacleX[i];
            obs_imfo[cal_obs_num].y = ObstacleY[i];
            obs_imfo[cal_obs_num].vx = ObstacleVX[i];
            obs_imfo[cal_obs_num].vy = ObstacleVY[i];
            obs_imfo[cal_obs_num].ax = ObstacleAX[i];
            obs_imfo[cal_obs_num].ay = ObstacleAY[i];
            obs_imfo[cal_obs_num].radius = ROBOT_SIZE/2;
            cal_obs_num++;
            if(obs_size <= cal_obs_num){
                break;
            }
        }
    }
}
//障害物の回避について評価
float obstacle(float path_x[], float path_y[], uint16_t size, int16_t obs_size, int8_t *flag, obs obs_imfo[], int32_t targetX, int32_t targetY){
    float score_obstacle = Area_DIS_TO_ODS;
    if(obs_size == 0){
        return score_obstacle;
    }
    float temp_dis_to_obs = 0.0;
    int8_t finish_flag = 0;
    float robot_target_dis = 0.0;
    if((path_x[0]-targetX)*(path_x[0]-targetX)+(path_y[0]-targetY)*(path_y[0]-targetY) != 0){
        robot_target_dis = sqrt((path_x[0]-targetX)*(path_x[0]-targetX)+(path_y[0]-targetY)*(path_y[0]-targetY));
        for(int i= 0; i<size+1; i++){
            float target_predict_dis = 0;
            if((path_x[i]-targetX)*(path_x[i]-targetX) + (path_y[i]-targetY)*(path_y[i]-targetY) != 0){
                target_predict_dis = sqrt((path_x[i]-targetX)*(path_x[i]-targetX) + (path_y[i]-targetY)*(path_y[i]-targetY));
            }
            float deg_robto_target_predit = 0.0;
            if(target_predict_dis != 0){
                deg_robto_target_predit = modifid_acosf(((path_x[0]-targetX)*(path_x[i]-targetX) + (path_y[0]-targetY)*(path_y[i]-targetY))/(robot_target_dis * target_predict_dis));
            }
            if(deg_robto_target_predit < M_PI/2){
                for(int j= 0; j<obs_size; j++){
                    //障害物のロボットの位置計算
                    float obs_x = obs_imfo[j].x + obs_imfo[j].vx*TIME_STEP*i + obs_imfo[j].ax*TIME_STEP*i*TIME_STEP*i;
                    float obs_y = obs_imfo[j].y + obs_imfo[j].vy*TIME_STEP*i + obs_imfo[j].ay*TIME_STEP*i*TIME_STEP*i;
                    //ロボットのpathと障害物の距離計算
                    temp_dis_to_obs = sqrt((path_x[i] - obs_x)*(path_x[i] - obs_x) + (path_y[i] - obs_y)*(path_y[i] - obs_y));
                    //pathの中で障害物と最も近くなる際の距離を計算
                    if(temp_dis_to_obs < score_obstacle){
                        score_obstacle = temp_dis_to_obs;
                        //pathが障害物に突入している場合
                        if(score_obstacle < obs_imfo[j].radius + ROBOT_SIZE/2 + MARGIN_DISTANCE && i != 0){
                            score_obstacle = 0.0;
                            finish_flag = 1;
                            *flag = 1;
                            break;
                        }
                    }
                    if(i == 0){
                        if(temp_dis_to_obs < obs_imfo[j].radius + ROBOT_SIZE/2 + MARGIN_DISTANCE){
                            obs_imfo[j].radius = temp_dis_to_obs - ROBOT_SIZE/2 - MARGIN_DISTANCE;
                        }
                    }
                }
            }
            if(finish_flag == 1){
                break;
            }
        }
    }
    //score_obstacle = score_obstacle - (ROBOT_SIZE + MARGIN_DISTANCE);
    return score_obstacle;
}
//DWAのパスを使うべきか否かの判定
bool path_enable_check( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t output_vx, int32_t output_vy, int32_t output_ax, int32_t output_ay, obs obs_imfo[], int16_t obs_size,
                        uint32_t fine_path_predict_step, bool prohidited_zone_ignore, bool midle_target_flag, bool ollision_flag, bool pre_path_enable_flag, bool prohidited_zone_start){
    bool path_enable = 1;
    float distance = (targetX - x)*(targetX - x) + (targetY - y)*(targetY - y);
    float velocity = output_vx * output_vx + output_vy * output_vy;
    float accel = output_ax * output_ax + output_ay * output_ay;
    float predict_distance = (2*sqrt(velocity) - sqrt(accel)*(float)fine_path_predict_step/10)/(2*(float)fine_path_predict_step/10);
    if(distance != 0 && accel != 0){
        float dwa_check_angle = ((targetX - x) * output_ax + (targetY - y) * output_ay)/(sqrt(distance)* sqrt(accel));
        float vel_accel_diff_deg = 180 / M_PI * angle_range_corrector(modifid_acosf(dwa_check_angle));
        if(-GNORE_ANGLE < vel_accel_diff_deg && vel_accel_diff_deg < GNORE_ANGLE){//加速度が目標値方向を向いている時は台形制御
            path_enable = 0;
        }
        else if((vel_accel_diff_deg < -180 + GNORE_ANGLE2 || 180 - GNORE_ANGLE2 < vel_accel_diff_deg) && sqrt(distance) < predict_distance){//目標値付近で目標値と反対方向に加速度がある時は台形制御
            path_enable = 0;
        }
        else{
            path_enable = 1;
        }
    }
    else{
        path_enable = 1;
    }
    velocity = sqrt(velocity);
    float deg_change = -(velocity/1000) * 5 + 5;
    if(deg_change < 2){
        deg_change = 2;
    }
    if(velocity !=0 && distance != 0 && path_enable == 1){
        float dwa_check_angle = (output_vx * (targetX - x) + output_vy * (targetY - y))/(velocity * sqrt(distance));
        float vel_accel_diff_deg = 180 / M_PI * angle_range_corrector(modifid_acosf(dwa_check_angle));
        //速度が目標値方向に向いている場合は台形制御
        if((-deg_change <= vel_accel_diff_deg && vel_accel_diff_deg <= deg_change) || vel_accel_diff_deg <= -180 + deg_change || 180 - deg_change <= vel_accel_diff_deg){
            path_enable = 0;
        }
        else{
            path_enable = 1;
        }
    }
    //中間目標点の場合，壁に衝突する場合は台形制御をしない
    if(midle_target_flag == 1 || ollision_flag == 1){
        path_enable = 1;
    }
    //ここは改造の必要あり
    //目標値と現在地の間にペナルティゾーンがあるときは台形制御をしない
    else if(distance != 0){
        for(int i = 10; i < sqrt(distance); i += 10){
            float x_check = x + (targetX - x)/sqrt(distance)*i;
            float y_check = y + (targetY - y)/sqrt(distance)*i;
            if(field_penalty_zone_check(x_check, y_check, prohidited_zone_ignore, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus) == 1){//ペナルティゾーンに直進していないかを判定
                path_enable = 1;
                break;
            }
        }
    }
    else if(Vx*Vx + Vy*Vy <= 20){
        path_enable = 0;
    }

    //障害物(他のロボット)と衝突する場合は台形制御しない
    if(path_enable == 0 && 750 < velocity){
        trape_con trape_c;
        trapezoidal_init(&trape_c);
        trapezoidal_DWA_change(x, y, Vx, Vy, &trape_c, targetX, targetY, (float)output_ax, (float)output_ay);
        float *pre_obs_robot_dis = (float *)malloc(obs_size * sizeof(float));
        bool *cul_flag = (bool *)malloc(obs_size * sizeof(bool));
        //printf("%f,%f\n",trape_c.virtual_x, trape_c.virtual_y);
        bool finish_flag = 0;
        for(int i = 0; i < obs_size; i++){
            pre_obs_robot_dis[i] = Area_DIS_TO_ODS;
            cul_flag[i] = 1;
        }
        for(int i = 0; i < 200; i++){//2秒間の台形制御の軌道を推定し他のロボットと衝突しないか確認する
            trapezoidal_control(targetX, targetY, &trape_c);//time stepは100ms
            for(int k = 0; k < obs_size; k++){
                if(cul_flag[k] == 1){
                    float obs_x = obs_imfo[k].x + obs_imfo[k].vx*TRAP_TIME_STEP*i + 1/2*obs_imfo[k].ax*TRAP_TIME_STEP*i*TRAP_TIME_STEP*i;
                    float obs_y = obs_imfo[k].y + obs_imfo[k].vy*TRAP_TIME_STEP*i + 1/2*obs_imfo[k].ay*TRAP_TIME_STEP*i*TRAP_TIME_STEP*i;
                    float obs_robot_dis = 0;
                    if((obs_x-trape_c.virtual_x)*(obs_x-trape_c.virtual_x) + (obs_y-trape_c.virtual_y)*(obs_y-trape_c.virtual_y) != 0){
                        obs_robot_dis = sqrt((obs_x-trape_c.virtual_x)*(obs_x-trape_c.virtual_x) + (obs_y-trape_c.virtual_y)*(obs_y-trape_c.virtual_y));
                    }
                    if(obs_robot_dis < pre_obs_robot_dis[k]){
                        pre_obs_robot_dis[k] = obs_robot_dis;
                        if(pre_obs_robot_dis[k] <= (ROBOT_SIZE + COLLISION_CHECK_MARGIN_DISTANCE)){
                            path_enable = 1;
                            finish_flag = 1;
                            break;
                        }
                    }
                    else{
                        cul_flag[k] = 0;
                    }
                    //printf("%d,%f,%d,%f\n",obs_imfo[k].x,trape_c.virtual_x,obs_imfo[k].y,trape_c.virtual_y);
                    //printf("obs_robot_dis,%f\n",obs_robot_dis);
                }
            }
            if(finish_flag == 1){
                break;
            }
        }
        free(pre_obs_robot_dis);
        free(cul_flag);
    }
    if(path_enable == 1 && midle_target_flag == 0){
        if(distance < 90 && pre_path_enable_flag == 0){  //目標値と現在地が近く台形制御に入っている場合
            path_enable = 0;
        }
        else{
            path_enable = 1;
        }
    }
    if(prohidited_zone_start == 1){
        path_enable = 1;
    }
    //中間目標地点の場合は台形制御を使用しない
    if(midle_target_flag == 1){
        path_enable = 1;
    }
    return path_enable;
}
//ペナルティゾーンへの侵入やフィールド外に行くパスを確認してフラグを付与する(long path用)
void field_out_penalty_zone_long_path_check(path_long_prediction *long_prediction, uint16_t path_size ,uint32_t long_path_predict_step, bool prohidited_zone_start, bool prohidited_zone_ignore){
    /*
    if(in_prohidited_zone_flag == 1){//パスプランニングスタート時に侵入禁止エリアに入っている時はこの関数を起動すると全てのパスが利用不可となるので、これを回避
        return;
    }
    */
    for(int i = 0; i < path_size; i++){
        for(int k = 0; k < long_path_predict_step; k++){
            if(field_penalty_zone_check(long_prediction[i].x[k], long_prediction[i].y[k], prohidited_zone_ignore, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus) == 1 && prohidited_zone_start == 0){
                long_prediction[i].availability_flag = 1;
                break;
            }
        }
    }
}
//ペナルティゾーンへの侵入やフィールド外に行くパスを確認してフラグを付与する(fine path用)
void field_out_penalty_zone_fine_path_check(path_fine_prediction *fine_prediction, uint16_t path_size ,uint32_t fine_path_predict_step, bool prohidited_zone_start, bool prohidited_zone_ignore){
    /*
    if(in_prohidited_zone_flag == 1){//パスプランニングスタート時に侵入禁止エリアに入っている時はこの関数を起動すると全てのパスが利用不可となるので、これを回避
        return;
    }
    */
    for(int i = 0; i < path_size; i++){
        for(int k = 0; k < fine_path_predict_step; k++){
            if(field_penalty_zone_check(fine_prediction[i].x[k], fine_prediction[i].y[k], prohidited_zone_ignore, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus) == 1 && prohidited_zone_start == 0){
                fine_prediction[i].availability_flag = 1;
                break;
            }
        }
    }
}
//DWAの使用可能性確認
bool DWA_enable_check(int8_t long_path_availability_flag, int8_t fine_path_availability_flag){
    if(long_path_availability_flag == 1){
        return 0;
    }
    else if(fine_path_availability_flag == 1){
        return 0;
    }
    else{
        return 1;
    }
}
//制動距離を算出して壁に激突しないか判定する
void avoid_field_wall_ollision(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, path_fine_prediction *opt_path, uint32_t fine_path_predict_step, bool *ollision_flag){
    float x_stop = (float)x + *opt_path->vx * *opt_path->vx * *opt_path->vx/(2 * robot_max_accel * fabs(*opt_path->vx));
    float y_stop = (float)y + *opt_path->vy * *opt_path->vy * *opt_path->vy/(2 * robot_max_accel * fabs(*opt_path->vy));
    if(x_stop <= x_outside_line_minus - FIELD_OUT_LINE + ROBOT_SIZE/2 + FIELD_MARGIN){
        opt_path->accel_derection = 0;
        opt_path->accel = robot_max_accel;
        *ollision_flag = 1;
    }
    else if(x_outside_line_puls + FIELD_OUT_LINE - ROBOT_SIZE/2 - FIELD_MARGIN <= x_stop){
        opt_path->accel_derection = 180;
        opt_path->accel = robot_max_accel;
        *ollision_flag = 1;
    }
    if(y_stop <= y_outside_line_minus - FIELD_OUT_LINE + ROBOT_SIZE/2 + FIELD_MARGIN){
        opt_path->accel_derection = 90;
        opt_path->accel = robot_max_accel;
        *ollision_flag = 1;
    }
    else if(y_outside_line_puls + FIELD_OUT_LINE - ROBOT_SIZE/2 - FIELD_MARGIN <= y_stop){
        opt_path->accel_derection = 270;
        opt_path->accel = robot_max_accel;
        *ollision_flag = 1;
    }
    float angle_deg = angle_range_corrector_deg(opt_path->accel_derection);
    float angle = angle_deg*M_PI/180;
    angle = angle_range_corrector(angle);
    float accel_x = opt_path->accel*cosf(angle);
    float accel_y = opt_path->accel*sinf(angle);
    predict_robot_fine_path(x, y, theta, Vx, Vy, omega, opt_path, accel_x, accel_y, fine_path_predict_step);
}
//中間目標点が設置されている時のロボットの速度制限(long path用)
void midle_target_velo_limit_long_path( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, 
                                        path_long_prediction *long_prediction, uint16_t path_size ,uint32_t long_path_predict_step, bool midle_target_flag){
    //中間目標点が設定されていない場合はこの関数は必要ない
    if(midle_target_flag == 0){
        return;
    }

    float midle_target_target_dis;
    if((midle_targetX-targetX)*(midle_targetX-targetX) + (midle_targetY-targetY)*(midle_targetY-targetY) != 0){
        midle_target_target_dis = sqrt((midle_targetX-targetX)*(midle_targetX-targetX) + (midle_targetY-targetY)*(midle_targetY-targetY));
    }
    else{
        midle_target_target_dis = 0;
    }
    float dis;
    if((x-midle_targetX)*(x-midle_targetX) + (y-midle_targetY)*(y-midle_targetY) != 0){
        dis = sqrt((x-midle_targetX)*(x-midle_targetX) + (y-midle_targetY)*(y-midle_targetY));
    }
    else{
        dis = 0;
    }
    float angle_mt_robot_to_mt_t_cos = 0;
    if(dis != 0 && midle_target_target_dis != 0){
        angle_mt_robot_to_mt_t_cos = ((midle_targetX-x)*(targetX-midle_targetX)+(midle_targetY-y)*(targetY-midle_targetY))/(midle_target_target_dis*dis);
    }
    float midle_target_velo_ajust_accel = 1000;
    float robot_velo_limit = sqrt(2*midle_target_velo_ajust_accel*(dis+midle_target_target_dis*angle_mt_robot_to_mt_t_cos*angle_mt_robot_to_mt_t_cos));
    if(robot_velo_limit < 300){
        robot_velo_limit = 300;
    }
    if(robot_max_velo < robot_velo_limit){
        robot_velo_limit = robot_max_velo;
    }
    //robot_velo_limit:中間目標点が設定されている時のロボットの速度制限
    float robot_velo;
    if(Vx*Vx + Vy*Vy != 0){
        robot_velo = sqrt(Vx*Vx + Vy*Vy);
    }
    else{
        robot_velo = 0;
    }
    if(robot_velo_limit < robot_velo){
        //ロボットを減速させるための処理
        //角度計算
        float robot_decele_angle = angle_range_corrector(atan2f((float)Vy, (float)Vx) + M_PI);
        float robot_decele_angle_min = robot_decele_angle - M_PI/4;
        float robot_decele_angle_max = robot_decele_angle + M_PI/4;
        for(int i = 0; i < path_size; i++){
            if(angele_check(M_PI*long_prediction[i].accel_derection/180, robot_decele_angle_max, robot_decele_angle_min) == 0){
                long_prediction[i].availability_flag = 1;
            }
        }
    }
}
//中間目標点が設置されている時のロボットの速度制限(fine path用)
void midle_target_velo_limit_fine_path( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, 
                                        path_fine_prediction *fine_prediction, uint16_t path_size ,uint32_t fine_path_predict_step, bool midle_target_flag){
    //中間目標点が設定されていない場合はこの関数は必要ない
    if(midle_target_flag == 0){
        return;
    }

    float midle_target_target_dis;
    if((midle_targetX-targetX)*(midle_targetX-targetX) + (midle_targetY-targetY)*(midle_targetY-targetY) != 0){
        midle_target_target_dis = sqrt((midle_targetX-targetX)*(midle_targetX-targetX) + (midle_targetY-targetY)*(midle_targetY-targetY));
    }
    else{
        midle_target_target_dis = 0;
    }
    float dis;
    if((x-midle_targetX)*(x-midle_targetX) + (y-midle_targetY)*(y-midle_targetY) != 0){
        dis = sqrt((x-midle_targetX)*(x-midle_targetX) + (y-midle_targetY)*(y-midle_targetY));
    }
    else{
        dis = 0;
    }
    float angle_mt_robot_to_mt_t_cos = 0;
    if(dis != 0 && midle_target_target_dis != 0){
        angle_mt_robot_to_mt_t_cos = ((midle_targetX-x)*(targetX-midle_targetX)+(midle_targetY-y)*(targetY-midle_targetY))/(midle_target_target_dis*dis);
    }
    float midle_target_velo_ajust_accel = 1000;
    float robot_velo_limit = sqrt(2*midle_target_velo_ajust_accel*(dis+midle_target_target_dis*angle_mt_robot_to_mt_t_cos*angle_mt_robot_to_mt_t_cos));
    if(robot_max_velo < robot_velo_limit){
        robot_velo_limit = robot_max_velo;
    }
    //robot_velo_limit:中間目標点が設定されている時のロボットの速度制限
    float robot_velo;
    if(Vx*Vx + Vy*Vy != 0){
        robot_velo = sqrt(Vx*Vx + Vy*Vy);
    }
    else{
        robot_velo = 0;
    }
    if(robot_velo_limit < robot_velo){
        //ロボットを減速させるための処理
        //角度計算
        float robot_decele_angle = angle_range_corrector(atan2f((float)Vy, (float)Vx) + M_PI);
        float robot_decele_angle_min = robot_decele_angle - M_PI/4;
        float robot_decele_angle_max = robot_decele_angle + M_PI/4;
        for(int i = 0; i < path_size; i++){
            if(angele_check(M_PI*fine_prediction[i].accel_derection/180, robot_decele_angle_max, robot_decele_angle_min) == 0){
                fine_prediction[i].availability_flag = 1;
            }
        }
    }
}