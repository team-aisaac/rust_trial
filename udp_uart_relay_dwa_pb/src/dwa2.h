#ifndef _DWA_H_
#define _DWA_H_

#define DWA_CAL_TIME_STEP 0.1          //cdwa ontroll time step
#define TIME_STEP_RECIPROCAL 10          //reciprocal cdwa ontroll time step


//デフォルト
#define PREDICT_LONG_TIME 2.0     //(s)
#define PREDICT_LONG_STEP 20  //PREDICT_LONG_TIME*1/TIME_STEP
#define PREDICT_FINE_TIME 2.0  //(s)
#define PREDICT_FINE_STEP 20   //PREDICT_FINE_TIME*1/TIME_STEP

#define Area_DIS_TO_ODS 4000    //(mm)distance to obstacles to consider
#define MARGIN_DISTANCE 100      //Margin distance when avoiding obstacles
#define OBS_EMERGENCY_MARGIN_DISTANCE 10      //Margin distance when emergency avoiding obstacles

#define COLLISION_AVOID_PREDICT_SECOND 2.0  //(s) predict second for avoiding collision

#define ONE_ROUND_DEG 360   //探索時の粗目の時の探索角度(deg)
#define DELTA_BIG_DEG 8      //探索時の粗目の時の刻み幅(deg)
#define DELTA_BIG_ACCEL 500  //探索時の粗目の時の刻み幅(mm/s^2)
#define DELTA_FINE_DEG 0.5  //探索時の細かめの時の刻み幅(deg)
#define DELTA_FINE_ACCEL 250 //探索時の細かめの時の刻み幅(mm/s^2)

//path評価の重みづけ//デフォルト
#define LONG_WEIGHT_ANGLE 0.04  //探索時の粗目の時の角度の重み
#define LONG_WEIGHT_VELO 0.25    //探索時の粗目の時の速度の重み
#define LONG_WEIGHT_OBS 0.3    //探索時の粗目の時の障害物回避の重み
#define LONG_WEIGHT_ANGLE_MIDLE_TARGET 0.1  //探索時の粗目の時の角度の重み
#define LONG_WEIGHT_VELO_MIDLE_TARGET 0.2    //探索時の粗目の時の速度の重み
#define LONG_WEIGHT_OBS_MIDLE_TARGET 0.3    //探索時の粗目の時の障害物回避の重み
#define FINE_WEIGHT_ANGLE 0.5   //探索時の細かめの時の角度の重み
#define FINE_WEIGHT_VELO 0.2    //探索時の細かめの時の速度の重み
#define FINE_WEIGHT_OBS 0.3     //探索時の細かめの時の障害物回避の重み
#define FINE_WEIGHT_ANGLE_MIDLE_TARGET 0.5   //探索時の細かめの時の中間目標点に向かう角度の重み
#define FINE_WEIGHT_VELO_MIDLE_TARGET 0.2    //探索時の細かめの時の中間目標点に向かう速度の重み
#define FINE_WEIGHT_ANGLE_MIDLE_F_TARGET 0.2   //探索時の細かめの時の最終目標点に向かう角度の重み
#define FINE_WEIGHT_VELO_MIDLE_F_TARGET 0.1    //探索時の細かめの時の最終目標点に向かう速度の重み
#define FINE_WEIGHT_OBS_MIDLE_TARGET 0.3     //探索時の細かめの時の障害物回避の重み

//DWAを使用しない速度と加速度の成す角度(直線的な躍度制御を利用する)
#define GNORE_ANGLE 5   //(deg)
#define GNORE_ANGLE2 10   //(deg)

#ifdef __cplusplus
extern "C" {
#endif
//構造体
//このプログラムが受け取ったロボットの状態を保存する構造体
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
} dwa_robot_state;
//1度目の長い時間を予測する時のpath
typedef struct{
	float x[PREDICT_LONG_STEP];		//(mm)
	float y[PREDICT_LONG_STEP];		//(mm)
    float theta[PREDICT_LONG_STEP];  //(deg)
    float vx[PREDICT_LONG_STEP];    //(mm/s)
    float vy[PREDICT_LONG_STEP];    //(mm/s)
    float omega[PREDICT_LONG_STEP];  //(deg/s)
    float accel;       //(mm/s^2)
    float accel_derection;  //(0~360 deg)
    int8_t availability_flag;   //0:このpahtは利用可能,1:このpathは利用不可能
} path_long_prediction;
//2度目の短い時間で詳細を予測する時のpath
typedef struct{
	float x[PREDICT_FINE_STEP];		//(mm)
	float y[PREDICT_FINE_STEP];		//(mm)
	float theta[PREDICT_FINE_STEP];	//(deg)
    float vx[PREDICT_FINE_STEP];    //(mm/s)
    float vy[PREDICT_FINE_STEP];    //(mm/s)
    float omega[PREDICT_FINE_STEP];  //(deg/s)
    float accel;       //(mm/s^2)
    float accel_derection;  //(deg)
    int8_t availability_flag;   //0：このpahtは利用可能,1:このpathは利用不可能
} path_fine_prediction;
//評価時に計算する障害物
typedef struct{
    int32_t x;      //(mm)
    int32_t y;      //(mm)
    int32_t vx;     //(mm/s)
    int32_t vy;     //(mm/s)
    int32_t ax;     //(mm/s^2)
    int32_t ay;     //(mm/s^2)
    int32_t radius; //(mm)
} obs;

//  関数avgのプロトタイプ宣言
//DWAを実行するための関数
void execDWA(   int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t *targetX, int32_t *targetY, int32_t targetTheta, int32_t *midle_targetX, int32_t *midle_targetY,
                int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[], int32_t ObstacleVX[], int32_t ObstacleVY[], int32_t ObstacleaX[], int32_t ObstacleaY[], bool prohidited_zone_ignore, bool *midle_target_flag,
                bool *is_enable, bool *path_enable, int32_t *output_x, int32_t *output_y, int32_t *output_omega, int32_t *output_ax, int32_t *output_ay);
//ロボットと目標点までの距離を計算する
float cal_robot_target_distance(int32_t x, int32_t y, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, bool midle_target_flag);
//ロボットの軌道予測(long path用)
void predict_robot_long_path(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, 
                            path_long_prediction *long_prediction, float accel_x, float accel_y, uint32_t long_path_predict_step);
//ロボットの軌道予測(fine path用)
void predict_robot_fine_path(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, 
                            path_fine_prediction *fine_prediction, float accel_x, float accel_y, uint32_t fine_path_predict_step);

//DWAで探索する時間とステップ数を算出する
void dwa_predict_step_cal(uint32_t *long_path_predict_step, uint32_t *fine_path_predict_step, float distance);
//障害物との衝突を回避するためにDWAで探索する時間とステップを算出する
uint32_t dwa_cllision_avoid_step_cal();
//DWAで全体方位をざっくり計算するために経路を検索する関数
void make_path_long(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetTheta,
                    path_long_prediction *long_prediction, float midle_accel_value, float accel_range, float accel_resolution,
                    float midle_deg_value, float deg_range, float deg_resolution, uint16_t *long_path_num, uint32_t long_path_predict_step);
//DWAで全体方位を計算した後に、詳細を検討するために検索する関数
void make_path_fine(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetTheta,
                    path_fine_prediction *fine_prediction, float midle_accel_value, float accel_range, float accel_resolution,
                    float midle_deg_value, float deg_range, float deg_resolution, uint16_t *long_path_num, uint32_t fine_path_predict_step);
//DWAで探索された経路の中でも最適経路を評価するための関数(long path用)
void eval_long_path( int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetX, int32_t targetY, int32_t targetTheta, int32_t midle_targetX, int32_t midle_targetY,
                path_long_prediction *long_prediction, uint16_t path_size,
                int16_t obs_size, obs obs_imfo[], uint32_t obs_avoid_step_size, bool midle_target_flag,
                path_long_prediction *long_opt_path, uint32_t long_path_predict_step);
//DWAで探索された経路の中でも最適経路を評価するための関数(fine path用)
void eval_fine_path( int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, int32_t targetX, int32_t targetY, int32_t targetTheta, int32_t midle_targetX, int32_t midle_targetY,
                path_fine_prediction *fine_prediction, uint16_t path_size,
                int16_t obs_size, obs obs_imfo[], uint32_t obs_avoid_step_size, bool midle_target_flag,
                path_fine_prediction *fine_opt_path, uint32_t fine_path_predict_step);
//侵入禁止ゾーンや侵入負荷エリア(フィールド範囲外等)に入らないかを計算
//経路が目標値の方向を向いているかを評価
float heading_angle(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], int8_t flag, uint16_t size,  int32_t targetX, int32_t targetY);
//経路が目標値方向の速度を持っているか評価
float heading_velo(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], int8_t flag, uint16_t size,  int32_t targetX, int32_t targetY);
//中間目標点の時の評価方法(long path用)
void long_path_midle_target_score(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], int8_t flag, uint16_t size,  int32_t targetX, int32_t targetY, float *angle_score, float *velo_score);
//中間目標点の時の評価方法(long path用)
void fine_path_midle_target_score(float path_x[], float path_y[], float path_th[], float path_vx[], float path_vy[], float path_omega[], int8_t flag, uint16_t size,  int32_t targetX, int32_t targetY, float *angle_score, float *velo_score);
//計算する障害物の数を計算する
int16_t nearest_obs_mun(int32_t x, int32_t y, int32_t vx, int32_t vy, int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[]);
//計算するべき障害物の情報をobs構造体に代入する
void cul_obs(int32_t x, int32_t y, int32_t vx, int32_t vy, int32_t numOfObstacle, int32_t ObstacleX[], int32_t ObstacleY[], int32_t ObstacleVX[], int32_t ObstacleVY[], int32_t ObstacleAX[], int32_t ObstacleAY[], int16_t obs_size, obs *obs_imfo);
//障害物の回避について評価
float obstacle(float path_x[], float path_y[], uint16_t size, int16_t obs_size, int8_t *flag, obs obs_imfo[], int32_t targetX, int32_t targetY);
//DWAのパスを使うべきか否かの判定
bool path_enable_check( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t output_vx, int32_t output_vy, int32_t output_ax, int32_t output_ay, obs obs_imfo[], int16_t obs_size,
                        uint32_t fine_path_predict_step, bool prohidited_zone_ignore, bool midle_target_flag, bool collision_flag, bool pre_path_enable_flag, bool prohidited_zone_start);
//ペナルティゾーンへの侵入確認 0:パスを使用可能，1:パスがペナルティゾーンに入っている
//bool field_penalty_zone_check(float x, float y, bool prohidited_zone_ignore);
//ペナルティゾーンへの侵入しているパスやフィールド外に行くパスを利用不可能にする(long path用)
void field_out_penalty_zone_long_path_check(path_long_prediction *long_prediction, uint16_t path_size ,uint32_t long_path_predict_step, bool prohidited_zone_start, bool prohidited_zone_ignore);
//ペナルティゾーンへの侵入確認しているパスやフィールド外に行くパスを利用不可能にする(fine path用)
void field_out_penalty_zone_fine_path_check(path_fine_prediction *fine_prediction, uint16_t path_size ,uint32_t fine_path_predict_step, bool prohidited_zone_start, bool prohidited_zone_ignore);
//DWAの使用可能性確認
bool DWA_enable_check(int8_t long_path_availability_flag, int8_t fine_path_availability_flag);
//制動距離を算出して壁に激突しないか判定する
void avoid_field_wall_collision(int32_t x, int32_t y, int32_t theta, int32_t Vx, int32_t Vy, int32_t omega, path_fine_prediction *opt_path, uint32_t fine_path_predict_step, bool *collision_flag);
//障害物が至近距離がある場合回避する
void avoid_obstacle_collision(int32_t x, int32_t y, int16_t obs_size, obs obs_imfo[], path_fine_prediction *opt_path, uint32_t fine_path_predict_step, bool *collision_flag);
//中間目標点が設置されている時のロボットの速度制限(long path)
void midle_target_velo_limit_long_path( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, 
                                        path_long_prediction *long_prediction, uint16_t path_size ,uint32_t long_path_predict_step, bool midle_target_flag);
//中間目標点が設置されている時のロボットの速度制限(fine path)
void midle_target_velo_limit_fine_path( int32_t x, int32_t y, int32_t Vx, int32_t Vy, int32_t targetX, int32_t targetY, int32_t midle_targetX, int32_t midle_targetY, 
                                        path_fine_prediction *fine_prediction, uint16_t path_size ,uint32_t fine_path_predict_step, bool midle_target_flag);
#ifdef __cplusplus
}
#endif
#endif // _DWA_H_