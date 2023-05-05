#ifndef _TRAPEZOIDALCONTROL_H_
#define _TRAPEZOIDALCONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif
//構造体
//ロボットの状態を保存する構造体
/*テスト用の構造体
typedef struct{
    int32_t x;
    int32_t y;
    int32_t theta;
    int32_t Vx;
    int32_t Vy;
    int32_t omega;
    int32_t targetX;
    int32_t targetY;
    int32_t targetTheta;
} robot_state;
*/
//台形制御を行うための変数を保存する構造体
typedef struct{
    float jerk;             //(mm/s^3)
    float accel;            //(mm/s^2)
    float max_accel;        //(mm/s^2)
    float de_max_accel;         //減速時の(mm/s^2)
    float velocity;         //(mm/s)
    float max_velocity;     //(mm/s)
    float unit_vector_x;               //(mm/s)
    float unit_vector_y;               //(mm/s)
    float virtual_x;         //(mm)
    float virtual_y;         //(mm)
} micon_trape_con;

// 関数のプロトタイプ宣言
//DWAと台形制御の切り替え判定 0:DWAを使用,1:台形制御を使用
bool micon_trapezoidal_DWA_change(int32_t x, int32_t y, int32_t vx, int32_t vy, micon_trape_con *trape_c, int32_t targetX, int32_t targetY, 
                                    bool trape_c_flag, bool dwa_enable_flag, bool dwa_pathe_enable_flag);
int8_t micon_jerk_flag_check(float distance, float braking_distance);
//台形制御
void micon_trapezoidal_control(int32_t targetX, int32_t targetY, micon_trape_con *trape_c);
//台形制御の係数初期化
void micon_trapezoidal_init(micon_trape_con *trape_c);
//ドリブルのための台形制御の係数初期化
void dribble_micon_trapezoidal_init(micon_trape_con *trape_c);
//台形制御時にロボットと仮想目標値が離れすぎた場合に補正するための関数
void micon_trapezoidal_robotXY_vertualXY_distance_check(micon_trape_con *trape_c, int32_t x, int32_t y);
//ドリブルのための台形制御
void micon_trapezoidal_dribble_control(int32_t targetX, int32_t targetY, micon_trape_con *trape_c, int32_t w_ball_x, int32_t w_ball_y);

#ifdef __cplusplus
}
#endif
#endif // _TRAPEZOIDALCONTROL_H_