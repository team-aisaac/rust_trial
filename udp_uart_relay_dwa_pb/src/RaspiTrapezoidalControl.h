#ifndef _RSPI_TRAPEZOIDALCONTROL_H_
#define _RSPI_TRAPEZOIDALCONTROL_H_

#define TRAP_TIME_STEP 0.04     //execDWA時に台形制御かDWAかを判断するために使用する

#ifdef __cplusplus
extern "C" {
#endif
//構造体
//ロボットの状態を保存する構造体
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
} trap_robot_state;
//台形制御を行うための変数を保存する構造体
typedef struct{
    float jerk;             //(mm/s^3)
    float accel;            //(mm/s^2)
    float max_accel;        //(mm/s^2)
    float velocity;         //(mm/s)
    float max_velocity;     //(mm/s)
    float unit_vector_x;               //(mm/s)
    float unit_vector_y;               //(mm/s)
    float virtual_x;         //(mm)
    float virtual_y;         //(mm)
} trape_con;

// 関数のプロトタイプ宣言
//DWAと台形制御の切り替え判定 0:DWAを使用,1:台形制御を使用
bool trapezoidal_DWA_change(int32_t x, int32_t y, int32_t vx, int32_t vy, trape_con *trape_c, int32_t targetX, int32_t targetY, float ax, float ay);
//台形制御
void trapezoidal_control(int32_t targetX, int32_t targetY, trape_con *trape_c);
//台形制御の係数初期化
void trapezoidal_init(trape_con *trape_c);

#ifdef __cplusplus
}
#endif
#endif // _RSPI_TRAPEZOIDALCONTROL_H_