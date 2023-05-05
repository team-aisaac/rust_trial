#ifndef _TOOLS_H_
#define _TOOLS_H_

#define ROBOT_SIZE 180            //robot size(diameter mm)
#define BALL_DIAMETER 46          //ball diameter(mm)

#define DEG_TO_RAD 0.0174532925199
#define RAD_TO_DEG 57.29577951

//フィールド選択
#define FIELD "A" //  フィールドの選択A or B
//フィールドAのペナルティゾーンの範囲
#define PENALTY_ZONE_X_A 1800
#define PENALTY_ZONE_Y_A 3600
#define FIELD_X_A 12600
#define FIELD_Y_A 9600
//フィールドBのペナルティゾーンの範囲
#define PENALTY_ZONE_X_B 1000
#define PENALTY_ZONE_Y_B 2000
#define FIELD_X_B 9600
#define FIELD_Y_B 6600
#define FIELD_OUT_LINE 300  //フィールドの外白線と壁までの距離

#define ROBOT_MAX_VEL 1200  //ロボットの最大速度(mm/s)
#define ROBOT_MAX_ACCEL 2000 //ロボットの最大速度(mm/s^2)
#define ROBOT_MAX_JARK 10000 //ロボットの最大躍度(mm/s^3)
#define ROBOT_DRIBBLE_MAX_ACCEL 300 //ドリブル時の最大加速度

#define MICON_TIME_STEP 0.001          //micon controll time step
#define RASPI_TIME_STEP 0.01

//ペナルティゾーンのpathのマージン
#define PENALTY_ZONE_MARGIN_DISTANCE 150      //Margin distance when avoiding obstacles
//フィールドのペナルティゾーンのマージン設定
#define PENALTY_MARGIN 3   //mm
#define PENALTY_OUT_MARGIN 15   //mm
#define MIDLE_TARGET_MARGIN 50   //mm
#define PENALTY_MIDLE_TARGET_MARGIN 300   //mm
//侵入禁止エリアに侵入している際に使うマクロ
#define MAX_SEARCH_DISTANCE 1000    //侵入禁止エリア外に出るために探索する最大範囲(mm)
#define DELTA_SEARCH_DISTANCE 100   //侵入禁止エリア外に出るために探索する際の探索幅(mm)
//フィールドの壁への衝突を回避するときのマージン設定
#define FIELD_MARGIN 50   //mm
#define FIELD_TRGET_MARGIN 55   //mm

#define DWA_TRAP_CHANGE_DIS 100  //DWAと台形制御を切り替えるロボットと目標値の距離(mm)

#define DWA_ROBOTXY_VIRUALXY_DISTANCE_CHECK 500   //DWA中に仮の目標値とロボットの位置が離れられる最大値
#define TRAPE_ROBOTXY_VIRUALXY_DISTANCE_CHECK 600   //台形制御中に仮の目標値とロボットの位置が離れられる最大値
#define ROBOT_POSITION_RESET_DISTANCE 700         //ロボットの現在地と仮想目標値が大きく離れすぎた場合に現在地等をリセットする閾値となる距離

#define WRAP_KICK_CONTROL_CHANGE_DISTANCE 100   //ボールに対する回り込み動作を行う最大のロボットとボールの距離 (mm)
#define ROBOT_KICK_ENABLE_X 90                  //ロボットがボールを蹴れる最大値のx(ロボット座標系) (mm)
#define ROBOT_KICK_ENABLE_Y 40                  //ロボットがボールを蹴れる最大値のy(ロボット座標)系 (mm)
#define ROBOT_KICK_MIN_X 95    //ボールを蹴るために保持するときのボールのロボット座標系のx座標(mm)(ロボットのキッカー中心までの距離＋ボールの半径)
#define ROBOT_WRAP_KICK_MAGIC_NUMBER 100    //回り込みキックをする際の調整係数
#define ROBOT_NOT_TOUCH_MIN_X 105           //回り込み動作時にボールに触れない最小のx座標
#define ROBOT_WRAP_DRIBBLE_NON_ACTIVE 30    //回り込みキックの際にドリブラーを回さない距離(mm)(実際のロボットでは0にする予定)

#define ROBOT_DRIBBLE_START_DEG 10         //ロボットがドリブルによってボールを運ぶときにロボット目標値の向きからずれられる最大角度(DEG)
#define ROBOT_DRIBBLE_ABJUST_DEG 5/200     //ロボットがドリブルによってボールを運ぶときにボールがロボット中央からずれていたら傾きを調整する係数
#define ROBOT_DRIBBLE_DISTANCE 100        //ロボットがドリブルによってボールを移動させるときにこれ以上離れたら動作を停止する距離

//最大値を算出
float max_value(float data[], uint16_t size);
//最小値を算出
float min_value(float data[], uint16_t size);
//正規化
void min_max_normalize(float *data, uint16_t size);
//角度範囲内に再計算
float angle_range_corrector(float angle);
//角度範囲内に再計算(deg)
float angle_range_corrector_deg(float angle);
//角度が範囲内かを判定する
bool angele_check(float check_angle, float angle_max, float angle_min);
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.acosfの引数などに使用)
float modifid_acosf(float value);
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.asinfの引数などに使用)
float modifid_asinf(float value);
//ワールド座標系をロボット座標系に変更する関数
void w_to_r_coordinate_chang(float w_x, float w_y, float *r_x, float *r_y, float w_robot_x, float w_robot_y, float w_robot_theta);
//ロボット座標系をワールド座標系に変換する
void r_to_w_coordinate_chang(float *w_x, float *w_y, float r_x, float r_y, float w_robot_x, float w_robot_y, float w_robot_theta);
//(x1,y1)と(x2,y2)を通る直線上の店の中で(x3,y3)に最も近い点を求める
bool nearest_point_to_straight_line(float x1, float y1, float x2, float y2 ,float x3, float y3, float *x, float *y);
//(x1,y1)と(x2,y2)のノルムを計算する関数
float norm(float x1, float y1, float x2, float y2);
#endif // _TOOLS_H_