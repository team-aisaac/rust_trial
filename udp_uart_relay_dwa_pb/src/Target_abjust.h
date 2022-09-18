#ifndef _TARGET_ABJUST_H_
#define _TARGET_ABJUST_H_

#define ROBOT_SIZE 180            //robot size(diameter mm)
//フィールド選択
#define FIELD "A" //  フィールドの選択A or B
//ペナルティゾーンのpathのマージン
#define PENALTY_ZONE_MARGIN_DISTANCE 150      //Margin distance when avoiding obstacles
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
//フィールドのペナルティゾーンのマージン設定
#define PENALTY_MARGIN 5   //mm
#define PENALTY_OUT_MARGIN 10   //mm
#define MIDLE_TARGET_MARGIN 50   //mm
#define PENALTY_MIDLE_TARGET_MARGIN 300   //mm
//侵入禁止エリアに侵入している際に使うマクロ
#define MAX_SEARCH_DISTANCE 1000    //侵入禁止エリア外に出るために探索する最大範囲(mm)
#define DELTA_SEARCH_DISTANCE 100   //侵入禁止エリア外に出るために探索する際の探索幅(mm)
//フィールドの壁への衝突を回避するときのマージン設定
#define FIELD_MARGIN 50   //mm
#define FIELD_TRGET_MARGIN 55   //mm

//フィールドの情報を格納
void field_data(float *x_penalty_line_puls, float *x_penalty_line_minus, float *y_penalty_line_puls, float *y_penalty_line_minus, float *x_outside_line_puls,
                float *x_outside_line_minus, float *y_outside_line_puls, float *y_outside_line_minus);
//目標値がペナルティゾーン内やフィールド外にセットされた場合に，目標値を修正する 0:修正なし,1:修正有
bool target_abjust(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, int32_t *midle_targetX, int32_t *midle_targetY, bool prohidited_zone_ignore, bool *prohidited_zone_start);
//フィールド外に目標値がセットされているか確認
bool field_out_check(int32_t targetX, int32_t targetY, float x_outside_line_puls, float x_outside_line_minus, float y_outside_line_puls, float y_outside_line_minus);
//フィールド外の目標値をフィールド内に再設置
void target_field_in(int32_t *targetX, int32_t *targetY, bool field_out_flag, float x_outside_line_puls, float x_outside_line_minus, float y_outside_line_puls, float y_outside_line_minus);
//ペナルティゾーンへの侵入確認 0:ターゲット修正なし，1:ターゲット修正有
bool field_penalty_zone_check(float x, float y, bool prohidited_zone_ignore, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus);
//ペナルティゾーン内の目標値をペナルティゾーン外に再設置
void target_penalty_zone_out(int32_t *targetX, int32_t *targetY, bool field_penalty_flag, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus);
//現在地がペナルティゾーン内の場合は目標値をゾーン外に再設置
void penalty_zone_in_out(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, bool field_penalty_flag, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus);
//現在地と目標値の経路中にペナルティゾーンを通過する場合，中継点を設置する
void midle_target_set(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, bool prohidited_zone_ignore, bool *midle_target_flag, 
                      float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus, bool prohidited_zone_start);
#endif // _TARGET_ABJUST_H_