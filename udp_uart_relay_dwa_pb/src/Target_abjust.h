#ifndef _TARGET_ABJUST_H_
#define _TARGET_ABJUST_H_

#ifdef __cplusplus
extern "C" {
#endif
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
#ifdef __cplusplus
}
#endif
#endif // _TARGET_ABJUST_H_