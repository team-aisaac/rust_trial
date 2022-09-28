#ifndef _TOOLS_H_
#define _TOOLS_H_

#define ROBOT_SIZE 180            //robot size(diameter mm)

#define DEG_TO_RAD 0.0174532925199
#define RAD_TO_DEG 57.29577951

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

#endif // _TOOLS_H_