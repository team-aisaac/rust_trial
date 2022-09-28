#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include <unistd.h>
#include<math.h>
#include "dwa.h"

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