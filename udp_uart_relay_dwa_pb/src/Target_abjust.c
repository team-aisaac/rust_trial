#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include <unistd.h>
#include<math.h>
#include "Target_abjust.h"

#include <time.h>

//目標値がペナルティゾーン内やフィールド外にセットされた場合に，目標値を修正する 0:修正なし,1:修正有
bool target_abjust(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, int32_t *midle_targetX, int32_t *midle_targetY, bool prohidited_zone_ignore, bool *prohidited_zone_start){
    float x_penalty_line_puls;
    float x_penalty_line_minus;
    float y_penalty_line_puls;
    float y_penalty_line_minus;
    float x_outside_line_puls;
    float x_outside_line_minus;
    float y_outside_line_puls;
    float y_outside_line_minus;
    //フィールドの情報を格納
    field_data( &x_penalty_line_puls, &x_penalty_line_minus, &y_penalty_line_puls, &y_penalty_line_minus, &x_outside_line_puls,
                &x_outside_line_minus, &y_outside_line_puls, &y_outside_line_minus);
    //フィールド外の目標値をフィールド内に再設置
    bool field_out_flag = field_out_check(*targetX, *targetY, x_outside_line_puls, x_outside_line_minus, y_outside_line_puls, y_outside_line_minus);
    target_field_in(targetX, targetY, field_out_flag, x_outside_line_puls, x_outside_line_minus, 
                    y_outside_line_puls, y_outside_line_minus);
    //目標値がペナルティゾーン内の時に目標値を再設置
    bool field_penalty_flag = field_penalty_zone_check((float)*targetX, (float)*targetY, prohidited_zone_ignore, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus);
    target_penalty_zone_out(targetX, targetY, field_penalty_flag, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus);
    //現在地がペナルティゾーン内の場合はゾーン外に目標値を再設置
    *prohidited_zone_start = field_penalty_zone_check((float)x, (float)y, prohidited_zone_ignore, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus);
    penalty_zone_in_out(x, y, targetX, targetY, *prohidited_zone_start, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus);
    //現在地と目標値の経路中にペナルティゾーンを通過する場合，中継点を設置する
    bool midle_target_flag;
    *midle_targetX = *targetX;
    *midle_targetY = *targetY;
    midle_target_set(x, y, midle_targetX, midle_targetY, prohidited_zone_ignore, &midle_target_flag, x_penalty_line_puls, x_penalty_line_minus, y_penalty_line_puls, y_penalty_line_minus, *prohidited_zone_start);
    return midle_target_flag;
}

//フィールドの情報を格納
void field_data(float *x_penalty_line_puls, float *x_penalty_line_minus, float *y_penalty_line_puls, float *y_penalty_line_minus, float *x_outside_line_puls,
                float *x_outside_line_minus, float *y_outside_line_puls, float *y_outside_line_minus){
    if(FIELD == "A"){
        *x_penalty_line_puls = FIELD_X_A/2 - FIELD_OUT_LINE - PENALTY_ZONE_X_A;
        *x_penalty_line_minus = -FIELD_X_A/2 + FIELD_OUT_LINE + PENALTY_ZONE_X_A;
        *y_penalty_line_puls = PENALTY_ZONE_Y_A/2;
        *y_penalty_line_minus = -PENALTY_ZONE_Y_A/2;
        *x_outside_line_puls = FIELD_X_A/2 - FIELD_OUT_LINE;
        *x_outside_line_minus = -FIELD_X_A/2 + FIELD_OUT_LINE;
        *y_outside_line_puls = FIELD_Y_A/2 - FIELD_OUT_LINE;
        *y_outside_line_minus = -FIELD_Y_A/2 + FIELD_OUT_LINE;
    }
    else if(FIELD == "B"){
        *x_penalty_line_puls = FIELD_X_B/2 - FIELD_OUT_LINE - PENALTY_ZONE_X_B;
        *x_penalty_line_minus = -FIELD_X_B/2 + FIELD_OUT_LINE + PENALTY_ZONE_X_B;
        *y_penalty_line_puls = PENALTY_ZONE_Y_B/2;
        *y_penalty_line_minus = -PENALTY_ZONE_Y_B/2;
        *x_outside_line_puls = FIELD_X_B/2 - FIELD_OUT_LINE;
        *x_outside_line_minus = -FIELD_X_B/2 + FIELD_OUT_LINE;
        *y_outside_line_puls = FIELD_Y_B/2 - FIELD_OUT_LINE;
        *y_outside_line_minus = -FIELD_Y_B/2 + FIELD_OUT_LINE;
    }
}

//フィールド外に目標値がセットされているか確認
bool field_out_check(int32_t targetX, int32_t targetY, float x_outside_line_puls, float x_outside_line_minus, float y_outside_line_puls, float y_outside_line_minus){
    if(x_outside_line_minus - FIELD_OUT_LINE + ROBOT_SIZE/2 + FIELD_MARGIN <= targetX && targetX <= x_outside_line_puls + FIELD_OUT_LINE - ROBOT_SIZE/2 - FIELD_MARGIN){
        if(y_outside_line_minus - FIELD_OUT_LINE + ROBOT_SIZE/2 + FIELD_MARGIN <= targetY && targetY <= y_outside_line_puls + FIELD_OUT_LINE - ROBOT_SIZE/2 - FIELD_MARGIN){
            return 0;
        }
    }
    return 1;
}
//フィールド外の目標値をフィールド内に再設置
void target_field_in(int32_t *targetX, int32_t *targetY, bool field_out_flag, float x_outside_line_puls, float x_outside_line_minus, 
                     float y_outside_line_puls, float y_outside_line_minus){
    if(field_out_flag == 0){
        return;
    }
    float field_x_up = x_outside_line_puls + FIELD_OUT_LINE - FIELD_TRGET_MARGIN - ROBOT_SIZE/2;
    float field_x_down = x_outside_line_minus - FIELD_OUT_LINE + FIELD_TRGET_MARGIN + ROBOT_SIZE/2;
    float field_y_up = y_outside_line_puls + FIELD_OUT_LINE - FIELD_TRGET_MARGIN - ROBOT_SIZE/2;
    float field_y_down = y_outside_line_minus - FIELD_OUT_LINE + FIELD_TRGET_MARGIN + ROBOT_SIZE/2;
    if(*targetX < field_x_down){
        *targetX = field_x_down;
    }
    else if(field_x_up < *targetX){
        *targetX = field_x_up;
    }
    if(*targetY < field_y_down){
        *targetY = field_y_down;
    }
    else if(field_y_up < *targetY){
        *targetY = field_y_up;
    }
}
//ペナルティゾーンへの侵入確認
bool field_penalty_zone_check(float x, float y, bool prohidited_zone_ignore, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus){
    if(prohidited_zone_ignore == 1){
        return 0;
    }
    if(y_penalty_line_minus-PENALTY_MARGIN-ROBOT_SIZE/2 <= y && y <= y_penalty_line_puls+PENALTY_MARGIN+ROBOT_SIZE/2){
        if(x <= x_penalty_line_minus + PENALTY_MARGIN + ROBOT_SIZE/2){
            if(x_penalty_line_minus < x && (y < y_penalty_line_minus || y_penalty_line_puls < y)){
                float distance = sqrt((x_penalty_line_minus-x)*(x_penalty_line_minus-x) + (y_penalty_line_minus-y)*( y_penalty_line_minus-y));
                float distance2 = sqrt((x_penalty_line_minus-x)*(x_penalty_line_minus-x) + (y_penalty_line_puls-y)*(y_penalty_line_puls-y));
                if(distance < PENALTY_MARGIN+ROBOT_SIZE/2 || distance2 < PENALTY_MARGIN+ROBOT_SIZE/2){
                    return 1;
                }
                else{
                    return 0;
                }
            }
            else{
                return 1;
            }
        }
        else if(x_penalty_line_puls - PENALTY_MARGIN - ROBOT_SIZE/2 <= x){
            if(x < x_penalty_line_puls && (y < y_penalty_line_minus || y_penalty_line_puls < y)){
                float distance = sqrt((x_penalty_line_puls-x)*(x_penalty_line_puls-x) + (y_penalty_line_minus-y)*( y_penalty_line_minus-y));
                float distance2 = sqrt((x_penalty_line_puls-x)*(x_penalty_line_puls-x) + (y_penalty_line_puls-y)*(y_penalty_line_puls-y));
                if(distance < PENALTY_MARGIN+ROBOT_SIZE/2 || distance2 < PENALTY_MARGIN+ROBOT_SIZE/2){
                    return 1;
                }
                else{
                    return 0;
                }
            }
            else{
                return 1;
            }
        }
        return 0;
    }
}
//ペナルティゾーン内の目標値をペナルティゾーン外に再設置
void target_penalty_zone_out(int32_t *targetX, int32_t *targetY, bool field_penalty_flag, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus){
    float x_line_puls = x_penalty_line_puls - PENALTY_OUT_MARGIN - ROBOT_SIZE/2;
    float x_line_minus = x_penalty_line_minus + PENALTY_OUT_MARGIN + ROBOT_SIZE/2;
    float y_line_puls = y_penalty_line_puls + PENALTY_OUT_MARGIN + ROBOT_SIZE/2;
    float y_line_minus = y_penalty_line_minus - PENALTY_OUT_MARGIN - ROBOT_SIZE/2;
    if(field_penalty_flag == 0){//特に干渉無し
        return;
    }
    else{
        if(0 <= *targetX){
            if(0 <= *targetY){//第1象限第1象限
                float dx = abs(x_line_puls - *targetX);
                float dy = abs(y_line_puls - *targetY);
                if(dx < dy){
                    if(x_line_puls < *targetX){
                        *targetX = x_line_puls;
                    }
                }else{
                    if(*targetY < y_line_puls){
                        *targetY = y_line_puls;
                    }
                }
            }
            else{//第4象限
                float dx = abs(x_line_puls - *targetX);
                float dy = abs(y_line_minus - *targetY);
                if(dx < dy){
                    if(x_line_puls < *targetX){
                        *targetX = x_line_puls;
                    }
                }else{
                    if(y_line_minus < *targetY){
                        *targetY = y_line_minus;
                    }
                }
            }
        }
        else{
            if(0 < *targetY){//第2象限
                float dx = abs(x_line_minus - *targetX);
                float dy = abs(y_line_puls - *targetY);
                if(dx < dy){
                    if(*targetX < x_line_minus){
                        *targetX = x_line_minus;
                    }
                }else{
                    if(*targetY < y_line_puls){
                        *targetY = y_line_puls;
                    }
                }
            }
            else{//第3象限
                float dx = abs(x_line_minus - *targetX);
                float dy = abs(y_line_minus - *targetY);
                if(dx < dy){
                    if(*targetX < x_line_minus){
                        *targetX = x_line_minus;
                    }
                }else{
                    if(y_line_minus < *targetY){
                        *targetY = y_line_minus;
                    }
                }
            }
        }
    }
}
//現在地がペナルティゾーン内の場合は目標値をゾーン外に再設置
void penalty_zone_in_out(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, bool field_penalty_flag, float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus){
    float x_line_puls = x_penalty_line_puls - PENALTY_MARGIN - ROBOT_SIZE/2;
    float x_line_minus = x_penalty_line_minus + PENALTY_MARGIN + ROBOT_SIZE/2;
    float y_line_puls = y_penalty_line_puls + PENALTY_MARGIN + ROBOT_SIZE/2;
    float y_line_minus = y_penalty_line_minus - PENALTY_MARGIN - ROBOT_SIZE/2;
    if(field_penalty_flag == 0){//特に干渉無し
        return;
    }
    else{
        if(0 <= x){
            if(0 <= y){//第1象限第1象限
                float dx = abs(x_line_puls - x);
                float dy = abs(y_line_puls - y);
                if(dx < dy){
                    *targetX = x_line_puls - PENALTY_MIDLE_TARGET_MARGIN;
                    *targetY = y;
                }else{
                    *targetX = x;
                    *targetY = y_line_puls + PENALTY_MIDLE_TARGET_MARGIN;
                }
            }
            else{//第4象限
                float dx = abs(x_line_puls - x);
                float dy = abs(y_line_minus - y);
                if(dx < dy){
                   *targetX = x_line_puls - PENALTY_MIDLE_TARGET_MARGIN;
                   *targetY = y;
                }else{
                    *targetX = x;
                    *targetY = y_line_minus - PENALTY_MIDLE_TARGET_MARGIN;
                }
            }
        }
        else{
            if(0 < y){//第2象限
                float dx = abs(x_line_minus - x);
                float dy = abs(y_line_puls - y);
                if(dx < dy){
                    *targetX = x_line_minus + PENALTY_MIDLE_TARGET_MARGIN;
                    *targetY = y;
                }else{
                    *targetX = x;
                    *targetY = y_line_puls + PENALTY_MIDLE_TARGET_MARGIN;
                }
            }
            else{//第3象限
                float dx = abs(x_line_minus - x);
                float dy = abs(y_line_minus - y);
                if(dx < dy){
                    *targetX = x_line_minus + PENALTY_MIDLE_TARGET_MARGIN;
                    *targetY = y;
                }else{
                    *targetX = x;
                    *targetY = y_line_minus - PENALTY_MIDLE_TARGET_MARGIN;
                }
            }
        }
    }
}
//現在地と目標値の経路中にペナルティゾーンを通過する場合，中継点を設置する
void midle_target_set(int32_t x, int32_t y, int32_t *targetX, int32_t *targetY, bool prohidited_zone_ignore, bool *midle_target_flag, 
                      float x_penalty_line_puls, float x_penalty_line_minus, float y_penalty_line_puls, float y_penalty_line_minus, bool prohidited_zone_start){
    *midle_target_flag = 0;
    if(prohidited_zone_ignore == 1 || prohidited_zone_start == 1){
        return;
    }
    float x_line_puls = x_penalty_line_puls - ROBOT_SIZE/2;
    float x_line_minus = x_penalty_line_minus + ROBOT_SIZE/2;
    float y_line_puls = y_penalty_line_puls + ROBOT_SIZE/2;
    float y_line_minus = y_penalty_line_minus - ROBOT_SIZE/2;
    if(*targetY - y != 0){
        float t = (y_line_puls - y)/(*targetY - y);
        float t2 = (y_line_minus - y)/(*targetY - y);
        float x_check = (*targetX - x) * t + x;
        float x_check2 = (*targetX - x) * t2 + x;
        if(0 < y){
            if(x_line_puls <= x_check && (0 < t && t < 1)){
                *targetX = x_line_puls - MIDLE_TARGET_MARGIN;
                *targetY = y_line_puls + MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_check <= x_line_minus && (0 < t && t < 1)){
                *targetX = x_line_minus + MIDLE_TARGET_MARGIN;
                *targetY = y_line_puls + MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_line_puls <= x_check2 && (0 < t2 && t2 < 1)){
                *targetX = x_line_puls - MIDLE_TARGET_MARGIN;
                *targetY = y_line_minus - MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_check2 <= x_line_minus && (0 < t2 && t2 < 1)){
                *targetX = x_line_minus + MIDLE_TARGET_MARGIN;
                *targetY = y_line_minus - MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
        }
        else{
            if(x_line_puls <= x_check2 && (0 < t2 && t2 < 1)){
                *targetX = x_line_puls - MIDLE_TARGET_MARGIN;
                *targetY = y_line_minus - MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_check2 <= x_line_minus && (0 < t2 && t2 < 1)){
                *targetX = x_line_minus + MIDLE_TARGET_MARGIN;
                *targetY = y_line_minus - MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_line_puls <= x_check && (0 < t && t < 1)){
                *targetX = x_line_puls - MIDLE_TARGET_MARGIN;
                *targetY = y_line_puls + MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
            else if(x_check <= x_line_minus && (0 < t && t < 1)){
                *targetX = x_line_minus + MIDLE_TARGET_MARGIN;
                *targetY = y_line_puls + MIDLE_TARGET_MARGIN;
                *midle_target_flag = 1;
            }
        }
    }
}


