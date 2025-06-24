#include "Arduino.h"
#include "dji_mController.h"
#define sign(num) ((num) < 0 ? -1 : 1)

DJI_mController::DJI_mController(float unit_conversion, float init_Kp, float init_Ki, float init_Kd){
    factor = unit_conversion;
    Kp = init_Kp;
    Ki = init_Ki;
    Kd = init_Kd;
    time_pre = micros();
}

void DJI_mController::set_pos(float num){
    fb_position = num  *factor;
}

void DJI_mController::print(){
    Serial.printf("%7d, %7d, %7d\n", fb_position, tg_position, tg_current);
    //Serial.printf("%7d, %7d, %7d, %7d, %7d\n", encoder_pre, fb_position, tg_position, error, fb_velocity);
    //Serial.printf("%7d, %7d, %7d, %7d, %7d\n", error, tg_current, tg_velocity, fb_velocity, fb_rpm/60*8192);

    //Serial.printf("%6d,%6d,%6d,%6d\n",abs(tg_velocity), abs(fb_rpm *8192/60), 0, 12000);
    //Serial.printf("%6d\n",abs(fb_rpm *8192/60) );
    //Serial.printf("%6d,%6d,%6d,%6d,%6d\n",error, tg_current, abs(tg_velocity), abs(fb_rpm)*8192/60,abs(int(control_d)));

}

void DJI_mController::set_acc_param(float target_current_limit, float target_acceleration, float target_deceleration, float target_position, float cruising_speed){
    tg_current_limit= target_current_limit;
    tg_acceleration = target_acceleration *factor;
    tg_deceleration = target_deceleration *factor;
    tg_position     = target_position     *factor;
    tg_velocity_limit  = cruising_speed   *factor;

    fb_velocity_atT0 = fb_rpm * 8192/60;
    fb_micros_atT0 = micros();
}

void DJI_mController::set_trg(int target_current_limit, float target_position){
    tg_current_limit= target_current_limit;
    tg_position = target_position *factor;
}


void DJI_mController::rcv_data(int rcv_count, int rcv_rpm , int rcv_current){
    //オドメトリ更新
    int dif = rcv_count - encoder_pre;
    encoder_pre = rcv_count;
    if(abs(dif) > 4095)//原点をまたぐ場合
        dif = sign(dif)*(abs(dif)-8192);
    fb_position += dif;
    
    fb_rpm = fb_rpm *0.9 + 0.1 *rcv_rpm;
    fb_current = rcv_current;

    //if(abs(fb_rpm) > 120)fb_rpm = sign(fb_rpm) *120;

    //fb_rpm_dif = (fb_rpm - fb_rpm_pre);
    //fb_rpm_pre = fb_rpm;
}

int DJI_mController::stop(){
    tg_current_limit=0;
    return 0;
}

int DJI_mController::calc_current(){

    long time_cur = micros();
    dt = float(time_cur - time_pre)/ 1E6;
    time_pre = time_cur;


    /*PI-D begin*/
    error = tg_position - fb_position;
    //error[0] = long(target *factor) - position[0];
    control_p = Kp *error;

    integral += float(error) *dt;
    control_i = Ki *integral;

    //differential =float(position[0] - position[1]) /dt;
    differential =  Kd *(136.5333333 *fb_rpm);
    //position[1] = position[0];
    control_d = -Kd *differential;
    /*PI-D end*/

    /*台形速度 begin
    error = tg_position - fb_position;
    tg_velocity_atDec = sign(error)* sqrt(2.0f*tg_deceleration*abs(error));

    //振動抑制
    
    long tolerance_velocity = 2000;
    if(abs(tg_velocity_atDec) < tolerance_velocity)
        tg_velocity_atDec = sign( tg_velocity_atDec)*  tg_velocity_atDec * tg_velocity_atDec /tolerance_velocity;

      //deltaVmax = sign(deltaVmax) *(deltaVmax/10000000.0) *(deltaVmax/10000000.0) *10000000.0;

    
    tg_velocity_atAcc = fb_velocity_atT0 + sign(error)*tg_acceleration*float(micros()-fb_micros_atT0)/1E6;

    

    if(abs(tg_velocity_atDec) < abs(tg_velocity_atAcc))
        tg_velocity = long(tg_velocity_atDec);
    else{
        tg_velocity = long(tg_velocity_atAcc);
    }

    if(abs(tg_velocity) > tg_velocity_limit)
        tg_velocity = sign(tg_velocity) *tg_velocity_limit;

    //fb_velocity = fb_velocity *0.99 +0.01*(fb_position -fb_position_pre) /dt;
    //fb_position_pre = fb_position;

    //control_p = Kp *(tg_velocity - fb_velocity);
    control_p = Kp *float(tg_velocity - 136.5333333 *fb_rpm);

    control_d = Kd *float(fb_rpm_dif) /dt;


    //台形速度 end*/

    tg_current = int(control_p + control_i + control_d + control_g);
    if(abs(tg_current) > tg_current_limit) 
        tg_current = sign(tg_current)* tg_current_limit;

    return tg_current;
    //return 0;
}