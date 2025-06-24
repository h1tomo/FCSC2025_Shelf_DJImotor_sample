#ifndef dji_mController_h
#define dji_mController_h


class DJI_mController
{
public:


  DJI_mController(float unit_conversion, float init_Kp, float init_Ki, float init_Kd);
  void print();
  void set_acc_param(float target_current_limit, float target_acceleration, float target_deceleration, float target_position, float cruising_speed);
  int  stop();
  void rcv_data(int rcv_count, int rcv_rpm , int rcv_current);
  void set_pos(float num);
  void set_trg(int target_current_limit, float target_position);
  int  calc_current();
  float get_pos(){return float(fb_position) /factor;}
  float get_abs(){return float(encoder_pre) /factor;}
  int get_cur(){return fb_current;}

  


private:
// クラス内で使用するメンバ変数
  float dt;//制御周期
  long time_pre;

  long encoder_pre = 0;//0~8192　position計算に使う

  float factor;//mm変換
  const float PpsToRpm = 0.00732421875; //60sec /8192pulse
  float Kp = 0, Ki = 0, Kd = 0;
  float control_p = 0, control_i = 0, control_d = 0, control_g = 0;   
  //long encoder_dif; //目標値との差
  //long error[2];//0:今回の偏差　1:前回
  float differential;//偏差の微分
  float integral;//偏差の積分



  //電流
  int fb_current = 0;
  int tg_current = 0;
  int tg_current_limit = 0;

  //位置と偏差
  long fb_position = 0;
  long fb_position_pre = 0;
  long tg_position = 0;

  long error = 0; //tg_pos - fb_pos
  long error_pre = 0;

  //速度
  float   fb_rpm = 0;
  long   fb_rpm_pre = 0;
  long   fb_rpm_dif = 0;
  long   fb_velocity = 0;//fb_posの一回微分
  long   fb_velocity_atT0 = 0;
  long   fb_micros_atT0 = 0;
  long   tg_velocity = 0;
  long   tg_velocity_limit = 0;
  float tg_velocity_atAcc = 0;
  float tg_velocity_atDec = 0;

  float fb_acceleration = 0;//fb_posの二回微分
  float tg_acceleration = 0;//加速度
  float tg_deceleration = 0;//減速度
    
};

#endif