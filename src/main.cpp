#include <Arduino.h>
#include <M5Stack.h>

//DJI制御ライブラリ
#include <dji_mController.h>
#define PI2 6.28318530718

//DJIMcontrollerのメモリ確保
const int  NoM = 4;  //接続しているモーター数
DJI_mController *motor[NoM];

//CAN通信セッティング
#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsgOut;
struct can_frame canMsg;
MCP2515 mcp2515(12);

//モーター電流指令値
int set_current[NoM];
int gm_max_cur = 16384; //最大電流値

//設計値
const double d1 = 35;
const double d2 = 38;//42
const double d3 = 40;
const double gear = 36.1;
const double mm_to_pulse_Pos = 2/d2 *d2/d1*gear *8192/PI2;
const double mm_to_pulse_Mov = 2/d3 *d2/d1 *gear *8192/PI2;


//--------------------------------------------------------
//   以下，関数定義
//--------------------------------------------------------

//受信データデコード関数
inline int toRealData(unsigned char DataH, unsigned char DataL){
  return short(word(DataH, DataL));
}

//CAN通信でモーターに電流指令値を送信する関数
void can_send(){
    canMsgOut.can_id = 0x200;
    canMsgOut.can_dlc = 8;
    for (size_t i = 0; i < NoM/2; i++){
        //if(i> 3) break;
        canMsgOut.data[i*2]   = (char)(set_current[i] / 256); // High 8 bit
        canMsgOut.data[i*2+1] = (char)(set_current[i] % 256); // Low  8 bit
    }
    mcp2515.sendMessage(&canMsgOut);
}

//CAN通信でモーターの情報取得する関数
void can_read(){
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK ) {
        uint8_t id = canMsg.can_id - 0x201;
        if(id==3){
            //can_send();
            //これいる？　mcp2515_.sendMessage(&canMsgOut_);
        }
        int can_data[3];
        can_data[0] = toRealData(canMsg.data[0], canMsg.data[1]);//count
        can_data[1] = toRealData(canMsg.data[2], canMsg.data[3]);//rpm
        can_data[2] = toRealData(canMsg.data[4], canMsg.data[5]);//current
        motor[id] -> rcv_data(can_data[0], can_data[1] , can_data[2]);
    }
}

//M5のモニターに下記内容を表示する関数
void drawLCD(){
  //M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(5 ,5);
  M5.Lcd.printf("id, pos, mov  ");

  M5.Lcd.setCursor(35,210);
  M5.Lcd.print("push          pull");
}

//一定速度でモーター1個動作させるための関数
void moveShelfs(float trgPos_[], float speed[], int ID_){
  float dif = trgPos_[ID_] - motor[ID_] -> get_pos();
  Serial.printf("trgPos = %5.1f, crtPos = %5.1f, dif = %5.1f \n", trgPos_[ID_], motor[ID_] -> get_pos(), dif);
  float step_size = 1.0;//mm
  float samples = fabs(dif / step_size);
  if(samples < 10)samples =10;
  Serial.printf("samples = %5.1f", samples);
  int period_ms = 1000* step_size/speed[ID_];
  unsigned long myTimer = millis();
  float tg_pos;
  float startPos = motor[ID_] ->get_pos();
  for (size_t i = 1; i <= samples; i++){
    tg_pos = startPos + (trgPos_[ID_] - startPos)* i/samples;
    printf("i = %4d, tg_pos = %4.1f\n", i, tg_pos);
    motor[ID_]  -> set_trg(gm_max_cur*0.3, tg_pos);
    myTimer += period_ms;
    //Serial.printf("%5.1f, %5.1f\n", tg_posX, tg_posY);
    while (millis() < myTimer){
      delay(1);
    }
  }
}

//コア0のスレッドa
void Core0(void *args){
  while (1) {
    delayMicroseconds(1);
    can_read();
  }
}

//コア1のスレッドa
void Core0b(void *args) {
  while (1) {
    delay(1);
    set_current[0] = motor[0] -> calc_current(); 
    set_current[1] = motor[1] -> calc_current(); 
    set_current[2] = motor[2] -> calc_current(); 
    set_current[3] = motor[3] -> calc_current(); 
    can_send();
  }
}


//--------------------------------------------------------
//   以下，セットアップとメインループ
//--------------------------------------------------------

//セットアップ定義
void setup() {
  M5.begin(true,false,true,false);
  delay(50);

  Serial.begin(115200);

  //can init
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK ,WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0,50);
  M5.Lcd.println("CAN is");
  M5.Lcd.print("not connected");
  delay(10);

  Serial.println("Hello!!!!!!");
  //モーターのPIDゲインと変換パラメータ設定してインスタンス生成
  for (size_t i = 0; i < NoM/2; i++){
    Serial.println(i);
    motor[i*2]   = new DJI_mController(mm_to_pulse_Pos, 0.1, 0, 0.1); //mm_to_pulse_Pos
    motor[i*2+1] = new DJI_mController(mm_to_pulse_Mov, 0.1, 0, 0.1); 
  }

  Serial.print("can connecting");
  while(1){//canが接続するまでループ
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK ) break;
    if(M5.BtnB.read()==HIGH)break;
    Serial.print(".");
  }
  Serial.println("OK");

  //マルチタスク設定
  xTaskCreateUniversal(Core0,"Core0",8192,NULL,1,NULL,PRO_CPU_NUM);
  xTaskCreateUniversal(Core0b,"Core0b",8192,NULL,1,NULL,APP_CPU_NUM);

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(115,210);
  M5.Lcd.print("start");
  while(1){
    M5.update();
    if(M5.BtnB.isPressed())break;
  }

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(35,210);
  M5.Lcd.print("push      pull");

  delay(100);
  //初期位置の設定
  for (size_t i = 0; i < NoM; i++){
    motor[i] -> set_pos(0);
  }
}


float trgPos[] = {0,0,0,0};
float speed[] = {200, 100, 100, 100};

//メインループ定義
void loop() {

  if(M5.BtnA.read() == 1){
    
    // trgPos[0] = 500;
    // moveShelfs(*motor, trgPos, speed, 0);

    trgPos[1] = 200;
    moveShelfs(trgPos, speed, 1);

    while(M5.BtnA.read()){
      delay(10);
    }
  }

  if(M5.BtnB.read() == 1){
    motor[0]   -> stop();
    motor[1]   -> stop();
    motor[2]   -> stop();
    motor[3]   -> stop();

    while(M5.BtnB.read()){
      delay(10);
    }
  }

  if(M5.BtnC.read() == 1){
    trgPos[0] = 100;
    moveShelfs(trgPos, speed, 0);
    while(M5.BtnC.read()){
      delay(10);
    }
  }

  Serial.printf("%3.1f   %3.1f   %3.1f   %3.1f\n", motor[0] -> get_pos(), motor[1] -> get_pos(), motor[2] -> get_pos(), motor[3] -> get_pos());
  delay(50);
}