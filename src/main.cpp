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

//制御モータ制御関連パラメータ
bool moving = false;
float trgPos[] = {0,0,0,0};
float speed[] = {200, 100, 200, 100};

//UART通信関係変数
bool rcv = false;
bool findHeader = false;
int16_t rcvData[4];
uint8_t rcvBinary[3];
long sendTime = 0;
int cnt = 0;

//送信データ配列
uint8_t sendData[10];

//作業完了状態配列


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
    for (size_t i = 0; i < NoM; i++){
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
  M5.Lcd.print("A     B     C");
}

//一定速度でモーター1個動作させるための関数
void moveShelfs(float trgPos_[], float speed[], int ID_){
  float dif = trgPos_[ID_] - motor[ID_] -> get_pos();
  //Serial.printf("trgPos = %5.1f, crtPos = %5.1f, dif = %5.1f \n", trgPos_[ID_], motor[ID_] -> get_pos(), dif);
  float step_size = 1.0;//mm
  float samples = fabs(dif / step_size);
  if(samples < 10)samples =10;
  //Serial.printf("samples = %5.1f", samples);
  int period_ms = 1000* step_size/speed[ID_];
  unsigned long myTimer = millis();
  float tg_pos;
  float startPos = motor[ID_] ->get_pos();
  for (size_t i = 1; i <= samples; i++){
    tg_pos = startPos + (trgPos_[ID_] - startPos)* i/samples;
    //printf("i = %4d, tg_pos = %4.1f\n", i, tg_pos);
    motor[ID_]  -> set_trg(gm_max_cur*0.3, tg_pos);
    myTimer += period_ms;
    //Serial.printf("%5.1f, %5.1f\n", tg_posX, tg_posY);
    while (millis() < myTimer){
      delay(1);
    }
  }
}

//移動完了判定関数
bool moveCmp(float trgPos_[]){
  bool cmpFlg = true;
  for(int i = 0; i < NoM; i++){
    if(fabs(trgPos_[i] - motor[i] -> get_pos()) > 10.0 ){
      cmpFlg = false;
    }
  }
  return cmpFlg;
}

//コア0のスレッドa
void Core0a(void *args){
  while (1) {
    delayMicroseconds(1);
    can_read();
  }
}

//コア1のスレッドa
void Core1a(void *args) {
  while (1) {
    delay(1);
    set_current[0] = motor[0] -> calc_current(); 
    set_current[1] = motor[1] -> calc_current(); 
    set_current[2] = motor[2] -> calc_current(); 
    set_current[3] = motor[3] -> calc_current(); 
    can_send();
  }
}

//コア1のスレッドb
void Core1b(void *args) {
  while (1) {
    delay(1);
    if(moving){
      moveShelfs(trgPos, speed, 0);
      moveShelfs(trgPos, speed, 1);
      moveShelfs(trgPos, speed, 2);
      moveShelfs(trgPos, speed, 3);
      moving = false;
      motor[1] -> set_pos(0);
      motor[1] -> set_trg(0,0);
      motor[3] -> set_pos(0);
      motor[3] -> set_trg(0,0);
    }
  }
}

//--------------------------------------------------------
//   以下，セットアップとメインループ
//--------------------------------------------------------

//セットアップ定義
void setup() {
  M5.begin(true,false,true,false);
  delay(50);

  //UART通信のセットアップ
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

  //モーターのPIDゲインと変換パラメータ設定してインスタンス生成
  for (size_t i = 0; i < NoM/2; i++){
    //Serial.println(i);
    motor[i*2]   = new DJI_mController(mm_to_pulse_Pos, 0.1, 0, 0.1); //mm_to_pulse_Pos
    motor[i*2+1] = new DJI_mController(mm_to_pulse_Mov, 0.1, 0, 0.1); 
  }

  //Serial.print("can connecting");
  while(1){//canが接続するまでループ
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK ) break;
    if(M5.BtnB.read()==HIGH)break;
    //Serial.print(".");
  }
  //Serial.println("CAN is OK");

  //マルチタスク設定
  xTaskCreateUniversal(Core0a,"Core0a",8192,NULL,1,NULL,PRO_CPU_NUM);
  xTaskCreateUniversal(Core1a,"Core1a",8192,NULL,1,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(Core1b,"Core1b",8192,NULL,2,NULL,APP_CPU_NUM);

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


//メインループ定義
void loop() {
  //ボタンAが押されたとき
  if(M5.BtnA.read() == 1){
    trgPos[0] = 300;
    trgPos[1] = 150;
    trgPos[2] = 300;
    trgPos[3] = 150;
    moving = true;
    while(M5.BtnA.read()){
      delay(10);
    }
  }
  //ボタンBが押されたとき
  if(M5.BtnB.read() == 1){
    motor[0]   -> stop();
    motor[1]   -> stop();
    motor[2]   -> stop();
    motor[3]   -> stop();

    while(M5.BtnB.read()){
      delay(10);
    }
  }
  //ボタンCが押されたとき
  if(M5.BtnC.read() == 1){
    trgPos[0] = 0;
    trgPos[1] = -150;
    trgPos[2] = 0;
    trgPos[3] = -150;
    moving = true;
    while(M5.BtnC.read()){
      delay(10);
    }
  }

  //シリアル受信処理
  // ヘッダー検出
  while (!findHeader && Serial.available() > 0) {
    if (Serial.read() == 0xFF) {
      findHeader = true;
    }
  }
  // ヘッダー検出後、必要なバイト数があるか確認
  if (findHeader && Serial.available() >= 12) {
    for (int i = 0; i < 4; i++) {
      // 3バイト受信
      for (int j = 0; j < 3; j++) {
        rcvBinary[j] = Serial.read();
      }
      // デコード処理（変更しない）
      uint16_t high = ((rcvBinary[0] << 6) | (rcvBinary[1] >> 1)) & 0xFF;
      uint16_t low  = ((rcvBinary[1] << 7) | rcvBinary[2]) & 0xFF;
      rcvData[i] = (high << 8) | low;
    }
    rcv = true;
    findHeader = false;
    cnt = 0;
    // 表示更新
    M5.Lcd.fillRect(0, 80, 320, 20, BLACK);  // 黒でクリア
    while (Serial.available() > 0) {
      Serial.read();// 残っているバイトを読み捨てる
    }

    // モーター目標値登録や表示灯の設定など
    int id = rcvData[0];
    trgPos[id*2] = rcvData[1];
    trgPos[id*2+1] = rcvData[2];
    moving = true;
  }

  //作業状態配列を更新

  //シリアル送信処理
  if(millis() - sendTime > 1000){
    //各棚の状態と非常停止押下状態の送信
    uint16_t sendData[10];
    if(rcv && cnt < 2){
      sendData[0] = 255;
      sendData[1] = 1;
      sendData[2] = 1;
      sendData[3] = 1;
      sendData[4] = 1;
      sendData[5] = 1;
      sendData[6] = 1;
      sendData[7] = 0;
      sendData[8] = 0;
      sendData[9] = 1;
      cnt++;
    }
    else if(rcv && cnt >= 2 && cnt < 4){
      sendData[0] = 255;
      sendData[1] = 1;
      sendData[2] = 1;
      sendData[3] = 1;
      sendData[4] = 1;
      sendData[5] = 1;
      sendData[6] = 1;
      sendData[7] = 0;
      sendData[8] = 0;
      sendData[9] = 0;
      cnt++;
    }
    else{
      sendData[0] = 255;
      sendData[1] = 0;
      sendData[2] = 0;
      sendData[3] = 0;
      sendData[4] = 0;
      sendData[5] = 0;
      sendData[6] = 0;
      sendData[7] = 0;
      sendData[8] = 0;
      sendData[9] = 0;
    }
    
    for(int i = 0; i < 10; i++){
      Serial.write(sendData[i]);
    }
    sendTime = millis();
  }

  //Serial.printf("%3.1f   %3.1f   %3.1f   %3.1f\n", motor[0] -> get_pos(), motor[1] -> get_pos(), motor[2] -> get_pos(), motor[3] -> get_pos());
  delay(50);
}