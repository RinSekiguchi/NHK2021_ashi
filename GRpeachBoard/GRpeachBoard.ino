// 1経路のみの軌道追従制御のサンプルプログラムです(まだ出来てない8/7現在)
// dualshock4対応
// 作成日：2021年5月27日　作成者：ueno
// 最終修正日：2021年7月25日 編集者：ueno　※上半身との通信対応
#include <Arduino.h>
#include <MsTimer2.h>
#include <math.h>
#include <SPI.h>

#include "define.h"

#include "AutoControl.h"
#include "Button.h"
#include "Controller.h"
#include "LpmsMe1Peach.h"
#include "ManualControl.h"
#include "PhaseCounterPeach.h"
#include "Platform.h"
#include "SDclass.h"
#include "RoboClaw.h"
#include "CommHost.h"

PhaseCounter enc1(1);
PhaseCounter enc2(2);
ManualControl manualCon;
AutoControl autonomous;
Platform platform(1, 1, 1, 1); // 括弧内の引数で回転方向を変えられる
Controller CON;
CommHost UpperBody(&SERIAL_UPPER);
PID PID_x(0.9, 0.0, 0.01, INT_TIME);      //速度PID用(ジョイスティックから速度生成-現在速度)
PID PID_y(0.9, 0.0, 0.01, INT_TIME);      //まだ決まってない
PID angle_posi(2.4, 0.0, 0.01, INT_TIME); //旋回位置のPID
PID POSI_x(0.7, 0.0, 0.0, INT_TIME);      //位置PID_x
PID POSI_y(0.7, 0.0, 0.0, INT_TIME);      //位置PID_y
Filter fil_x(INT_TIME);
Filter fil_y(INT_TIME);
Filter Low_vel(INT_TIME);
Filter High_vel(INT_TIME);

//AMT203V amt203(&SPI, PIN_CSB);
LpmsMe1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
bool SDwrite = false; // trueでSDカードに書き出し

coords gPosi = {0.0, 0.0, 0.0};
coords gRefV = {0.0, 0.0, 0.0};
//自分で付け足した変数
coords pre_gPosi = {0.0, 0.0, 0.0};
double angle_rad = 0.0;
int zone = RED;
bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;
bool flag_5s = false;
bool flag_1s = false;

unsigned int robotState = 0; // ロボットの状態
#define STATE_LPMS_ENABLE 0x01
#define STATE_SD_INIT 0x02
#define STATE_SD_WRITE 0x04
#define STATE_ZONE 0x08
#define STATE_READY 0x10

Button button_up(PIN_SW_UP);
Button button_down(PIN_SW_DOWN);
Button button_left(PIN_SW_LEFT);
Button button_right(PIN_SW_RIGHT);
Button button_yellow(PIN_SW_A);
Button button_white(PIN_SW_B);

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if (value > minmax)
    value = minmax;
  else if (value < -minmax)
    value = -minmax;
  return value;
}

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval)
{
  analogWrite(pin, 0);
  for (int i = 0; i < times; i++)
  {
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

// setupで有効にされるタイマ割り込み処理が書いてある場所
void timer_warikomi()
{
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  static int count_flag = 0;
  static int count_flag_5s = 0;
  static int count_flag_1s = 0;
  count += 2; // ここで光る周期を変えられる(はず)
  count_flag++;
  count_flag_5s++;
  count_flag_1s++;

  if (count < 255)
  {
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if (count < 255 * 2)
  {
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if (count < 255 * 3)
  {
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else
  {
    count = 0;
  }

  // フラグ立てるための処理
  flag_10ms = true;
  if (count_flag >= 10)
  {
    flag_100ms = true;
    count_flag = 0;
  }
  if (count_flag_1s >= 100)
  {
    flag_1s = true;
    count_flag_1s = 0;
  }
  if (count_flag_5s >= 500)
  {
    flag_5s = true;
    count_flag_5s = 0;
  }

  //double angle_rad;//グローバルに移動
  int encX, encY; // X,Y軸エンコーダのカウント値
  // 自己位置推定用エンコーダのカウント値取得
  encX = -enc1.getCount();
  encY = -enc2.getCount();

  // LPMS-ME1のから角度を取得
  angle_rad = (double)lpms.get_z_angle();
  gPosi = platform.getPosi(encX, encY, angle_rad);
}

// エラーが発生したら無限ループで停止
void error_stop()
{
  SERIAL_M5STACK.println("!!ERROR!");
  while (1)
  {
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void setup()
{
  Serial.begin(115200);
  //SERIAL_CON.begin(115200);
  SERIAL_M5STACK.begin(115200);
  SERIAL_UPPER.begin(115200);
  SERIAL_XBEE.begin(115200);

  pinMode(PIN_XBEERESET, OUTPUT); // XBeeのリセット
  digitalWrite(PIN_XBEERESET, 0);
  delay(10);
  digitalWrite(PIN_XBEERESET, 1);
  delay(10);

  pinMode(PIN_SW, INPUT); // オンボードのスイッチ

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);

  pinMode(PIN_DIP1, INPUT);
  pinMode(PIN_DIP2, INPUT);
  pinMode(PIN_DIP3, INPUT);
  pinMode(PIN_DIP4, INPUT);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
  analogWrite(PIN_LED_BLUE, 0);
  analogWrite(PIN_LED_GREEN, 0);

  // LPMS-ME1の初期化
  if (lpms.init() != 1)
    error_stop(); // 理由はわからないが，これをやる前にLEDblinkかanalogWriteを実行していないと初期化できない
  robotState |= STATE_LPMS_ENABLE;
  LEDblink(PIN_LED_BLUE, 2, 100);                 // 初期化が終わった証拠にブリンク
  SERIAL_M5STACK.println("!LPMS-ME1 init done!"); // M5stackのデバッグエリアに出力「!」を先頭に付ければ改行コードの前までを出力できる
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();

  if (mySD.init() == 0)
  {
    robotState |= STATE_SD_INIT;
    SERIAL_M5STACK.println("!SD-card init done!");
  }
  else
  {
    SERIAL_M5STACK.println("!SD-card init failed!!!");
  }
  delay(10);

  SDwrite = digitalRead(PIN_DIP4) == 0 ? true : false;

  if (SDwrite)
  {
    mySD.make_logfile();
    robotState |= STATE_SD_WRITE;
    SERIAL_M5STACK.println("!Log file created!");
  }

  zone = digitalRead(PIN_DIP1) == 0 ? BLUE : RED;
  int actpathnum = autonomous.init(&mySD, zone); //←mySD.path_read(BLUE, motion->Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
  Serial.print("path num: ");
  Serial.println(actpathnum + 1);
  SERIAL_M5STACK.print("!Path num: ");
  SERIAL_M5STACK.println(actpathnum + 1);
  if (zone == BLUE)
    robotState |= STATE_ZONE;

  autonomous.gPosiInit();
  LEDblink(PIN_LED_RED, 2, 100);

  //robotState |= STATE_WAIT_INPUT;
  SERIAL_M5STACK.println("!Waiting controller input");

  // コントローラの"右"ボタンが押されるまで待機
  while ((robotState & STATE_READY) == 0)
  {
    delay(5);
    CON.update();
    send_state();

    if (CON.readButton(BUTTON_RIGHT) == 2)
    {
      robotState |= STATE_READY;
      SERIAL_M5STACK.println("!READY TO GO !!!!!!!!!!");
    }
  }

  enc1.init();
  enc2.init();

  PID_x.PIDinit(0.0, 0.0);
  PID_y.PIDinit(0.0, 0.0);
  POSI_x.PIDinit(0.0, 0.0);
  POSI_y.PIDinit(0.0, 0.0);
  angle_posi.PIDinit(0.0, 0.0);

  fil_x.setSecondOrderPara(22.0, 1.0, 0.0); //目標位置を2次遅れ系にする
  fil_y.setSecondOrderPara(22.0, 1.0, 0.0);
  Low_vel.setSecondOrderPara(22.0, 1.0, 0.0);
  High_vel.setSecondOrderPara(22.0, 1.0, 0.0);

  manualCon.init();
  platform.platformInit(gPosi);

  autonomous.initSettings();            // これをやっていないと足回りの指令速度生成しない
  autonomous.setConvPara(0.02, 0.997);  // 初期化
  autonomous.setMaxPathnum(actpathnum); // パス数の最大値

  LEDblink(PIN_LED_GREEN, 2, 100);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  // 10msに1回ピン情報を出力する
  if (flag_10ms)
  {
    bool conUpdate = CON.update(); // コントローラからの受信

    //ユーザ編集部分 >>>>>>>>>>>>>>>>>
    static double speed = 1.0;
    static double posi_x = 0.0;
    static double posi_y = 0.0;
    static double ref_angle = 0.0;
    static double ref_vel_x = 0.0;
    static double ref_vel_y = 0.0;
    static double ref_posi_x = 0.0;
    static double ref_posi_y = 0.0;
    static double fil_ref_posi_x = 0.0;
    static double fil_ref_posi_y = 0.0;
    static double fil_mode_vel = 0.0;
    static int count_pad = 0;
    if (CON.readButton(BUTTON_PAD) == 2)
    {
      autonomous.phase = 4;
      count_pad++;
    }
    // 軌道追従制御させるための処理 >>>>
    if (autonomous.phase == 0 || autonomous.phase == 1 || autonomous.phase == 2 || autonomous.phase == 3)
    {
      //analogWrite(PIN_LED_USER, 255);
      coords refV_tra = autonomous.getRefVel(CON.getButtonState()); // 各目標点に対する位置決め動作を生成
      platform.VelocityControl(refV_tra);                           // 目標速度に応じて，プラットフォームを制御
    }

    // <<<<

    if (autonomous.phase == 3 || count_pad == 2)
    {
      //手動操作するための処理
      //倍速モード
      analogWrite(PIN_LED_USER, 255);
      if (CON.readButton(BUTTON_L2) == 1)
        speed = 1.8;
      else if (CON.readButton(BUTTON_R2) == 1)
        speed = 0.3;
      else
      {
        speed = 1.0;
      }

      fil_mode_vel = Low_vel.SecondOrderLag(speed);
      coords robot_vel;
      robot_vel.x = (gPosi.x - pre_gPosi.x) / INT_TIME;
      robot_vel.y = (gPosi.y - pre_gPosi.y) / INT_TIME;
      coords refV = manualCon.getRefVel(CON.readJoyLXbyte(), CON.readJoyLYbyte(), CON.readJoyRYbyte());
      if (CON.readJoyRYbyte() >= 137)
        ref_angle += (((double)CON.readJoyRYbyte() - 137) * (0.009 / 118));
      else if (CON.readJoyRYbyte() <= 117)
        ref_angle += (((double)CON.readJoyRYbyte() - 117) * (0.006 / 118));
      else
      {
        ref_angle += 0.0;
      }
      static bool flag = false;

      if (CON.readButton(BUTTON_L1) == 1)
      {
        analogWrite(PIN_LED_USER, 0);
        flag = true;
        refV.z = -refV.y / pow(0.1444 + 0.04, 0.5);
        refV.x = -refV.y * (0.2 / pow(0.1444 + 0.04, 0.5));

        fil_ref_posi_x = fil_x.SecondOrderLag(ref_posi_x);
        fil_ref_posi_y = fil_y.SecondOrderLag(ref_posi_y);

        //coords gol_con_vel;
        //gol_con_vel.x = {refV.x*cos(gPosi.z)+(refV.y*sin(gPosi.z))};
        //gol_con_vel.y = {-1*refV.x*sin(gPosi.z)+(refV.y)*cos(gPosi.z)};//座標系をフィールドに固定する
        if (CON.readButton(BUTTON_R1) == 1)
        {
          refV.x = 0.0;
          refV.z = -refV.y;
        }
      }
      if (CON.readButton(BUTTON_SHARE) == 2)
      {
        count_pad = 0;
        autonomous.phase = 0;
        digitalWrite(PIN_LED_1, LOW);
        digitalWrite(PIN_LED_2, LOW);
        digitalWrite(PIN_LED_3, LOW);
        digitalWrite(PIN_LED_4, LOW);
        autonomous.gPosiInit();
      }
      else
      {
        //analogWrite(PIN_LED_USER, 0);
        if (flag)
        {
          ref_angle = gPosi.z;
          flag = false;
        }
        refV.z = angle_posi.getCmd(ref_angle, gPosi.z, 3.14);
      }
      //refV.x= -refV.y*0.4;
      coords refV_d = {refV.x * fil_mode_vel, refV.y * fil_mode_vel, refV.z * -1 * fil_mode_vel};
      coords refV_g;
      //refV.x = PID_x.getCmd(ref_vel_x,robot_vel.x,1.0);//test用
      //refV.y = PID_y.getCmd(ref_vel_y,robot_vel.y,1.0);//test用
      //refV.x = POSI_x.getCmd(fil_ref_posi_x,gPosi.x,1.1);
      //refV.y = POSI_y.getCmd(fil_ref_posi_y,gPosi.y,1.1);
      //refV_g.x = PID_x.getCmd(refV_d.x, robot_vel.x, speed + 0.2); //速度PID x軸
      //refV_g.y = PID_y.getCmd(refV_d.y, robot_vel.y, speed + 0.2); //速度PID x軸
      //refV_g.z = -1 * refV_d.z;
      platform.VelocityControl(refV_d);

      pre_gPosi = gPosi; //前の値参照
    }
    // 上半身との通信部分
    /*if (conUpdate)
    {
      UpperBody.send((uint16_t)CON.getButtonState(), 0x44, 0x00);
    }
    if (UpperBody.recv())
    {
      //Serial.print("recv:");
      //Serial.print(UpperBody.recvOrder);
    }*/

    // シリアル出力する
    /*Serial.print(CON.readButton_bin(BUTTON_L2));
    Serial.print(" ");
    Serial.print(CON.readButton_bin(BUTTON_R2));
    Serial.print(" ");
    Serial.print(CON.readButton_bin(BUTTON_L1));
    Serial.print(" ");
    Serial.print(CON.readButton_bin(BUTTON_R1));
    Serial.print(" ");
    Serial.print(CON.readJoyLYbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyLXbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyRYbyte());
    Serial.print(" ");
    Serial.print(refV_g.x);
    Serial.print(" ");
    Serial.print(refV_g.y);
    Serial.print(" ");
    Serial.println(refV_g.z);*/
    //Serial.print(" ");
    Serial.println(count_pad);
    //Serial.print(" ");
    //Serial.print(refV_d.y);
    //Serial.print(" ");
    //Serial.println(refV_d.z);
    //Serial.print(ref_posi_y);
    /*Serial.print("\t");
    Serial.print(ref_posi_x);
    Serial.print("\t");
    Serial.print(fil_ref_posi_y);
    Serial.print("\t");
    Serial.println(fil_ref_posi_x);*/

    // SDカードにログを吐く
    if (SDwrite)
    { // 変数が追加されています!!!!
      String dataString = "";
      static bool first_write = true;
      if (first_write)
      {
        dataString += "gPosix,gPosiy,gPosiz,gRefVx,gRefVy,gRefVz";
        mySD.write_logdata(dataString);
        first_write = false;
        dataString = "";
      }
      //dataString += String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
      //dataString += "," + String(gRefV.x, 4) + "," + String(gRefV.y, 4) + "," + String(gRefV.z, 4);
      dataString += String(gPosi.x) + "," + String(gPosi.y);
      mySD.write_logdata(dataString);
    }

    //ユーザ編集部分 <<<<<<<<<<<<<<<<<

    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if (flag_100ms)
  {
    send_state(); // M5stackへ状態の送信 ※不要な場合はコメントアウトを

    //ユーザ編集部分 >>>>>>>>>>>>>>>>>

    //ユーザ編集部分 <<<<<<<<<<<<<<<<<

    flag_100ms = false;
  }
}

// M5stackに送るデータ
void send_state()
{
  unsigned int checksum = 0;
  char sendStr[25] = {0};
  int sendaData[6] = {(int)(gPosi.x * 100), (int)(gPosi.y * 100), (int)(gPosi.z * 100), (int)(gRefV.x * 100), (int)(gRefV.y * 100), (int)(gRefV.z * 100)};
  bool flagMinus[6] = {0};

  for (int i = 0; i < 6; i++)
  {
    if (sendaData[i] < 0)
    {
      flagMinus[i] = true;
      sendaData[i] = abs(sendaData[i]);
    }
  }
  sendStr[0] = 0; // ゼロで状態送信を指定
  sendStr[1] = robotState;
  sendStr[2] = sendaData[0] & 0x3F;
  sendStr[3] = (sendaData[0] >> 6) & 0x1F;
  if (flagMinus[0])
    sendStr[3] |= 0x20;

  sendStr[4] = sendaData[1] & 0x3F;
  sendStr[5] = (sendaData[1] >> 6) & 0x1F;
  if (flagMinus[1])
    sendStr[5] |= 0x20;

  sendStr[6] = sendaData[2] & 0x3F;
  sendStr[7] = (sendaData[2] >> 6) & 0x1F;
  if (flagMinus[2])
    sendStr[7] |= 0x20;

  sendStr[8] = sendaData[3] & 0x3F;
  sendStr[9] = (sendaData[3] >> 6) & 0x1F;
  if (flagMinus[3])
    sendStr[9] |= 0x20;

  sendStr[10] = sendaData[4] & 0x3F;
  sendStr[11] = (sendaData[4] >> 6) & 0x1F;
  if (flagMinus[4])
    sendStr[11] |= 0x20;

  sendStr[12] = sendaData[5] & 0x3F;
  sendStr[13] = (sendaData[5] >> 6) & 0x1F;
  if (flagMinus[5])
    sendStr[13] |= 0x20;

  sendStr[14] = (int)(CON.readJoyLXbyte() * 0.247) & 0x3F;
  sendStr[15] = (int)(CON.readJoyLYbyte() * 0.247) & 0x3F;
  sendStr[16] = (int)(CON.readJoyRXbyte() * 0.247) & 0x3F;
  sendStr[17] = (int)(CON.readJoyRYbyte() * 0.247) & 0x3F;

  unsigned int ButtonState = CON.getButtonState(); // コントローラデータ格納用
  sendStr[18] = ButtonState & 0x3F;
  sendStr[19] = (ButtonState >> 6) & 0x3F;
  sendStr[20] = (ButtonState >> 12) & 0x3F; // ここはボタン数によって書き換える

  for (int i = 0; i < 21; i++)
  {
    checksum += (unsigned int)sendStr[i];
    SERIAL_M5STACK.write(sendStr[i] + 0x20);
  }
  sendStr[20] = (char)checksum & 0x3F;
  SERIAL_M5STACK.write(sendStr[20] + 0x20);
  //SERIAL_M5STACK.write((checksum & 0x3F) + 0x20);
  SERIAL_M5STACK.print("\n");
}