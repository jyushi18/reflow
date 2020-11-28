/*************************************************** 
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <MsTimer2.h>

#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

#define fan     12        //ファンのピン番号
#define heater  11        //ヒーターのピン番号
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

boolean flag_start = LOW; //リフローのスタート
int power_freq = 50;      //交流電源の周波数
int control_len = 50;     //制御周期の長さ
double T_s = (1/double(power_freq))*double(control_len);//計測周期
int flashcounter = 0;     //フラッシュ関数の呼び出し回数
double err_c;             //目標値との差
double power_duty = 0;    //Duty比
int power_count;          //Duty比から計算したHIGHの回数
double err_c_d1 = 25;     //１つ前の目標値との差,d=delay
double err_c_d2 = 25;     //2つ前の目標値との差,d=delay
double power_duty_d1 = 0; //１つ前のduty比
double delta_power_duty;
double c_d1 = 0;          //１つ前の温度
double time_s = 0;

/////////////////////////////////////////
// PID制御
double Kp=10;             //比例ゲイン
double Kd=70;             //微分ゲイン
double Ki=0;              //積分ゲイン

/////////////////////////////////////////
// 温度設定
#define pre_temp    150     //予熱温度．鉛入りだと150度くらい
#define peak_temp   230     //本加熱．鉛入りだと(数度高めに)230度くらい
#define PreHeatTime 180     //予熱の時間
#define HeatEndTime 250     //本加熱の終了時間
double ref_c=pre_temp;    //温度リファレンス初期値（余熱温度と等しくすること！！）
/////////////////////////////////////////

void flash(){
  if(time_s < PreHeatTime){
    ref_c=pre_temp;
  }else if(time_s >= PreHeatTime && time_s < HeatEndTime){
    ref_c=peak_temp;
  }else if(time_s >= HeatEndTime){
    ref_c=0;
    digitalWrite(heater,LOW);
    Serial.println("Finish! You should door open!");
  }
  
  if(flashcounter%control_len == 0){
    double c = thermocouple.readCelsius();
    if(isnan(c)){
      c = c_d1;
    }
    Serial.print(time_s);
    Serial.print(",");
    Serial.print(c);
    Serial.print(",");
    //Serial.println("1");
    err_c = ref_c - c;
    delta_power_duty = (Kp*(err_c - err_c_d1) + Ki*err_c*T_s + (err_c-2*err_c_d1+err_c_d2)*Kd/T_s);//速度型PID制御
    power_duty = power_duty_d1 + delta_power_duty;
    power_duty_d1 = power_duty;
    Serial.print(power_duty);
    Serial.print(",");
    //power_duty = (err_c*Kp - (err_c-err_c_d1)*Kd);
    power_duty = min(100.0,max(0,power_duty))/100.0;
    power_count = int(power_duty*control_len);
    Serial.println(power_count);
    err_c_d2 = err_c_d1;
    err_c_d1 = err_c;
    c_d1 = c;
  }
  if(power_count > 0){
    //Serial.println(power_count);
    digitalWrite(heater,HIGH);
    power_count--;
  }else{
    digitalWrite(heater,LOW);
  }
  flashcounter++;
  time_s = double(flashcounter)*(1/double(power_freq));
}

void setup() {
  Serial.begin(9600);
 
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  delay(500);

  MsTimer2::set(20, flash); // 20ms period
  
  
  //Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  pinMode(heater, OUTPUT); //ヒーター
  pinMode(fan, OUTPUT); //ファン
}

void loop() {
  int inputchar;      //入力状態の読み取りに使う
  inputchar = Serial.read();  //シリアル通信で送信された値を読み取る
  if(inputchar!=-1){
    switch(inputchar){
      case 's':
        digitalWrite(fan,HIGH);
        MsTimer2::start();
        flag_start = HIGH;
        break;
      case 'f':
        digitalWrite(fan,HIGH);
        digitalWrite(heater,LOW);
        MsTimer2::stop();
        flag_start = LOW;
        break;
      case 'q':
        digitalWrite(fan,LOW);
        digitalWrite(heater,LOW);
        MsTimer2::stop();
        flag_start = LOW;
        break;
    }
  }
  
  if(flag_start == LOW){
    double c = thermocouple.readCelsius();
    Serial.println(c);
    c_d1 = c;
    err_c_d2 = 0;
    err_c_d1 = 0;
    delay(int(T_s*1000));

  }
}
