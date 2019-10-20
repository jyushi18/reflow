/*************************************************** 
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <MsTimer2.h>


// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

int fan = 12;//ファンのピン番号
int heater = 11;//ヒーターのピン番号

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Example creating a thermocouple instance with hardware SPI
// on a given CS pin.
//#define MAXCS   10
//Adafruit_MAX31855 thermocouple(MAXCS);
boolean flag_start = LOW;

int power_freq = 50;//交流電源の周波数
int control_len = 50;//制御周期の長さ
double T_s = (1/double(power_freq))*double(control_len);//計測周期
int flashcounter = 0;//フラッシュ関数の呼び出し回数

boolean out_h = HIGH;
double ref_c=180;//温度リファレンス
double err_c;//目標値との差
double Kp=10;//比例ゲイン
double Kd=70;//微分ゲイン
double Ki=0;//積分ゲイン
double power_duty = 0;//Duty比
int power_count;//Duty比から計算したHIGHの回数
double err_c_d1 = 25;//１つ前の目標値との差,d=delay
double err_c_d2 = 25;//2つ前の目標値との差,d=delay
double power_duty_d1 = 0;//１つ前のduty比
double delta_power_duty;
double c_d1 = 0;//１つ前の温度

double time_s = 0;

void flash(){
  if(time_s < 180){
    ref_c=180;
  }else if(time_s >= 180 && time_s < 250){
    ref_c=250;
  }else if(time_s >= 250){
    ref_c=0;
    digitalWrite(heater,LOW);
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

  MsTimer2::set(20, flash); // 500ms period
  
  
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