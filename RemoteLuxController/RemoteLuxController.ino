// Author: Yutaka Yamamoto
// Github Repo: https://github.com/yutaka-pana/RemoteLuxController.git
// Description: IP controlled automatic lux controller.
//              Get environment light strength from Lux sensor and 
//              controll external lights to meet ordered Lux parameter.
// SCL は analog 5 に接続してください
// SDA は analog 4 に接続してください
#include "TSL2561.h"

#define VERSION "1.0.0"

//-- 出力設定
//- PWM
#define PWM_INVERT true
#define PWM_MIN 0
#define PWM_MAX 255 //255固定
#define DUTY2PWM 2.56
#define ORDER2PWM 0.128
//- コントローラ
#define CONT_MIN -1000
#define CONT_MAX 1000
#define GAIN_P 0.1
#define GAIN_D 0
#define GAIN_I 0
#define LUX_MIN 10
#define LUX_MAX 30000

//-- IO定義
#define SERIAL_EN true
#define SERIAL_RATE 9600
#define PWMPORT 3

//-- センサ変数設定
//- アドレス設定
//TSL2561 tsl(TSL2561_ADDR_LOW);   //(0x29)
//TSL2561 tsl(TSL2561_ADDR_FLOAT); //(0x39)
TSL2561 tsl(TSL2561_ADDR_HIGH);  //(0x49)
//- ゲイン設定
#define TSL2561_GAIN TSL2561_GAIN_0X 
//#define TSL2561_GAIN TSL2561_GAIN_16X
//- 測定値積算時間
#define TSL2561_INT TSL2561_INTEGRATIONTIME_13MS
//#define TSL2561_INT TSL2561_INTEGRATIONTIME_101MS
//#define TSL2561_INT TSL2561_INTEGRATIONTIME_402MS

///グローバル変数
uint32_t diffAccum;
int target;
int pwmPrev;

///リセット関数ポインタ
void(* resetFunc)(void) = 0;

///センサデータ受信モジュール
uint32_t lxSensor_RX(){
  return tsl.getFullLuminosity();
}

///センサ値変換モジュール（LUX）
uint32_t lxSensor_getLux(uint32_t lxSensorVal){
  uint16_t full, ir;
  ir = lxSensorVal >> 16;
  full = lxSensorVal & 0xFFFF;
  return tsl.calculateLux(full, ir);
}

///センサ値フィルタモジュール
uint32_t lxSensor_filter(uint32_t luxVal){
  return luxVal;
}

///コントローラ
double controller_PID(uint32_t sense, uint32_t sense_prev, uint32_t* diff_accum, uint32_t target){
  double cont_P = 0, cont_D = 0, cont_I = 0, cont_total;

  cont_P = ((double)target - (double)sense) * GAIN_P;
  cont_D = 0;
  cont_I = 0;
  Serial.print(", TARGET:");
  Serial.print(target);
  Serial.print(", CONT_P:");
  Serial.print(cont_P);
  Serial.print(", CONT_D:");
  Serial.print(cont_D);
  Serial.print(", CONT_I:");
  Serial.print(cont_I);

  cont_total = cont_P + cont_D + cont_I;
  if(cont_total > CONT_MAX){
    cont_total = CONT_MAX;
  }else if(cont_total < CONT_MIN){
    cont_total = CONT_MIN;
  }else{
    cont_total = cont_total;
  }

  Serial.print(", CONT_OUT:");
  Serial.print(cont_total);

  return cont_total;
}

///PWM出力調整モジュール
int pwmOut_filter(int rawPwm){
  int pwmOut;
  if(rawPwm < PWM_MIN){
    pwmOut = PWM_MIN;
  }else if(rawPwm > PWM_MAX){
    pwmOut = PWM_MAX;
  }else{
    pwmOut=rawPwm;
  }
  return pwmOut;
}

///PWM出力値生成モジュール
int pwmOut_getPwm(double order, double scale, int pwmPrev){
  int pwmFactor,pwmOut;

  pwmFactor = (int)(order * scale);
  Serial.print(", PWM_F:");
  Serial.print(pwmFactor);
  
  pwmOut = pwmPrev + pwmFactor;
  Serial.print(", PWM_OUT:");
  Serial.print(pwmOut);
  
  return pwmOut_filter(pwmOut);
}

///PWM出力モジュール
void pwmOut_TX(int pwm){
  if(PWM_INVERT){
    pwm = pwmOut_filter(255 - pwm);
  }else{
    pwm = pwm;
  }

  analogWrite(PWMPORT, pwm);
}

///シリアル受信モジュール
String Serial_RX(){
  String data;
  if ( Serial.available()){
    data = Serial.readStringUntil('\n');

    Serial.print("Receive:");
    Serial.println(data);
  }else{
    data = "";
  }
  return data;
}

///シリアル入力フィルタモジュール
int serialIn_validation(String data){
  int retVal = -1;
  int data_len = data.length();

  if((data_len >= 1)&&(data_len <= 5)){
    int get_lux = data.toInt();
    if(get_lux < LUX_MIN || get_lux > LUX_MAX){
      Serial.println("!!!INVALID VALUE!!!");
      retVal = -1;
    }else{
      retVal = get_lux;
    }
  }else{
     Serial.println("!!!INVALID LENGTH!!!");
  }
  return retVal;
}

///指示入力受信モジュール
int target_RX(){
  return serialIn_validation(Serial_RX());
}

void setup() {
  //Setup Serial
  Serial.begin(SERIAL_RATE);
  //WelcomeMessage
  Serial.println("RemoteLuxController Startup!");
  Serial.print("Version.");
  Serial.println(VERSION); 
  Serial.println();
  Serial.println("---INSTRUCTION---");
  Serial.println("TYPE Duty (0 - 100) and Press Enter");
  //Initialize LuxSensor
  Serial.println("lxSensor_Connect(): SensorConnect");
  if(tsl.begin()){
    Serial.println("lxSensor_Connect(): Found Sensor");
  }else{
    Serial.println("lxSensor_Connect(): No sensor?");
    delay(5000);
    resetFunc();
  }
  tsl.setGain(TSL2561_GAIN);
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);
  //Setup Pin
  pinMode(PWMPORT, OUTPUT);
  pwmOut_TX(0);

  diffAccum = 0;
  target = 0;
  pwmPrev = 0;
}

void loop() {
  uint32_t uint32Temp;
  double doubleTemp;
  int intTemp;

  intTemp = target_RX();
  if(intTemp >= 0){
    target = intTemp;
  }else{
    target = target;
  }

  uint32Temp = lxSensor_RX();
  uint32Temp = lxSensor_getLux(uint32Temp);
  uint32Temp = lxSensor_filter(uint32Temp);
  Serial.print("LUX:");
  Serial.print(uint32Temp);

  doubleTemp = controller_PID(uint32Temp, uint32Temp, &diffAccum, target);
  Serial.print(", CONT:");
  Serial.print(doubleTemp);
  intTemp = pwmOut_getPwm(doubleTemp, ORDER2PWM, pwmPrev);
  pwmPrev = intTemp;
  pwmOut_TX(intTemp);
  Serial.print(", PWM:");
  Serial.println(intTemp); 

  delay(100);
}
