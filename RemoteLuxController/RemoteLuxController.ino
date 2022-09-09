// Author: Yutaka Yamamoto
// Github Repo: https://github.com/yutaka-pana/RemoteLuxController.git
// Description: IP controlled automatic lux controller.
//              Get environment light strength from Lux sensor and 
//              controll external lights to meet ordered Lux parameter.
// SCL は analog 5 に接続してください
// SDA は analog 4 に接続してください
#include "TSL2561.h"
#include <SPI.h>
#include <Ethernet.h>

#define VERSION "1.1.0"

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
//- GPIO
#define PWMPORT 3
//- シリアル定義
#define SERIAL_EN true
#define SERIAL_RATE 230400
//- Ethernetシールド
byte mac_address[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
byte IP_address[] = {
  192,168,1,110
};
byte dns_address[] = {
  192,168,1,1
};
byte gateway_address[] = {
  192,168,1,1
};
byte subnet[] = {
  255,255,255,0
};
//IPAddress ip(192, 168, 1, 110);
EthernetServer server(80);
String pathname="";
String query="";

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

/// API処理：クエリーに対応する値があれば返す
String getQuery(String command)
{
  if(query.indexOf(command + "=") > -1){
    int num = query.indexOf(command + "=");
    int start = num + command.length() + 1;
    int end = query.indexOf("&", start);
    if(end == -1){
      end = query.length();
    }
    return query.substring(start, end);
  }
  return ""; 
}

///Ether受信モジュール
String Ether_RX(){
  String target;
  EthernetClient client = server.available();
  if(client){
    boolean currentLineIsBlank = true;
    boolean isGETLine = true;
    String header = "";
    String headerGET = "";
    while(client.connected()){
      //Serial.println("Ether: ClientAccess!");
      if(client.available()){
        //Serial.println("Ether: ClientHello!");
        char c = client.read();
        if(isGETLine){
          if(c=='\n'){
            isGETLine =false;
            Serial.println(header);
          }else{
            header += String(c);
          }
        }
        if (c == '\n' && currentLineIsBlank) {
          headerGET = header.substring(header.indexOf(" ")+1, header.lastIndexOf(" "));
          // favicon.ico対策
          if(headerGET == "/favicon.ico"){
            client.println("HTTP/1.1 204 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          pathname = headerGET.substring(0, headerGET.indexOf("?"));
          query = headerGET.indexOf("?") > -1?headerGET.substring(headerGET.indexOf("?")+1):"";
          Serial.println("pathname=" + pathname + ", query=" + query);

          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          //client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          //client.println();
          //client.println("<!DOCTYPE HTML>");
          //client.println("<html>");
          
          // output the value of each analog input pin
          //for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
          //  int sensorReading = analogRead(analogChannel);
          //  client.print("analog input ");
          //  client.print(analogChannel);
          //  client.print(" is ");
          //  client.print(sensorReading);
          //  client.println("<br />");
          //}
          // ディレクトリやクエリーで分けてみたらいかが？提案用
          //client.println("<a href='../abc/?a=123&b=456'>../abc/?a=123&b=456</a>");
          //client.println("<br />");
          // API ぽく使いたい時はディレクトリ分けしたら良いかも
          if(pathname == "/cont/"){
            target = getQuery("target");
            if(target != ""){
             Serial.println("target=" + target);
            }
          }
//        client.println("<br />");
//        client.println("pathname:" + pathname + ", query:" + query);
//        client.println("</html>");
          break;
        }
        if (c == '\n') {
        // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
        // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    delay(10);
    client.stop();
  }
  return target;
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
int rxDatavalidation_target(String data){
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
     //Serial.println("!!!INVALID LENGTH!!!");
  }
  return retVal;
}

///指示入力受信モジュール
int target_RX(){
  //return  rxDatavalidation_target(Serial_RX());
  return  rxDatavalidation_target(Ether_RX());
}

void setup() {
  //Setup Serial
  Serial.begin(SERIAL_RATE);
  Serial.println("PowerActivated waiting 4 Ethernet StartUP!");
  //Setup Ethernet
  SPI.begin() ;
  SPI.setBitOrder(MSBFIRST) ;
  SPI.setClockDivider(SPI_CLOCK_DIV4) ;
  SPI.setDataMode(SPI_MODE0) ;
  Ethernet.begin(mac_address, IP_address, dns_address, gateway_address, subnet);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      Serial.println("Reset in 5 seconds...");
      delay(5000); // do nothing, no point running without Ethernet hardware
      resetFunc();
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  server.begin();
  Serial.print("WebServer is at ");
  Serial.println(Ethernet.localIP());
  
  //WelcomeMessage
  Serial.println("RemoteLuxController Initializing!");
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
    Serial.println("Reset in 5 seconds...");
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
    Serial.println("TARGET CHANGED!->" + intTemp);
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
