#include <Arduino.h>
#include "SPIFFS.h"
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// #define ENABLE_ODOMETRY

#define PWM_RES 8
#define PWM_FREQ 1000
#define PWM_SETUP ledcSetup
#define PWM_ATTACH ledcAttachPin
#define SET_PWM ledcWrite

#define MA_CH 0
#define MB_CH 1
#define MC_CH 2
#define MD_CH 3
#define LIDAR_CH 4

#define MOTOR_A_PWM 26
#define MOTOR_B_PWM 27
#define MOTOR_C_PWM 5
#define MOTOR_D_PWM 23

#define LIDAR_PWM 25

#define MOTOR_A_DIR 2
#define MOTOR_B_DIR 4
#define MOTOR_C_DIR 32
#define MOTOR_D_DIR 33

#define MA_ENC_C1 13
#define MA_ENC_C2 39 //12

#define MB_ENC_C1 14
#define MB_ENC_C2 15

#define MC_ENC_C1 34
#define MC_ENC_C2 35

#define MD_ENC_C1 18
#define MD_ENC_C2 19

unsigned long last_packet = 0;

const char *ssid = "Test";
const char *password = "123456789";

const char* PARAM_INPUT_1 = "valX";
const char* PARAM_INPUT_2 = "valY";
const char* PARAM_INPUT_3 = "valW";

String inputMessage1;
String inputMessage2;
String inputMessage3;

int joy_x = 0, joy_y = 0, joy_w = 0;
float max_pwm = 90;
float val_x = 0.0F, val_y = 0.0F, val_w = 0.0f;
float mA_Speed, mB_Speed, mC_Speed, motorSpeedA, motorSpeedB, motorSpeedC;
void get_speed(float x, float y , float w);

volatile int count_A, count_B, count_C, count_D;

void drive();
void isr_A();
void isr_B();
void isr_C();
void isr_D();

bool keepSpinning = true;
//uint16_t debugPrintCounter = 0;
//const uint16_t debugPrintThreshold = 48; // print data every (this many) datapoints (if you are getting CRC errors, there may be buffer overflow, try setting this to like 48+ (or uncommenting printing entirely))
uint32_t debugPrintTimer;
const uint32_t dubugPrintInterval = 5000; // micros between prints

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_C_PWM, OUTPUT);
  pinMode(MOTOR_D_PWM, OUTPUT);

  pinMode(LIDAR_PWM, OUTPUT);
  
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_C_DIR, OUTPUT);
  pinMode(MOTOR_D_DIR, OUTPUT);

  PWM_SETUP(MA_CH, PWM_FREQ, PWM_RES);
  PWM_SETUP(MB_CH, PWM_FREQ, PWM_RES);
  PWM_SETUP(MC_CH, PWM_FREQ, PWM_RES);
  PWM_SETUP(MD_CH, PWM_FREQ, PWM_RES);
  
  PWM_SETUP(LIDAR_CH, PWM_FREQ, PWM_RES);

  PWM_ATTACH(MOTOR_A_PWM, MA_CH);
  PWM_ATTACH(MOTOR_B_PWM, MB_CH);
  PWM_ATTACH(MOTOR_C_PWM, MC_CH);
  PWM_ATTACH(MOTOR_D_PWM, MD_CH);
  PWM_ATTACH(LIDAR_PWM, LIDAR_CH);

  // pinMode(MA_ENC_C2, INPUT);
  pinMode(MA_ENC_C1, INPUT_PULLUP);
  pinMode(MA_ENC_C2, INPUT);  
#ifdef ENABLE_ODOMETRY

  attachInterrupt(digitalPinToInterrupt(MA_ENC_C1), isr_A, FALLING);
  
  pinMode(MB_ENC_C1, INPUT_PULLUP);
  pinMode(MB_ENC_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MB_ENC_C1), isr_B, FALLING);

  pinMode(MC_ENC_C1, INPUT_PULLUP);
  pinMode(MC_ENC_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MC_ENC_C1), isr_C, FALLING);

  pinMode(MD_ENC_C1, INPUT_PULLUP);
  pinMode(MD_ENC_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MD_ENC_C1), isr_D, FALLING);
#endif

  digitalWrite(MOTOR_A_DIR, LOW);
  digitalWrite(MOTOR_B_DIR, LOW);
  digitalWrite(MOTOR_C_DIR, LOW);
  digitalWrite(MOTOR_D_DIR, LOW);

  SET_PWM(MA_CH, 0);
  SET_PWM(MB_CH, 0);
  SET_PWM(MC_CH, 0);
  SET_PWM(MD_CH, 0);
  SET_PWM(LIDAR_CH, 255);

  Serial.println("PWM Setup Done");

// Server Code
  if (!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("SPIFFS Setup Done");

  // uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.println("WiFi TX Power Limited");

  WiFi.softAP(ssid, password);
  // Connect to Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi..");
  // }
  
  Serial.println(WiFi.localIP());

  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/joy.html", "text/html"); });

  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2) && request->hasParam(PARAM_INPUT_3)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      inputMessage3 = request->getParam(PARAM_INPUT_3)->value();
      joy_x = inputMessage1.toInt();
      joy_y = inputMessage2.toInt();
      joy_w = inputMessage3.toInt();
      last_packet = millis();
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
      inputMessage3 = "No message sent";
     }
    request->send(200, "text/plain", "OK");
  });

  server.begin();

  Serial.println("[eBot Mini] Esp32 Dev Board Initialized");
}

void loop() {
  // Serial.print("A: ");Serial.print(count_A);
  // Serial.print(" B: ");Serial.print(count_B);
  // Serial.print(" C: ");Serial.print(count_C);
  // Serial.print(" D: ");Serial.println(count_D);

  if(millis() - last_packet < 1000){
    val_x = (float)joy_x / 100.0F;
    val_y = (float)joy_y / 100.0F;

    if(abs(joy_y) > 50 && abs(joy_x) < 50){
      mA_Speed = val_y * 200;

      digitalWrite(MOTOR_A_DIR, mA_Speed < 0 ? LOW : HIGH);
      digitalWrite(MOTOR_B_DIR, mA_Speed < 0 ? LOW : HIGH);
      digitalWrite(MOTOR_C_DIR, mA_Speed < 0 ? LOW : HIGH);
      digitalWrite(MOTOR_D_DIR, mA_Speed < 0 ? LOW : HIGH);
 
      if(mA_Speed < 0) mA_Speed = -1 * mA_Speed;

      SET_PWM(MA_CH, mA_Speed);
      SET_PWM(MB_CH, mA_Speed);
      SET_PWM(MC_CH, mA_Speed);
      SET_PWM(MD_CH, mA_Speed);
    } else if(abs(joy_x) > 50 && abs(joy_y) < 50){
        mA_Speed = val_x * 200;

        digitalWrite(MOTOR_A_DIR, mA_Speed < 0 ? HIGH : LOW);
        digitalWrite(MOTOR_B_DIR, mA_Speed < 0 ? LOW : HIGH);
        digitalWrite(MOTOR_C_DIR, mA_Speed < 0 ? HIGH : LOW);
        digitalWrite(MOTOR_D_DIR, mA_Speed < 0 ? LOW : HIGH);

        if(mA_Speed < 0) mA_Speed = -1 * mA_Speed;

        SET_PWM(MA_CH, mA_Speed);
        SET_PWM(MB_CH, mA_Speed);
        SET_PWM(MC_CH, mA_Speed);
        SET_PWM(MD_CH, mA_Speed);
    } else {
        digitalWrite(MOTOR_A_DIR, LOW);
        digitalWrite(MOTOR_B_DIR, LOW);
        digitalWrite(MOTOR_C_DIR, LOW);
        digitalWrite(MOTOR_D_DIR, LOW);

        SET_PWM(MA_CH, 0);
        SET_PWM(MB_CH, 0);
        SET_PWM(MC_CH, 0);
        SET_PWM(MD_CH, 0);      
    }
  } else{
      digitalWrite(MOTOR_A_DIR, LOW);
      digitalWrite(MOTOR_B_DIR, LOW);
      digitalWrite(MOTOR_C_DIR, LOW);
      digitalWrite(MOTOR_D_DIR, LOW);

      SET_PWM(MA_CH, 80);
      SET_PWM(MB_CH, 80);
      SET_PWM(MC_CH, 80);
      SET_PWM(MD_CH, 80);
  } 
}

void drive(){

}

void get_speed(float x, float y , float w) {
  motorSpeedA = ( (x * (0.67)) + (y * 0) + (w * 0.33) );
  motorSpeedB = ( (x * (-0.33f)) + (y * (-0.58f)) + (w * 0.33) );
  motorSpeedC = ( (x * (-0.33f)) + (y * (0.58f))  + (w * 0.33) );

  mA_Speed = motorSpeedA * max_pwm;
  mB_Speed = motorSpeedB * max_pwm;
  mC_Speed = motorSpeedC * max_pwm;
}


void isr_A(){
  if(digitalRead(MA_ENC_C2)) count_A--;
  else count_A++;
}

void isr_B(){
  if(digitalRead(MB_ENC_C2)) count_B--;
  else count_B++;
}

void isr_C(){
  if(digitalRead(MC_ENC_C2)) count_C--;
  else count_C++;
}


void isr_D(){
  if(digitalRead(MD_ENC_C2)) count_D--;
  else count_D++;
}