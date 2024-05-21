#include <Arduino.h>
#include "SPIFFS.h"
#include <ESPAsyncWebServer.h>
#include "ESP32Servo/ESP32Servo.h"

#define MOTOR_A 27
#define MOTOR_B 26
#define MOTOR_C 25

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

AsyncWebServer server(80);

Servo motorA;
Servo motorB;
Servo motorC;

void setup() {
  motorA.attach(MOTOR_A);
  motorB.attach(MOTOR_B);
  motorC.attach(MOTOR_C);

  Serial.begin(115200);
  Serial.println("Hello ");

  if (!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

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
}

void loop() {
  if(millis() - last_packet < 1000){
    val_x = (float)joy_x / 100.0F;
    val_y = (float)joy_y / 100.0F;
    get_speed(val_x, -1 * val_y, (float)joy_w);
  } else get_speed(0.0F, 0.0F, 0.0F);
}


void get_speed(float x, float y , float w) {
  motorSpeedA = ( (x * (0.67)) + (y * 0) + (w * 0.33) );
  motorSpeedB = ( (x * (-0.33f)) + (y * (-0.58f)) + (w * 0.33) );
  motorSpeedC = ( (x * (-0.33f)) + (y * (0.58f))  + (w * 0.33) );

  mA_Speed = motorSpeedA * max_pwm;
  mB_Speed = motorSpeedB * max_pwm;
  mC_Speed = motorSpeedC * max_pwm;

  motorA.write(mA_Speed);
  motorB.write(mB_Speed);
  motorC.write(mC_Speed);
}