#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <Arduino.h>
#include <Wire.h>
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define MOTOR_A D0 
#define MOTOR_B D7
#define MOTOR_C D2
#define MOTOR_D D8

const char *ssid = "RTR_JOY202";
const char *password = "RTR@2021";

struct JoyPayload {
  float x;
  float y;
  float w;
};

typedef struct {
  float roll;
  float pitch;
  float yaw;
} imu_data_t;

int16_t GyroX = 0, GyroY = 0, GyroZ = 0;
int16_t AccXLSB = 0, AccYLSB = 0, AccZLSB = 0;
float gf_yaw = 0.0F;
float elapsedTime, currentTime, previousTime, currentTime_accel;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};

// PID state
float err_roll = 0, err_pitch = 0, err_yaw = 0;
float prev_err_roll = 0, prev_err_pitch = 0, prev_err_yaw = 0;
float I_roll = 0, I_pitch = 0, I_yaw = 0;

float kp = 0.0f, ki = 0.00f, kd = 0.0f;
unsigned long prev_time = 0;
unsigned long elapsed_time = 0;

void gyro_signals(void);
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void calculate_pid(imu_data_t data, float setpoint_roll, float setpoint_pitch, float setpoint_yaw);

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();

TaskHandle_t Task1;
TaskHandle_t Task2;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

uint8_t left_trigger = 0, right_trigger = 0;
bool ball_picking_flagA = false, ball_picking_flagB = false;
unsigned long acctuate_timerA, acctuate_timerB;

int joystickX = 0;
int joystickY = 0;
int throttle = 0;

u8_t motor_speed_A = 0;
u8_t motor_speed_B = 0;
u8_t motor_speed_C = 0;
u8_t motor_speed_D = 0;

QueueHandle_t imuQueue;
imu_data_t  imu_proceesed;
imu_data_t imu_raw;

void setup() {
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_D, OUTPUT);
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
  analogWrite(MOTOR_C, 0);
  analogWrite(MOTOR_D, 0);
  
  Serial.begin(115200);
 
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer = micros();

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/joy.html", "text/html"); });

  server.begin();

  imuQueue = xQueueCreate(10, sizeof(imu_data_t));
  if (imuQueue == NULL) {
    Serial.println("Failed to create IMU queue!");
    while (1); // Halt here to catch error
  }

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    4096,        /* Stack size of task */
    NULL,        /* parameter of the task */
    5,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    2048,        /* Stack size of task */
    NULL,        /* parameter of the task */
    4,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);

  if (!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
  }

}

//Task 1
void Task1code( void * pvParameters ) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println("Task1- Drone Control");

  for (;;) {
    ws.cleanupClients();

    if (!xQueueReceive(imuQueue, &imu_proceesed, pdMS_TO_TICKS(100)) == pdPASS) {
      Serial.println("No IMU data received in time.");
    }

    if(millis() - prev_time > 50) {
      calculate_pid(imu_proceesed, AngleRoll, AnglePitch, gf_yaw);
    }

    // noInterrupts();
    // interrupts();
    
    vTaskDelay(2);
    yield();
  }
}

// Task 2
void Task2code( void * pvParameters ) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println("Task2- Kalman Filter");

  for (;;) {
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0]; 
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0]; 
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    
    imu_raw.roll = KalmanAngleRoll;
    imu_raw.pitch = KalmanAnglePitch;
    imu_raw.yaw = gf_yaw;

    Serial.print("A:");Serial.print(motor_speed_A);Serial.print(" B:");Serial.print(motor_speed_B);
    Serial.print(" C:");Serial.print(motor_speed_C);Serial.print(" D:");Serial.print(motor_speed_D);
    Serial.print(" Throttle:");Serial.print(throttle);

    Serial.print("Roll: "); Serial.print(imu_raw.roll, 4);
    Serial.print(" Pitch: "); Serial.print(imu_raw.pitch, 4);
    Serial.print(" Yaw: "); Serial.println(imu_raw.yaw, 4);

    xQueueSend(imuQueue, &imu_raw, 0);

    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
   
  Wire.requestFrom(0x68,6);
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68,6);
  GyroX = Wire.read()<<8 | Wire.read();
  GyroY = Wire.read()<<8 | Wire.read();
  GyroZ = Wire.read()<<8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
//  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch =- atan(AccX/sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

  RateYaw = (float)GyroZ / 32.8;

  RateYaw = RateYaw - 0.72F; // GyroErrorZ ~ (-0.8)
  if (abs(RateYaw) > 2.0F) gf_yaw =  gf_yaw + RateYaw * (currentTime - previousTime) / 1000.0f;
  if (gf_yaw > 180) gf_yaw = -179;
  if (gf_yaw < -180) gf_yaw = 179;

//  gf_yaw = -1 * gf_yaw;
  AngleRoll = -1 * AngleRoll;
  AnglePitch = -1 * AnglePitch;
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t) 1);
  data = Wire.read();
  return data;
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;

  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0; // Null-terminate the input
    char *ptr = (char*)data;

    char *xPos = strstr(ptr, "\"X\":");
    char *yPos = strstr(ptr, "\"Y\":");
    char *wPos = strstr(ptr, "\"W\":");

    if (xPos && yPos && wPos) {
      joystickX = atoi(xPos + 4);
      joystickY = atoi(yPos + 4);
      throttle = atoi(wPos + 4);

      // Serial.printf("Joystick X: %d, Y: %d, W: %d\n", joystickX, joystickY, throttle);
    } else {
      Serial.println("Invalid JSON format");
    }
  }
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void calculate_pid(imu_data_t data, float setpoint_roll, float setpoint_pitch, float setpoint_yaw) {
  elapsed_time = (millis() - prev_time);

  // === PID on ROLL ===
  err_roll = setpoint_roll - data.roll;
  float P_roll = kp * err_roll;
  I_roll += ki * err_roll * elapsed_time;
  float D_roll = kd * (err_roll - prev_err_roll) / elapsed_time;
  float PID_roll = P_roll + I_roll + D_roll;

  // === PID on PITCH ===
  err_pitch = setpoint_pitch - data.pitch;
  float P_pitch = kp * err_pitch;
  I_pitch += ki * err_pitch * elapsed_time;
  float D_pitch = kd * (err_pitch - prev_err_pitch) / elapsed_time;
  float PID_pitch = P_pitch + I_pitch + D_pitch;

  // === PID on YAW ===
  err_yaw = setpoint_yaw - data.yaw;
  float P_yaw = kp * err_yaw;
  I_yaw += ki * err_yaw * elapsed_time;
  float D_yaw = kd * (err_yaw - prev_err_yaw) / elapsed_time;
  float PID_yaw = P_yaw + I_yaw + D_yaw;

  float motorA = throttle + PID_roll + PID_pitch;// - PID_yaw;
  float motorB = throttle - PID_roll + PID_pitch;// + PID_yaw;
  float motorC = throttle - PID_roll - PID_pitch;// - PID_yaw;
  float motorD = throttle + PID_roll - PID_pitch;// + PID_yaw;

  // if(motorA < 0) motorA = 0;
  // if(motorB < 0) motorB = 0;
  // if(motorC < 0) motorC = 0;
  // if(motorD < 0) motorD = 0;

  motor_speed_A = (int)motorA;
  motor_speed_B = (int)motorB;
  motor_speed_C = (int)motorC;
  motor_speed_D = (int)motorD;

  // Debugging output
  // Serial.print("A:");Serial.print(motor_speed_A, 3);Serial.print(" B:");Serial.print(motor_speed_B, 3);
  // Serial.print(" C:");Serial.print(motor_speed_C, 3);Serial.print(" D:");Serial.print(motor_speed_D, 3);
  // Serial.print(" Throttle:");Serial.println(throttle);
  // Serial.print(" Roll:");Serial.print(data.roll);Serial.print(" Pitch:");Serial.print(data.pitch);
  // Serial.print(" Yaw:");Serial.println(data.yaw);

  motor_speed_A = constrain(motor_speed_A, 0, 255);
  motor_speed_B = constrain(motor_speed_B, 0, 255);
  motor_speed_C = constrain(motor_speed_C, 0, 255);
  motor_speed_D = constrain(motor_speed_D, 0, 255);

  if(throttle < 10) {
    motor_speed_A = 0;
    motor_speed_B = 0;
    motor_speed_C = 0;
    motor_speed_D = 0;
  }

  analogWrite(MOTOR_A, motor_speed_A);
  analogWrite(MOTOR_B, motor_speed_B);
  analogWrite(MOTOR_C, motor_speed_C);
  analogWrite(MOTOR_D, motor_speed_D);

  prev_err_roll = err_roll;
  prev_err_pitch = err_pitch;
  prev_err_yaw = err_yaw;
  prev_time = millis();
}

float map_1(float x, float in_min, float in_max, float out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}