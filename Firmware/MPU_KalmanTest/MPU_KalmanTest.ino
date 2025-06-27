#include <Wire.h>

#define MOTOR_A D0 
#define MOTOR_B D7
#define MOTOR_C D2
#define MOTOR_D D8

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
}

void loop() {
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
  
  Serial.print("Roll Angle [°] "); Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] "); Serial.print(KalmanAnglePitch);
  Serial.print(" Yaw Angle [°] "); Serial.println(gf_yaw);
  
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
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
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.write(data); // Put data in Tx buffer
  Wire.endTransmission(); // Send the Tx buffer
  //  Serial.println("mnnj");

}

//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
  data = Wire.read(); // Fill Rx buffer with result
  return data; // Return data read from slave register
}
