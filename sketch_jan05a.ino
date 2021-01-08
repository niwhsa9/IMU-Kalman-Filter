#include<Wire.h>

const char IMU = 0x68;
const float ACCEL_FSR = 8192;
const float GYRO_FSR = 131;
float GX_Cor, GY_Cor, GZ_Cor;
float prevTime = 0;

typedef struct Euler {
  float pitch;
  float roll;
  float yaw;
} Euler_t;

Euler_t pose;


void readAccel(float &AX, float &AY, float &AZ) {
  // Aquire accel data
  Wire.beginTransmission(IMU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  AX = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
  AY = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
  AZ = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
}

void readGyro(float &GX, float &GY, float &GZ) {
  // Aquire gyro data
  Wire.beginTransmission(IMU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  GX = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR);
  GY = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR);
  GZ = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR); 
}

void callibrate() {
  GX_Cor = 0;
  GY_Cor = 0;
  GZ_Cor = 0;
  
  for(int i = 0; i < 100; i++) {
    float GX,GY,GZ;
    readGyro(GX, GY, GZ);
    GX_Cor += GX;
    GY_Cor += GY;
    GZ_Cor += GZ;
  }
  GX_Cor /= 100;
  GY_Cor /= 100;
  GZ_Cor /= 100;
}

void setup() {
  pose.pitch = 0;
  pose.roll = 0;
  pose.yaw = 0;
  
  // one time I2C setup
  Serial.begin(9600);
  //Wire.setClock(400000UL);
  Wire.begin();

  Wire.beginTransmission(IMU);
  Wire.write(0x6B);                       //Indiciate power register
  Wire.write(0x00);                       //Use internal clock
  Wire.endTransmission(true);

  // set accel FSR to 4g
  Wire.beginTransmission(IMU);
  Wire.write(0x1C); 
  Wire.write(0x08);          
  Wire.endTransmission();

  // set gyro FSR to 250 dps
  Wire.beginTransmission(IMU);
  Wire.write(0x1B); 
  Wire.write(0x00);          
  Wire.endTransmission();

  callibrate();
  prevTime = millis()/1000.0;
}

void loop() {
  // dt timer 
  float curTime = millis()/1000.0;
  float dt = curTime - prevTime;
  prevTime = curTime;

  // read raw values 
  float AX,AY,AZ,GX,GY,GZ;
  readAccel(AX, AY, AZ);
  readGyro(GX, GY, GZ);

  // naieve pose estimation 
  pose.pitch += (GX - GX_Cor)* dt;
  pose.roll += (GZ - GZ_Cor)* dt;
  pose.yaw += (GY - GY_Cor) * dt;

  // Logging
  Serial.print(AX);
  Serial.print(" ");
  Serial.print(AY);
  Serial.print(" ");
  Serial.print(AZ);
  Serial.print("-----");
  Serial.print(pose.pitch);
  Serial.print(" ");
  Serial.print(pose.roll);
  Serial.print(" ");
  Serial.print(pose.yaw);
  Serial.println("");  

}
