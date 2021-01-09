#include<Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

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

Matrix<4> x;    //state vector 
Matrix<4, 4> A; //state transition mat
Matrix<4, 2> B; //control mat
Matrix<4, 4> P; //state uncertainty cov  
Matrix<4, 4> Q; //model noise
Matrix<2, 2> R; //sensor noise
Matrix<2, 4> H; //observation transform

// Aquire accel data
void readAccel(float &AX, float &AY, float &AZ) {
  Wire.beginTransmission(IMU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  AX = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
  AY = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
  AZ = ((float)(Wire.read() << 8 | Wire.read()) / ACCEL_FSR);
}

// Aquire gyro data
void readGyro(float &GX, float &GY, float &GZ) {
  Wire.beginTransmission(IMU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  GX = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR);
  GY = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR);
  GZ = (((float)(Wire.read() << 8 | Wire.read())) / GYRO_FSR); 
}
// Undergo a static callibration period of 100 readings 
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

//Construct a diagonal matrix
template <int q>
inline void setDiag(Matrix<q, q> &a, float val) {
  for(int i = 0; i < q; i++) a(i, i) = val;
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

  //callibration
  callibrate();

  x << 0, GX_Cor, 0, GZ_Cor;
  setDiag<4>(P, 1);
  setDiag<4>(Q, 1);
  setDiag<2>(R, 1);
  H << 1, 0, 0, 0,
       0, 0, 1, 0;
  
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

  //inertial transform and form control matrix from gyro
  float curPitch = x(0, 0);
  float curRoll = x(2, 0);
  Matrix<2, 1> u;
  u << GX, GZ;

  //Prediction step in state space
  x = A*x + B*u;
  P = A*P*(~A) + Q;

  //sensor reading
  Matrix<2, 1> z;
  z << atan(AY/sqrt(AX*AX+AZ*AZ)) * 180/PI, atan(-AX/sqrt(AY*AY+AZ*AZ)) * 180/PI;
  //innovation
  Matrix<2, 1> y = z - H*x;
  //sensor space transform and sensor noise                                            
  Matrix<2, 2> S = H*P*(~H) + R;
  //kalman gain
  Matrix<4, 2> K = P*(~H)*Invert(S);

  //Update step
  x = x + K*y;
  P = P - K*H*P;
  
  
  // Logging
  //Serial.print(GX);
  //Serial.print(",");
  //Serial.println(GY);
  
  Serial.print(x(0, 0));
  Serial.print(" ");
  Serial.print(x(2, 0));
  Serial.print(" ");
  Serial.print("****");
  Serial.print(z(0, 0));
  Serial.print(" ");
  Serial.print(z(1, 0));
  Serial.println("");  

}
