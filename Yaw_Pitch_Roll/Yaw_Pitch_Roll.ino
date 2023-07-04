#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Wireless transmission variables
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
struct Data_Package {
  float Roll = 0.0f;
  float Pitch = 0.0f;
  float Yaw = 0.0f;
};
Data_Package data; // Create a variable with the above structure

// MPU 9250 stuff
#define MPU9250_ADDRESS  0x68
#define CONFIG           0x1A // configuration register; choose bandwidth of filter nad samplerate  for Gyro and TEmp sesor
#define GYRO_CONFIG      0x1B // sets the gyro sensitifty in °/s and LSB
#define ACCEL_CONFIG     0x1C //Sets accelerometer range in g and LSB sensitifty
#define ACCEL_CONFIG_2   0x1D //Sets accelerometer filter bandwidth and samplerate
#define I2C_MST_CTRL     0x24 // I2C configuration for multi-master mode set ItC frequenzie etc
#define I2C_SLV0_ADDR    0x25 // Address of slave
#define I2C_SLV0_REG     0x26 // Slave register to read or write
#define I2C_SLV0_CTRL    0x27 // enable reading of ext sense data and number of bytes to read
#define ACCEL_XOUT_H     0x3B // Accelerometer data Higbyte of X dirct
#define GYRO_XOUT_H      0x43 // Gyrod data Higbyte of X dirct
#define EXT_SENS_DATA_00 0x49 // data to read from slave 0
#define I2C_SLV0_DO      0x63 // data out when slave 0 set to write
#define USER_CTRL        0x6A // Contol register of the MPU 9250 used for setting into ItC master mode etc
#define PWR_MGMT_1       0x6B // Select the clocksurce for the MPU 9250, restes MPU and put in standby
#define PWR_MGMT_2       0x6C // Enables and Disables the Gyro and Accel axis
// Magnetometer stuff
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // ID of AK8963 should return 0x48
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // Mganetometer data
#define AK8963_XOUT_H    0x04
#define AK8963_CNTL      0x0A // Used to set the reading mode or to power down
#define AK8963_CNTL2     0x0B // Used to Reset manetometer
#define AK8963_ASAX      0x10 // Sensitivity values


#define RadToDeg         180/3.14159265359
#define DegToRad         3.14159265359/180
/////////////////////////////////////////////////////////////
// Accelerometer and gyroscope stuff
// Stuff neded to calibrate the gyroscope
float RateRoll, RatePitch, RateYaw;
float RateCalibRoll, RateCalibPitch, RateCalibYaw;
int RateCalibNum;
// stuff neded to calibrate the accelerometer
float AccX, AccY, AccZ;
float AngleRoll,AnglePitch,AngleYaw;
uint32_t LoopTimer;
//Kalman filter constants
float KalAngleRoll=0,KalUncertAngleRoll=4;
float KalAnglePitch=0,KalUncertAnglePitch=4;
float KalAngleYaw=0,KalUncertAngleYaw=4;
float Kal1DOut[]={0,0};

/////////////////////////////////////////////////////
//Magnetometer sutff
const float hard_iron[3] ={42.68, -5.63, -0.84}; //Hard iron calibration from soft ware
const float soft_iron[3][3]={        //Soft iron calibrations from soft ware
  {0.999,0.008,-0.014},
  {0.008,0.974,0.073},
  {-0.014,0.073,1.033}
};
float Mag_x_hor, Mag_y_hor;
float Sense_adjX, Sense_adjY,Sense_adjZ;
float mag_data[3]={0,0,0}; // Magentic readigns of magetometer
float hi_calibMag[3]={0,0,0};

/////////////////////////////////////////////////////////
//Functions
void writeByte(uint8_t address, uint8_t subaddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subaddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
uint8_t readByte(uint8_t address, uint8_t subaddress) {
  uint8_t data = 0;
  Wire.beginTransmission(address);
  Wire.write(subaddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  data = Wire.read();
  return data;
}

////////////////////////////////////////////////////////
///Gyroscope functions 
void gyro_sygnals(void){
  //Gyroscope calibration
  writeByte(MPU9250_ADDRESS, CONFIG, 0x05); //Sets teh lowpass filter with badwidth of 10Hz Fs=1kHz         
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);// Sets the Full sacel range of the gyro to 250°/s and 131LSB sensifitty
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);//Sets accelerometer range ot -+2g and 16384 LSB 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG_2, 0x05);//Sets teh lowpass filter with badwidth of 10Hz Fs=1kHz

  Wire.beginTransmission(MPU9250_ADDRESS); // Reads the Acclerometer values
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false); // need such that the adress pointer is not reset back to 0x00
  Wire.requestFrom(MPU9250_ADDRESS, 6); // read all 6x bite
  int16_t AccXLSB = Wire.read()<<8|Wire.read();
  int16_t AccYLSB = Wire.read()<<8|Wire.read();
  int16_t AccZLSB = Wire.read()<<8|Wire.read();
  
  Wire.beginTransmission(MPU9250_ADDRESS);// Reads the gyroscope values
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false); // need such that the adress pointer is not reset back to 0x00
  Wire.requestFrom(MPU9250_ADDRESS, 6); // read all 6x bite
  int16_t GyroX=Wire.read()<<8|Wire.read(); // first the high byte then the low byte Register 66 to 72 on datasheet
  int16_t GyroY=Wire.read()<<8|Wire.read();
  int16_t GyroZ=Wire.read()<<8|Wire.read();
  
  RateRoll=(float)GyroX/131; //GYRO_XOUT = Gyro_Sensitivity[LSB/(°/s)] * X_angular_rate[(°/s)] Gyro_Sensitivity=131LSB/(°/s) 
  RatePitch=(float)GyroY/131;// ATENTION chekc LSB number carefully because its confusing on the datasheet hure mude.
  RateYaw=(float)GyroZ/131;

  AccX = (float)AccXLSB/16384+0.03935; // Acceleration values in g
  AccY = (float)AccYLSB/16384-0.01443; // ACCEL_OUT = Acc_sensitivity[LSB/g] * Acc[g]
  AccZ = (float)AccZLSB/16384+0.03771;
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*RadToDeg;
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*RadToDeg;
}
void gyro_calib(int a){
  for(RateCalibNum=0; RateCalibNum<a; RateCalibNum++){
    gyro_sygnals();
    RateCalibRoll+=RateRoll;
    RateCalibPitch+=RatePitch;
    RateCalibYaw+=RateYaw;
    delay(1);
  }
  RateCalibRoll/=a;
  RateCalibPitch/=a;
  RateCalibYaw/=a;
}
////////////////////////////////////////////////////////////////////////
//Magenetometer 
uint8_t getAK8963CID() {
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20);    // Enable I2C Master mode
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz
  // Send stuff to AK8963
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, WHO_AM_I_AK8963);           // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(10);
  uint8_t c = readByte(MPU9250_ADDRESS, EXT_SENS_DATA_00);             // Read the WHO_AM_I byte
  return c;
}

void initAK8963Slave(uint8_t Mscale, uint8_t Mmode) { // Mscale deifnes if 16 bit or 14 bit are read // Mmode define
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL2);              // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, 0x01);                       // Reset AK8963
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
  delay(50);
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, 0x00);                       // Power down magnetometer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
  delay(50);

  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, 0x0F);                       // Enter fuze mode
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
  delay(50);
  uint8_t rawData[3]; 
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
  delay(50);
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(EXT_SENS_DATA_00);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 3);            // Read the x-, y-, and z-axis calibration values
  char i = 0;
  while (i < 3) {
    while (!Wire.available()) {
      // waiting
    }
    rawData[i] = Wire.read();
    i++;
  }
  Sense_adjX =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
  Sense_adjY =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  Sense_adjZ =  (float)(rawData[2] - 128)/256.0f + 1.0f;
  //Serial.println(Sense_adjX);
  //Serial.println(Sense_adjY);
  //Serial.println(Sense_adjZ);
  
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, 0x00);                       // Power down magnetometer  
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(50);
  
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(MPU9250_ADDRESS, I2C_SLV0_DO, Mscale << 4 | Mmode);        // Set magnetometer data resolution and sample ODR
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(50);
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(50);
  uint8_t c = readByte(MPU9250_ADDRESS, EXT_SENS_DATA_00);
  //Serial.print("The magnetometer (AK8963_CNTL) is set ito the mode 0x");
  //Serial.print(c, HEX);
  //Serial.println("");
}
bool checkNewMagData() {
  bool test;
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_ST1);                // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  test = (readByte(MPU9250_ADDRESS, EXT_SENS_DATA_00) & 0x01);         // Check data ready status byte
  return test;
}

void readMagData() {
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
  //delay(2);
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(EXT_SENS_DATA_00);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 7);            // Read the x-, y-, and z-axis calibration values
  char i = 0;
  while (i < 7) {
    while (!Wire.available()) {
      // waiting
    }
    rawData[i] = Wire.read();
    i++;
  }
  uint8_t c = rawData[6]; // End data read by reading ST2 register
  if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    int16_t Xmag = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    int16_t Ymag = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    int16_t Zmag = ((int16_t)rawData[5] << 8) | rawData[4] ;
    mag_data[0] = (float)Xmag * (4912.0f / 32760.0f)*Sense_adjX;
    mag_data[1] = (float)Ymag * (4912.0f / 32760.0f)*Sense_adjY;
    mag_data[2] = (float)Zmag * (4912.0f / 32760.0f)*Sense_adjZ;
    // Apply hard iron offset
    for (uint8_t i=0;i<3;i++){hi_calibMag[i]= mag_data[i]-hard_iron[i];}
    // Apply soft iron offset
    for (uint8_t i=0;i<3;i++){
      mag_data[i] = (soft_iron[i][0]*hi_calibMag[0])+
                    (soft_iron[i][1]*hi_calibMag[1])+
                    (soft_iron[i][3]*hi_calibMag[2]);
    }
  }
}

////////////////////////////////////////////////////////////////////
//Kalmann filter suff
float kalman_1D(float KalState, float KalUncert, float KalInput, float KalMes, float dt){
  KalState=KalState+dt*KalInput; //KalState = angle calulated with Kal filter;
  KalUncert=KalUncert+dt*dt*16;  //KalMes = anlge measurend with accelerometer. 16 is the covarianz matrix of the roational speed
  float KalGain=KalUncert/(KalUncert+9);//KalmanInput = Rotation rate. 9 is an estimation of the covarianz matrix of the measurment noise form the accelermoemter angle
  KalState=KalState+KalGain*(KalMes-KalState); //dt= The delta time
  KalUncert=(1-KalGain)*KalUncert;
  Kal1DOut[0]=KalState;
  Kal1DOut[1]=KalUncert;
}

void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Wire.begin();
    delay(500);
    //Accelo gyro calibrations
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);//Power mamagement: set to 0x00 to activate the MPU9250  Register 107 on datasheet
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);//Enable gyroscope and magnetometer readings
    gyro_calib(2000);
    //Magenometer inctialisactions
    getAK8963CID();
    initAK8963Slave(1, 0x06);
    // Wireless transmission send intialisation
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    LoopTimer=micros();
}

void loop() {
  gyro_sygnals();
  RateRoll-=RateCalibRoll;
  RatePitch-=RateCalibPitch;
  RateYaw-=RateCalibYaw;
  kalman_1D(KalAngleRoll,KalUncertAngleRoll,RateRoll,AngleRoll,0.1);
  KalAngleRoll=Kal1DOut[0];
  data.Roll = KalAngleRoll;
  KalUncertAngleRoll=Kal1DOut[1];
  kalman_1D(KalAnglePitch,KalUncertAnglePitch,RatePitch,AnglePitch,0.1);
  KalAnglePitch=Kal1DOut[0];
  data.Pitch = KalAnglePitch;
  KalUncertAnglePitch=Kal1DOut[1];
  //Magnetometer stuff
  float Mag_pitch = -KalAngleRoll*DegToRad;
  float Mag_roll = KalAnglePitch*DegToRad;
  readMagData();
  Mag_x_hor = mag_data[0] * cos(Mag_pitch) + mag_data[1] * sin(Mag_roll) * sin(Mag_pitch) 
              - mag_data[2] * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = mag_data[1] * cos(Mag_roll) + mag_data[2] * sin(Mag_roll);
  AngleYaw = -1*(atan2(Mag_x_hor,Mag_y_hor))*RadToDeg;
  if (AngleYaw<0){
    AngleYaw+= 360;
  }
  kalman_1D(KalAngleYaw,KalUncertAngleYaw,RateYaw,AngleYaw,0.1);
  KalAngleYaw=Kal1DOut[0];
  data.Yaw=KalAngleYaw;
  KalUncertAngleYaw=Kal1DOut[1];
  
  radio.write(&data, sizeof(Data_Package));
  
  //Serial.print("Kalmann_Roll,");
  //Serial.print(KalAngleRoll);
  //Serial.print("Kalmann_Yaw,");
  //Serial.print(KalAngleYaw);
  //Serial.print(" Tillt,");
  //Serial.print(AngleYaw);
  //Serial.print("Accel_Roll ");
  //Serial.print(AngleRoll);
  //Serial.print(" Kalmann_pitch,");
  //Serial.println(KalAnglePitch);
  //Serial.print(" Accel_Acc ");
  //Serial.println(AnglePitch);
  while(micros()-LoopTimer<100000);
  LoopTimer=micros();
  
}
