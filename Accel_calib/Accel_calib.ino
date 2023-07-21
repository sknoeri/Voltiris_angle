#include <Wire.h>
/*#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Wireless transmission variables
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
struct Data_Package { // Max of 32bytes can be transmitted
  float Roll = 0.0f;
  float Pitch = 0.0f;
  float Yaw = 0.0f;
};
Data_Package data; // Create a variable with the above structure
*/
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
// Accelerometer

float AccX, AccY, AccZ, AccX_cal, AccY_cal, AccZ_cal;
uint32_t LoopTimer;
//Offset calibration Accelerometer
float AccXcalib;
float AccYcalib;
float AccZcalib;

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
void accel_sygnals(void){
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

  AccX = (float)AccXLSB/16384; // Acceleration values in g
  AccY = (float)AccYLSB/16384; // ACCEL_OUT = Acc_sensitivity[LSB/g] * Acc[g]
  AccZ = (float)AccZLSB/16384;
  volatile static uint16_t i=0;
  volatile static float accx=0;
  volatile static float accy=0;
  volatile static float accz=0;
  if(i<200){
    accx+=AccX;
    accy+=AccY;
    accz+=AccZ;
    i++;
  }else{
    i=0;
    AccXcalib=accx/200;
    AccYcalib=accy/200;
    AccZcalib=accz/200;
    accx=0;
    accy=0;
    accz=0;
  }
  AccX_cal = (float)AccXLSB/16384-AccXcalib; // Acceleration values in g
  AccY_cal = (float)AccYLSB/16384-AccYcalib; // ACCEL_OUT = Acc_sensitivity[LSB/g] * Acc[g]
  AccZ_cal = (float)AccZLSB/16384-AccZcalib;

}

void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Wire.begin();
    delay(500);
    //Accelo gyro calibrations
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);//Power mamagement: set to 0x00 to activate the MPU9250  Register 107 on datasheet
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);//Enable gyroscope and magnetometer readings
    
    /*radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();*/
    LoopTimer=micros();
}
uint8_t b=0;
void loop() {
  accel_sygnals();
  //radio.write(&data, sizeof(Data_Package));
  if(b<200){ 
    b++;
  }else{
    Serial.print("AccX_cal,");
    Serial.print(AccX_cal,4);
    Serial.print(" AccY_cal,");
    Serial.print(AccY_cal,4);
    Serial.print(" AccZ_cal,");
    Serial.print(AccZ_cal,4);
    Serial.print(" AccX, ");
    Serial.print(AccX);
    Serial.print(" AccY,");
    Serial.print(AccY);
    Serial.print(" AccZ ");
    Serial.print(AccZ);
    Serial.print(" AccXcalib, ");
    Serial.print(AccXcalib,4);
    Serial.print(" AccYcalib,");
    Serial.print(AccYcalib,4);
    Serial.print(" AccZcalib, ");
    Serial.println(AccZcalib,4);
    b=0;
   }
  
  while(micros()-LoopTimer<5000);
  LoopTimer=micros();
}
