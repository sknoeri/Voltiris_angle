#include <Wire.h>
//#include "MPU9250.h"

#define MPU9250_ADDRESS  0x68
#define USER_CTRL        0x6A // Contol register of the MPU 9250 used for setting into ItC master mode etc
#define I2C_MST_CTRL     0x24 // I2C configuration for multi-master mode set ItC frequenzie etc
#define I2C_SLV0_ADDR    0x25 // Address of slave
#define I2C_SLV0_REG     0x26 // Slave register to read or write
#define I2C_SLV0_CTRL    0x27 // enable reading of ext sense data and number of bytes to read
#define EXT_SENS_DATA_00 0x49 // data to read from slave 0
#define I2C_SLV0_DO      0x63 // data out when slave 0 set to write
// Magnetometer stuff
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // ID of AK8963 should return 0x48
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_CNTL      0x0A // Used to set the reading mode or to power down
#define AK8963_CNTL2     0x0B // Used to Reset manetometer
#define AK8963_ASAX      0x10 // Sensitivity values

float MagX, MagY, MagZ;
float Sense_adjX, Sense_adjY,Sense_adjZ;

//MPU9250 mpu;



uint8_t readByte(uint8_t address, uint8_t subaddress) {
  uint8_t data = 0;
  Wire.beginTransmission(address);
  Wire.write(subaddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1);
  data = Wire.read();
  return data;
}
void writeByte(uint8_t address, uint8_t subaddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subaddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t getAK8963CID() {
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20);    // Enable I2C Master mode
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz

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
  Serial.println(Sense_adjX);
  Serial.println(Sense_adjY);
  Serial.println(Sense_adjZ);

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
    MagX = (float)Xmag * (4912.0f / 32760.0f)*Sense_adjX;
    MagY = (float)Ymag * (4912.0f / 32760.0f)*Sense_adjY;
    MagZ = (float)Zmag * (4912.0f / 32760.0f)*Sense_adjZ;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  Wire.setClock(400000);
  delay(500);
  //Serial.print(" The ID of the Magnetometeri should be 0x48 and it is: 0x");
  //Serial.print(getAK8963CID(),HEX);
  getAK8963CID();
  //Serial.println("");
  initAK8963Slave(1, 0x06);
}
int a = 0;
void loop() {
  readMagData();
  //Serial.print("X, ");
  Serial.print("Raw:");
  Serial.print(a);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print((int)MagX*10);
  Serial.print(",");
  Serial.print((int)MagY*10);
  Serial.print(",");
  Serial.print((int)MagZ*10);
  Serial.println();
  delay(2);
  // put your main code here, to run repeatedly:

}
