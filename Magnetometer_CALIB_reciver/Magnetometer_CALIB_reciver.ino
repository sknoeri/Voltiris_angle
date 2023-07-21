#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
struct Data_Package {
  float MagX = 0.0f;
  float MagY = 0.0f;
  float MagZ = 0.0f;
};
Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(115200);
  delay(500);
  radio.begin(); 
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
int a = 0;
void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
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
    //Serial.print(',');
    Serial.print((int)data.MagX*10);
    Serial.print(',');
    Serial.print((int)data.MagY*10);
    Serial.print(',');
    Serial.print((int)data.MagZ*10);
    Serial.println();
  }
  
}
