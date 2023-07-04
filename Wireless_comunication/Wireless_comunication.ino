#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
struct Data_Package {
  float Roll = 0.0f;
  float Pitch = 0.0f;
  float Yaw = 0.0f;
};
Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(9600);
  radio.begin(); 
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    Serial.print("Roll: ");
    Serial.print(data.Roll);
    Serial.print(" Pitch: ");
    Serial.print(data.Pitch);
    Serial.print(" Yaw: ");
    Serial.println(data.Yaw);
  }
  
}
