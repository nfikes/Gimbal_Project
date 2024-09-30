// Program used to run the ESP32 DevkitC Wrover E (Used in Arduino IDE with Libraries installed for the includes below)

#include <ESP32Servo.h>
#include <math.h>    //round
#include <string.h>  //strcmp
#include <stdlib.h>  //abs

Servo servo_s1;
Servo servo_s2;
Servo servo_s3;

int s1_Motor_Pin = 15;
int s2_Motor_Pin = 2;
int s3_Motor_Pin = 5;

int s1_origin = 100.0;
int s2_origin = 90.0;
int s3_origin = 90.0;

int RX_PIN = 18; //16 pin on printed board
int TX_PIN = 19; //17 pin on printed board

void setupSerial() {
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("Serial Set up.\n");
}

void setup() {
  // put your setup code here, to run once:
  setupSerial();
  servo_s1.attach(s1_Motor_Pin, 500, 2500);
  servo_s2.attach(s2_Motor_Pin, 500, 2500);
  servo_s3.attach(s3_Motor_Pin, 500, 2500);
  Serial.println("Motors successfully attached.\n");

  servo_s1.write(s1_origin);
  servo_s2.write(s2_origin);
  servo_s3.write(s3_origin);
  Serial.println("Motors set to origins.\n");
}

static std::string receivedData;

void loop() {
  // put your main code here, to run repeatedly:'
  if (Serial1.available()) {
    
    int p = Serial1.read();

    if (p == -1) {

      //Handle Later

    } else if (p != '\n') { 

      receivedData += (char)p;

    } else {

      int length = receivedData.length();
      char motor = receivedData[0];
      std::string value = receivedData.substr(1, length - 1);
      int value_parsed = std::stoi(value);
      Serial.print(receivedData.c_str());
      Serial.println('\n');
      receivedData.clear();

      noInterrupts();
      switch (motor) {

      case 'A':
        {
          servo_s1.write(s1_origin + value_parsed);
          Serial.println("Value written to s1: ");
          Serial.print(s1_origin + value_parsed);
          Serial.println("\n");
          break;
        }

      case 'B':
        {
          servo_s2.write(s2_origin + value_parsed);
          Serial.println("Value written to s2: ");
          Serial.print(s2_origin + value_parsed);
          Serial.println("\n");
          break;
        }

      case 'C':
        {
        servo_s3.write(s3_origin + value_parsed);
        Serial.println("Value written to s3: ");
        Serial.print(s3_origin + value_parsed);
        Serial.println("\n");
        break;
        }
      }
      interrupts();
    }
  } else {
    //Serial.println("No Data Avaliable\n");
  }
}
