#include <Wire.h>

int gyro_address = 104;
float acc_x, acc_y, acc_z;
float acc_x_min, acc_x_max, acc_y_min, acc_y_max, acc_z_min, acc_z_max;

void setup(){
    Wire.begin();
    Serial.begin(115200);

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
    if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
        digitalWrite(12, HIGH);                                                  //Turn on the warning led
        while (1)delay(10);                                                      //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    int difference = 0;
    int loop_timer = micros();
    while(difference < 20000){
        Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
        Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                 //End the transmission.
        Wire.requestFrom(gyro_address, 6);                                      //Request 14 bytes from the gyro.

        while (Wire.available() < 6);                                          //Wait until the 6 bytes are received.
        acc_x = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
        acc_y = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
        acc_z = Wire.read() << 8 | Wire.read();

        if (acc_x < acc_x_min) acc_x_min = acc_x;
        if (acc_x > acc_x_max) acc_x_max = acc_x;
        if (acc_y < acc_y_min) acc_y_min = acc_y;
        if (acc_y > acc_y_max) acc_y_max = acc_y;
        if (acc_z < acc_z_min) acc_z_min = acc_z;
        if (acc_z < acc_z_max) acc_z_max = acc_z;

        Serial.println("X: " + (String) acc_x_min + ", " + (String) acc_x_max);
        Serial.println("Y: " + (String) acc_y_min + ", " + (String) acc_y_max);
        Serial.println("Z: " + (String) acc_z_min + ", " + (String) acc_z_max);

        difference = micros() - loop_timer;
        Serial.println("Time left: " + (String) difference);
        Serial.println();
    }

    Serial.println("Setup done!");
}

void loop(){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address, 6);                                      //Request 14 bytes from the gyro.

    while (Wire.available() < 6);                                          //Wait until the 6 bytes are received.
    acc_x = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    acc_y = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    acc_z = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

    acc_x = map(acc_x, acc_x_min, acc_x_max, -1000, 1000);
    acc_y = map(acc_y, acc_y_min, acc_y_max, -1000, 1000);
    acc_z = map(acc_z, acc_z_min, acc_z_max, -1000, 1000);

    Serial.println("X: " + (String) acc_x);
    Serial.println("Y: " + (String) acc_y);
    Serial.println("Z: " + (String) acc_z);
}