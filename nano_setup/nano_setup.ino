#include <Wire.h>
#include <EEPROM.h>

#define imu_address 0x68

int error = 0;

//Transmitter variables
unsigned long timer, timer_1, timer_2, timer_3, timer_4, timer_5, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
volatile int receiver_input[5];

//MPU9050
long acc_x, acc_y, acc_z;

//EEPROM variables
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
double accel_results[6];

typedef union {
  double decimal;
  uint8_t bytes[4];
} converter;

converter number;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 12)to trigger an interrupt on state change.

  setupSensor();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (error == 0) wait_for_receiver();      //Wait for transmitter to be turned on within 10 seconds
  if (error == 0) record_center_position(); //Find out the center positions of transmitter (including throttle)
  if (error == 0) register_min_max();       //Register the min and max values of the receiver channels
  if (error == 0) calibrate_accel();        //Measure the endpoints of each accelerometer axis
  if (error == 0) save_eeprom_data();       //If all is good, store the information in the EEPROM

  if (error == 0) Serial.println("Setup SUCCESS! \n You may close the program now :)");
  else Serial.println("There is an error during setup. Please try again.");

  while (1);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void record_center_position() {
  delay(2000);
  Serial.println(F("Place all sticks in the center position within 10 seconds."));
  for (int i = 9; i > 0; i--)  {
    delay(1000);
    Serial.print(i);
    Serial.print(" ");
  }
  Serial.println(" ");

  //Store the central stick positions
  center_channel_1 = receiver_input[1];
  center_channel_2 = receiver_input[2];
  center_channel_3 = receiver_input[3];
  center_channel_4 = receiver_input[4];

  Serial.println(F(""));
  Serial.println(F("Center positions stored."));
  Serial.print(F("Input D8 = "));
  Serial.println(receiver_input[1]);
  Serial.print(F("Input D9 = "));
  Serial.println(receiver_input[2]);
  Serial.print(F("Input D10 = "));
  Serial.println(receiver_input[3]);
  Serial.print(F("Input D11 = "));
  Serial.println(receiver_input[4]);
  Serial.println(F(""));
  Serial.println(F(""));

  delay(2000);
}

void wait_for_receiver() {
  Serial.println("Turn on your transmitter in the next 10 seconds.");
  byte zero = 0;
  timer = millis() + 10000;
  while (timer > millis() && zero < 15)  {
    if (receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;
    if (receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;
    if (receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;
    if (receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if (zero == 0)  {
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!"));
  }
  else Serial.println(F(" OK"));
}

void register_min_max() {
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("Gently move all the sticks simultaneously to their extends"));
  Serial.println(F("When ready put the sticks back in their center positions"));

  byte zero = 0;
  low_channel_1 = receiver_input[1];
  low_channel_2 = receiver_input[2];
  low_channel_3 = receiver_input[3];
  low_channel_4 = receiver_input[4];

  while (receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20) {
    //Serial.println(receiver_input[1]);
    delay(250); //check if channel 1 centered
  }
  Serial.println(F("Measuring endpoints...."));
  while (zero < 15) {
    if (receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20)  zero |= 0b00000001;
    if (receiver_input[2] < center_channel_2 + 20 && receiver_input[2] > center_channel_2 - 20)  zero |= 0b00000010;
    if (receiver_input[3] < center_channel_3 + 20 && receiver_input[3] > center_channel_3 - 20)  zero |= 0b00000100;
    if (receiver_input[4] < center_channel_4 + 20 && receiver_input[4] > center_channel_4 - 20)  zero |= 0b00001000;

    if (receiver_input[1] < low_channel_1) low_channel_1 = receiver_input[1];
    if (receiver_input[2] < low_channel_2) low_channel_2 = receiver_input[2];
    if (receiver_input[3] < low_channel_3) low_channel_3 = receiver_input[3];
    if (receiver_input[4] < low_channel_4) low_channel_4 = receiver_input[4];

    if (receiver_input[1] > high_channel_1)  high_channel_1 = receiver_input[1];
    if (receiver_input[2] > high_channel_2)  high_channel_2 = receiver_input[2];
    if (receiver_input[3] > high_channel_3)  high_channel_3 = receiver_input[3];
    if (receiver_input[4] > high_channel_4)  high_channel_4 = receiver_input[4];

    delay(100);
  }

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("High, low and center values found during setup"));
  Serial.print(F("Digital input 08 values: "));
  Serial.print(low_channel_1);
  Serial.print(F(" - "));
  Serial.print(center_channel_1);
  Serial.print(F(" - "));
  Serial.println(high_channel_1);
  Serial.print(F("Digital input 09 values: "));
  Serial.print(low_channel_2);
  Serial.print(F(" - "));
  Serial.print(center_channel_2);
  Serial.print(F(" - "));
  Serial.println(high_channel_2);
  Serial.print(F("Digital input 10 values: "));
  Serial.print(low_channel_3);
  Serial.print(F(" - "));
  Serial.print(center_channel_3);
  Serial.print(F(" - "));
  Serial.println(high_channel_3);
  Serial.print(F("Digital input 11 values: "));
  Serial.print(low_channel_4);
  Serial.print(F(" - "));
  Serial.print(center_channel_4);
  Serial.print(F(" - "));
  Serial.println(high_channel_4);
}

void setupSensor() {
  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search.
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.

  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search.
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search.
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(imu_address, 1);                                         //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(13, HIGH);                                                  //Turn on the warning led
    while (1) delay(10);                                                      //Stay in this loop for ever
    Serial.println("Error with sensor!");
  }

  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro
}

void calibrate_accel() {
  String text_prompt[6] = {"Place the drone upright on a flat surface (Z axis LOW)",
                           "Place the drone on its RIGHT side(Y axis LOW)",
                           "Place the drone on its LEFT side (Y axis HIGH)",
                           "Point the nose of the drone DOWNWARDS (X axis LOW)",
                           "Point the nose of the drone UPWARDS (X axis HIGH)",
                           "Flip the drone upside down (Z axis HIGH)"
                          };
  int axis[6] = {3, 2, 2, 1, 1, 3};

  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Measuring accelerometer endpoints"));
  Serial.println(F("==================================================="));
  Serial.print("Starting in: ");
  for (int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println();

  for (int i = 0; i < 6; i++) {
    Serial.println(text_prompt[i]);
    for (int j = 7; j > 0; j--) {
      Serial.print(j);
      Serial.print(" ");
      delay(1000);
    }
    Serial.print(F("Hold it there"));

    long acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
    for (int k = 0; k < 2000; k++) {
      if (k % 200 == 0) Serial.print(".");
      Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
      Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
      Wire.endTransmission();                                                 //End the transmission.
      Wire.requestFrom(imu_address, 6);                                     //Request 14 bytes from the gyro.

      while (Wire.available() < 6);
      acc_x = Wire.read() << 8 | Wire.read();
      acc_y = Wire.read() << 8 | Wire.read();
      acc_z = Wire.read() << 8 | Wire.read();

      acc_x *= -1;
      //acc_y *= -1;
      acc_z *= -1;

      acc_x_sum += acc_x;
      acc_y_sum += acc_y;
      acc_z_sum += acc_z;
    }
    Serial.println();
    if (axis[i] == 1) {
      acc_x_sum /= 2000;
      Serial.println(acc_x_sum);
      axis[i] = acc_x_sum;
    } else if (axis[i] == 2) {
      acc_y_sum /= 2000;
      Serial.println(acc_y_sum);
      axis[i] = acc_y_sum;
    } else if (axis[i] == 3) {
      acc_z_sum /= 2000;
      Serial.println(acc_z_sum);
      axis[i] = acc_z_sum;
    }
    Serial.println();
  }
  int data[6];
  Serial.print("X axis LOW: ");
  Serial.println(axis[3]);
  data[0] = axis[3];
  Serial.print("X axis HIGH: ");
  Serial.println(axis[4]);
  data[1] = axis[4];

  Serial.print("Y axis LOW: ");
  Serial.println(axis[1]);
  data[2] = axis[1];
  Serial.print("Y axis HIGH: ");
  Serial.println(axis[2]);
  data[3] = axis[2];

  Serial.print("Z axis LOW: ");
  Serial.println(axis[0]);
  data[4] = axis[0];
  Serial.print("Z axis HIGH: ");
  Serial.println(axis[5]);
  data[5] = axis[5];

  for (int i = 0; i < 3; i++) {
    double gradient = 8192.0 / (data[i * 2 + 1] - data[i * 2]);
    double intercept = -4096.0 - gradient * data[i * 2];

    accel_results[i * 2] = gradient;
    accel_results[i * 2 + 1] = intercept;
  }
  Serial.println();
  Serial.print("X axis gradient: ");
  Serial.println(accel_results[0], 5);
  Serial.print("X axis intercept: ");
  Serial.println(accel_results[1], 5);
  Serial.print("Y axis gradient: ");
  Serial.println(accel_results[2], 5);
  Serial.print("Y axis intercept: ");
  Serial.println(accel_results[3], 5);
  Serial.print("Z axis gradient: ");
  Serial.println(accel_results[4], 5);
  Serial.print("Z axis intercept: ");
  Serial.println(accel_results[5], 5);
}

void save_eeprom_data() {
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Storing EEPROM information"));
  Serial.println(F("==================================================="));
  Serial.println(F("Writing EEPROM"));
  delay(1000);

  EEPROM.write(0, center_channel_1 & 0b11111111);
  EEPROM.write(1, center_channel_1 >> 8);
  EEPROM.write(2, center_channel_2 & 0b11111111);
  EEPROM.write(3, center_channel_2 >> 8);
  EEPROM.write(4, center_channel_3 & 0b11111111);
  EEPROM.write(5, center_channel_3 >> 8);
  EEPROM.write(6, center_channel_4 & 0b11111111);
  EEPROM.write(7, center_channel_4 >> 8);
  EEPROM.write(8, high_channel_1 & 0b11111111);
  EEPROM.write(9, high_channel_1 >> 8);
  EEPROM.write(10, high_channel_2 & 0b11111111);
  EEPROM.write(11, high_channel_2 >> 8);
  EEPROM.write(12, high_channel_3 & 0b11111111);
  EEPROM.write(13, high_channel_3 >> 8);
  EEPROM.write(14, high_channel_4 & 0b11111111);
  EEPROM.write(15, high_channel_4 >> 8);
  EEPROM.write(16, low_channel_1 & 0b11111111);
  EEPROM.write(17, low_channel_1 >> 8);
  EEPROM.write(18, low_channel_2 & 0b11111111);
  EEPROM.write(19, low_channel_2 >> 8);
  EEPROM.write(20, low_channel_3 & 0b11111111);
  EEPROM.write(21, low_channel_3 >> 8);
  EEPROM.write(22, low_channel_4 & 0b11111111);
  EEPROM.write(23, low_channel_4 >> 8);

  for (int i = 0; i < 6; i++) {
    number.decimal = accel_results[i];
    EEPROM.write(i * 4 + 24, number.bytes[0]);
    EEPROM.write(i * 4 + 25, number.bytes[1]);
    EEPROM.write(i * 4 + 26, number.bytes[2]);
    EEPROM.write(i * 4 + 27, number.bytes[3]);
  } //last register number: 47

  Serial.println(F("Done!"));

  //Write the EEPROM signature
  EEPROM.write(48, 'J');
  EEPROM.write(49, 'M');
  EEPROM.write(50, 'B');

  //To make sure evrything is ok, verify the EEPROM data.
  Serial.println(F("Verify EEPROM data"));
  delay(1000);
  if (center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
  if (center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
  if (center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
  if (center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;

  if (high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
  if (high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
  if (high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
  if (high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;

  if (low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
  if (low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
  if (low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
  if (low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = EEPROM.read(i * 4 + 24);
    number.bytes[1] = EEPROM.read(i * 4 + 25);
    number.bytes[2] = EEPROM.read(i * 4 + 26);
    number.bytes[3] = EEPROM.read(i * 4 + 27);
    if (accel_results[i] != number.decimal) error = 1;
  }

  if ('J' != EEPROM.read(48))error = 1;
  if ('M' != EEPROM.read(49))error = 1;
  if ('B' != EEPROM.read(50))error = 1;

  if (error == 1)Serial.println(F("EEPROM verification failed!!!"));
  else Serial.println(F("Verification done"));
}

ISR(PCINT0_vect) {
  current_time = micros();
  //* ========================================= Channel 1 =========================================
  if (PINB & B00000001) {                                                   //Is input 8 high?
    if (last_channel_1 == 0) {                                              //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //* ========================================= Channel 2 =========================================
  if (PINB & B00000010) {                                                  //Is input 9 high?
    if (last_channel_2 == 0) {                                              //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //* ========================================= Channel 3 =========================================
  if (PINB & B00000100) {                                                  //Is input 10 high?
    if (last_channel_3 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
  }
  //* ========================================= Channel 4 =========================================
  if (PINB & B00001000) {                                                  //Is input 11 high?
    if (last_channel_4 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //* ========================================= Channel 5 =========================================
  if (PINB & B00010000) {                                                  //Is input 11 high?
    if (last_channel_5 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_5 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 4 is current_time - timer_4.
  }
}
