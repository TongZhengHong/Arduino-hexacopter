#include <Wire.h>
#include <EEPROM.h>

#define imu_address 0x68
#define mag_address 0x76

byte eeprom_data[50];
unsigned long difference, main_loop_timer;

double gyro_cal[4], accel_cal[6];
double acc_cal_roll, acc_cal_pitch;

int mag_x_raw, mag_y_raw, mag_z_raw;
int acc_x_raw, acc_y_raw, acc_z_raw;
int gyro_x_raw, gyro_y_raw, gyro_z_raw;
int temperature;

int acc_x_mem[16], acc_y_mem[16], acc_z_mem[16];
int gyro_x_mem[8], gyro_y_mem[8], gyro_z_mem[8];
long acc_x_sum, acc_y_sum, acc_z_sum, gyro_x_sum, gyro_y_sum, gyro_z_sum;
byte gyro_loop_counter = 0, acc_loop_counter = 0;

long acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

int roll, pitch;
float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;

typedef union {
  double decimal;
  uint8_t bytes[4];
} converter;

converter number;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (int start = 0; start <= 50; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[48] != 'J' || eeprom_data[49] != 'M' || eeprom_data[50] != 'B')
    delay(10);

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 24];
    number.bytes[1] = eeprom_data[i * 4 + 25];
    number.bytes[2] = eeprom_data[i * 4 + 26];
    number.bytes[3] = eeprom_data[i * 4 + 27];
    accel_cal[i] = number.decimal;
  }

  //setup_sensor();
  //calibrate_sensors();

  Serial.println("Setup DONE");
  delay(3000);
}

void loop() {
  calculate_pitch_roll();

  calculate_heading();

  maintain_loop_time();
}

void setup_sensor() {
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

  Wire.beginTransmission(mag_address);
  Wire.write(0x0A);                                                          //Write to Control register 1
  Wire.write(0x12);                                                          //Write 00010010 --> 16 bit output, Continuous measurement mode 1
  Wire.endTransmission();
}

void calibrate_sensors() {
  Serial.print("Calibrating sensor");
  for (int i = 0; i < 2000; i++) {
    if (i % 200 == 0) Serial.print(".");
    Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
    Wire.write(0x43);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(imu_address, 6);                                      //Request 14 bytes from the gyro.

    while (Wire.available() < 6);                                          //Wait until the 6 bytes are received.
    gyro_cal[1] += Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[2] += Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[3] += Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  }

  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;
  gyro_cal[3] /= 2000;

  Serial.println();
  Serial.println("Gyroscope calibration done!");
  for (int i = 1; i < 4; i++) Serial.println(gyro_cal[i]);

  Serial.print("Calibrating accelerometer offsets");
  for (int i = 0; i < 2000; i++) {
    if (i % 200 == 0) Serial.print(".");
    calibrate_accel();
    acc_cal_pitch += angle_pitch;
    acc_cal_roll += angle_roll;

    maintain_loop_time();
  }
  acc_cal_pitch /= 2000;
  acc_cal_roll /= 2000;

  Serial.println();
  Serial.println("Accelerometer offset calibration done!");
  Serial.println("Pitch: " + (String) acc_cal_pitch);
  Serial.println("Roll: " + (String) acc_cal_roll);
}

void calibrate_accel() {
  Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(imu_address, 14);                                     //Request 14 bytes from the gyro.

  while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
  acc_x_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  acc_y_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  acc_z_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
  gyro_x_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_y_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_z_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

  gyro_x_raw *= -1;
  //gyro_y_raw *= -1;
  gyro_z_raw *= -1;

  acc_x_raw *= -1;
  //acc_y_raw *= -1;
  acc_z_raw *= -1;

  acc_x_raw = accel_cal[0] * acc_x_raw + accel_cal[1];
  acc_y_raw = accel_cal[2] * acc_y_raw + accel_cal[3];
  acc_z_raw = accel_cal[4] * acc_z_raw + accel_cal[5];

  //Gyro calculations 0.000076336 = (0.005 / 65.5)
  angle_roll += gyro_x_raw * 0.000076336;
  angle_pitch += gyro_y_raw * 0.000076336;

  angle_roll_acc = (float) (atan2(acc_y_raw, acc_z_raw)) * 57.296;
  angle_pitch_acc = (float) (atan2(acc_z_raw, acc_x_raw)) * 57.296;

  angle_pitch_acc += (float) 90.0;
  if (angle_roll_acc > 90) angle_roll_acc -= (float) 180;
  else angle_roll_acc += (float) 180;

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
}

void calculate_pitch_roll() {
  Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(imu_address, 14);                                     //Request 14 bytes from the gyro.

  while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
  acc_x_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  acc_y_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  acc_z_raw = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
  gyro_x_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_y_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_z_raw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

  gyro_x_raw *= -1;
  //gyro_y_raw *= -1;
  gyro_z_raw *= -1;

  acc_x_raw *= -1;
  //acc_y_raw *= -1;
  acc_z_raw *= -1;

  calculate_moving_average();

  acc_x = accel_cal[0] * acc_x + accel_cal[1];
  acc_y = accel_cal[2] * acc_y + accel_cal[3];
  acc_z = accel_cal[4] * acc_z + accel_cal[5];

  //Gyro calculations 0.000076336 = (0.005 / 65.5)
  angle_roll += gyro_x * 0.000076336;
  angle_pitch += gyro_y * 0.000076336;

  //Accelerometer angle calculations
  angle_roll_acc = (float) (atan2(acc_y, acc_z)) * 57.296;
  angle_pitch_acc = (float) (atan2(acc_z, acc_x)) * 57.296;

  angle_pitch_acc += (float) 90.0;
  if (angle_roll_acc > 90) angle_roll_acc -= (float) 180;
  else angle_roll_acc += (float) 180;

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_roll_acc -= acc_cal_roll; //1.8;                                                    //Accelerometer calibration value for roll.
  angle_pitch_acc -= acc_cal_pitch; //2.7;                                                   //Accelerometer calibration value for pitch.

  roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.

  //  Serial.print(roll);
  //  Serial.print(",");
  //  Serial.println(pitch);
}

void calculate_moving_average() {
  gyro_x_sum += gyro_x_raw;
  gyro_x_sum -= gyro_x_mem[gyro_loop_counter];
  gyro_x_mem[gyro_loop_counter] = gyro_x_raw;
  gyro_x = gyro_x_sum / 8.0;

  gyro_y_sum += gyro_y_raw;
  gyro_y_sum -= gyro_y_mem[gyro_loop_counter];
  gyro_y_mem[gyro_loop_counter] = gyro_y_raw;
  gyro_y = gyro_y_sum / 8.0;

  gyro_z_sum += gyro_z_raw;
  gyro_z_sum -= gyro_z_mem[gyro_loop_counter];
  gyro_z_mem[gyro_loop_counter] = gyro_z_raw;
  gyro_z = gyro_z_sum / 8.0;

  acc_x_sum += acc_x_raw;
  acc_x_sum -= acc_x_mem[acc_loop_counter];
  acc_x_mem[acc_loop_counter] = acc_x_raw;
  acc_x = acc_x_sum / 16.0;

  acc_y_sum += acc_y_raw;
  acc_y_sum -= acc_y_mem[acc_loop_counter];
  acc_y_mem[acc_loop_counter] = acc_y_raw;
  acc_y = acc_y_sum / 16.0;

  acc_z_sum += acc_z_raw;
  acc_z_sum -= acc_z_mem[acc_loop_counter];
  acc_z_mem[acc_loop_counter] = acc_z_raw;
  acc_z = acc_z_sum / 16.0;

  if (gyro_loop_counter == 7) gyro_loop_counter = 0;
  else gyro_loop_counter++;

  if (acc_loop_counter == 15) acc_loop_counter = 0;
  else acc_loop_counter++;
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}

void calculate_heading() {
  byte data[6];

  //  Wire.beginTransmission(mag_address);                                   //Start communication with the gyro.
  //  Wire.write(0x03);                                                       //Start reading @ register 3Bh and auto increment with every read.
  //  Wire.endTransmission();                                                 //End the transmission.
  //  Wire.requestFrom(mag_address, 6);                                     //Request 14 bytes from the gyro.
  //
  //  while(Wire.available() < 6);
  //  for(int i = 0; i < 6; i++) data[i] = Wire.read();
  //  mag_x_raw = data[1] << 8 | data[0];
  //  mag_y_raw = data[3] << 8 | data[2];
  //  mag_z_raw = data[5] << 8 | data[4];

  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(mag_address);
    Wire.write((3 + i));
    Wire.endTransmission();
    Wire.requestFrom(mag_address, 1);

    if (Wire.available() == 1) data[i] = Wire.read();
  }

  mag_x_raw = ((data[1] * 256) + data[0]);
  mag_y_raw = ((data[3] * 256) + data[2]);
  mag_z_raw = ((data[5] * 256) + data[4]);

  float mag_x = 0.15 * mag_x_raw;

  Serial.println(mag_x_raw);
}

