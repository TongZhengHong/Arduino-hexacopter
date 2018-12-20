#include <Wire.h>
#include <EEPROM.h>

//#define CAL

#define GYRO_ADDR 0x6B
#define ACC_ADDR 0x1D

typedef union {
  float decimal;
  uint8_t bytes[4];
} converter;

converter number;

bool first_start = true;
unsigned long difference, main_loop_timer;

double gyro_cal[4], accel_cal[6], compass_cal_values[6];
double acc_cal_roll, acc_cal_pitch;

int acc_x_mem[16], acc_y_mem[16], acc_z_mem[16];
int gyro_x_mem[8], gyro_y_mem[8], gyro_z_mem[8];
long acc_x_sum, acc_y_sum, acc_z_sum, gyro_x_sum, gyro_y_sum, gyro_z_sum;
byte gyro_loop_counter = 0, acc_loop_counter = 0;

int roll, pitch;
float angle_roll, angle_pitch, angle_yaw;
float angle_roll_acc, angle_pitch_acc;

double compass_x, compass_y;
float compass_heading, heading;
float mag_x_offset, mag_y_offset, mag_z_offset;
float mag_x_scale, mag_y_scale, mag_z_scale;

int gyro_x_raw, gyro_y_raw, gyro_z_raw;
int acc_x_raw, acc_y_raw, acc_z_raw;
int mag_x_raw, mag_y_raw, mag_z_raw;

float gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  setup_sensor();
#ifdef CAL
  delay(3000);
  calibrate_mag();
  delay(1000);
#endif
  calibrate_sensor();

  number.bytes[0] = EEPROM.read(0);
  number.bytes[1] = EEPROM.read(1);
  number.bytes[2] = EEPROM.read(2);
  number.bytes[3] = EEPROM.read(3);
  mag_x_scale = number.decimal;

  number.bytes[0] = EEPROM.read(4);
  number.bytes[1] = EEPROM.read(5);
  number.bytes[2] = EEPROM.read(6);
  number.bytes[3] = EEPROM.read(7);
  mag_x_offset = number.decimal;

  number.bytes[0] = EEPROM.read(8);
  number.bytes[1] = EEPROM.read(9);
  number.bytes[2] = EEPROM.read(10);
  number.bytes[3] = EEPROM.read(11);
  mag_y_scale = number.decimal;

  number.bytes[0] = EEPROM.read(12);
  number.bytes[1] = EEPROM.read(13);
  number.bytes[2] = EEPROM.read(14);
  number.bytes[3] = EEPROM.read(15);
  mag_y_offset =  number.decimal;

  number.bytes[0] = EEPROM.read(16);
  number.bytes[1] = EEPROM.read(17);
  number.bytes[2] = EEPROM.read(18);
  number.bytes[3] = EEPROM.read(19);
  mag_z_scale = number.decimal;

  number.bytes[0] = EEPROM.read(20);
  number.bytes[1] = EEPROM.read(21);
  number.bytes[2] = EEPROM.read(22);
  number.bytes[3] = EEPROM.read(23);
  mag_z_offset = number.decimal;

  Serial.print("X Scale: ");
  Serial.println(mag_x_scale);
  Serial.print("X Offset: ");
  Serial.println(mag_x_offset);
  Serial.print("Y Scale: ");
  Serial.println(mag_y_scale);
  Serial.print("Y Offset: ");
  Serial.println(mag_y_offset);
  Serial.print("Z Scale: ");
  Serial.println(mag_z_scale);
  Serial.print("Z Offset: ");
  Serial.println(mag_z_offset);

  Serial.println("Setup DONE");
  delay(5000);
}

void loop() {
  get_data();

  calculate_pitch_roll();

  calculate_heading();

  maintain_loop_time();
}

void get_data() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(GYRO_ADDR);
    Wire.write((40 + i));
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 1);

    if (Wire.available() == 1) data[i] = Wire.read();
  }

  gyro_x_raw = ((data[1] * 256) + data[0]);
  gyro_y_raw = ((data[3] * 256) + data[2]);
  gyro_z_raw = ((data[5] * 256) + data[4]);

  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(ACC_ADDR);
    Wire.write((40 + i));
    Wire.endTransmission();
    Wire.requestFrom(ACC_ADDR, 1);

    if (Wire.available() == 1) data[i] = Wire.read();
  }

  acc_x_raw = ((data[1] * 256) + data[0]);
  acc_y_raw = ((data[3] * 256) + data[2]);
  acc_z_raw = ((data[5] * 256) + data[4]);

  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(ACC_ADDR);
    Wire.write((8 + i));
    Wire.endTransmission();
    Wire.requestFrom(ACC_ADDR, 1);

    if (Wire.available() == 1) data[i] = Wire.read();
  }

  mag_x_raw = ((data[1] * 256) + data[0]);
  mag_y_raw = ((data[3] * 256) + data[2]);
  mag_z_raw = ((data[5] * 256) + data[4]);

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

  gyro_x = gyro_x_raw * 0.0175;
  gyro_y = gyro_y_raw * 0.0175;
  gyro_z = gyro_z_raw * 0.0175;

  mag_x = (mag_x_raw - mag_x_offset) * mag_x_scale;
  mag_y = (mag_y_raw - mag_y_offset) * mag_y_scale;
  mag_z = (mag_z_raw - mag_z_offset) * mag_z_scale;

//  Serial.print(mag_x);
//  Serial.print(",");
//  Serial.print(mag_y);
//  Serial.print(",");
//  Serial.println(mag_z);
}

void calculate_pitch_roll() {
  gyro_z *= -1;
  acc_z *= -1;

  angle_roll += gyro_y * 0.005;
  angle_pitch += gyro_x * 0.005;
  angle_yaw += gyro_z * 0.005;

  angle_pitch_acc = (float) (atan2(acc_y_raw, acc_z_raw)) * RAD_TO_DEG;
  angle_roll_acc = (float) (atan2(acc_z_raw, acc_x_raw)) * RAD_TO_DEG;

  angle_roll_acc -= (float) 90.0;

  if (first_start) {
    angle_roll = angle_roll_acc;
    angle_pitch = angle_pitch_acc;
  }

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.

  roll = angle_roll;
  pitch = angle_pitch;

    Serial.print(roll);
    Serial.print(",");
    Serial.println(pitch);
}

void calculate_heading() {
  float roll_angle = roll * DEG_TO_RAD;
  float pitch_angle = pitch * DEG_TO_RAD * -1;

  compass_x = mag_x * cos(pitch_angle)
              + mag_y * sin(roll_angle) * sin(pitch_angle)
              - mag_z * cos(roll_angle) * sin(pitch_angle);

  compass_y = mag_y * cos(roll_angle)
              + mag_z * sin(roll_angle);

  //  if (compass_x < 0) {
  //    compass_heading = 180 - atan2(compass_y, compass_x) * RAD_DEG;
  //  } else if (compass_x > 0) {
  //    if (compass_y < 0) compass_heading = - atan2(compass_y, compass_x) * RAD_DEG;
  //    else if (compass_y > 0) 360 - atan2(compass_y, compass_x) * RAD_DEG;
  //  } else {
  //    if (compass_y < 0) compass_heading = 90;
  //    else if (compass_y > 0) compass_heading = 270;
  //  }

  if (compass_x == 0) {
    if (compass_y < 0) compass_heading = 90;
    else compass_heading = 0;
  } else compass_heading = atan2(compass_y, compass_x) * RAD_DEG;

  if (compass_heading < 0) {
    compass_heading = 360 + compass_heading;
  }

  if (first_start)  {
    angle_yaw = compass_heading;
    first_start = false;
  }

  //  heading = angle_yaw * 0.96 + compass_heading * 0.04;
  //
  //  int temp_heading = heading * 2;
  //  if (temp_heading >= 360) temp_heading %= 360;
  //  else if (temp_heading < 0) temp_heading += 359;

  Serial.println(compass_heading);
  Serial.println();
}

void calibrate_mag() {
  Serial.println("Calibrating compass...");
  unsigned long start = millis();
  unsigned long difference = 0;
  while (difference < 50000) {
    Serial.println(difference);
    difference = millis() - start;
    unsigned int data[6];
    for (int i = 0; i < 6; i++) {
      Wire.beginTransmission(ACC_ADDR);
      Wire.write((8 + i));
      Wire.endTransmission();
      Wire.requestFrom(ACC_ADDR, 1);

      if (Wire.available() == 1) data[i] = Wire.read();
    }

    mag_x_raw = ((data[1] * 256) + data[0]);
    mag_y_raw = ((data[3] * 256) + data[2]);
    mag_z_raw = ((data[5] * 256) + data[4]);

    if (mag_x_raw < compass_cal_values[0])compass_cal_values[0] = mag_x_raw;
    if (mag_x_raw > compass_cal_values[1])compass_cal_values[1] = mag_x_raw;
    if (mag_y_raw < compass_cal_values[2])compass_cal_values[2] = mag_y_raw;
    if (mag_y_raw > compass_cal_values[3])compass_cal_values[3] = mag_y_raw;
    if (mag_z_raw < compass_cal_values[4])compass_cal_values[4] = mag_z_raw;
    if (mag_z_raw > compass_cal_values[5])compass_cal_values[5] = mag_z_raw;
  }

  Serial.print("X LOW: ");
  Serial.println(compass_cal_values[0]);
  Serial.print("X HIGH: ");
  Serial.println(compass_cal_values[1]);
  Serial.print("Y LOW: ");
  Serial.println(compass_cal_values[2]);
  Serial.print("Y HIGH: ");
  Serial.println(compass_cal_values[3]);
  Serial.print("Z LOW: ");
  Serial.println(compass_cal_values[4]);
  Serial.print("Z HIGH: ");
  Serial.println(compass_cal_values[5]);

  mag_x_offset = (compass_cal_values[1] + compass_cal_values[0]) / 2;
  mag_y_offset = (compass_cal_values[3] + compass_cal_values[2]) / 2;
  mag_z_offset = (compass_cal_values[5] + compass_cal_values[4]) / 2;

  float avg_delta_x = (compass_cal_values[1] - compass_cal_values[0]) / 2;
  float avg_delta_y = (compass_cal_values[3] - compass_cal_values[2]) / 2;
  float avg_delta_z = (compass_cal_values[5] - compass_cal_values[4]) / 2;

  float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

  mag_x_scale = avg_delta / avg_delta_x;
  mag_y_scale = avg_delta / avg_delta_y;
  mag_z_scale = avg_delta / avg_delta_z;

  Serial.print("X Scale: ");
  Serial.println(mag_x_scale);
  Serial.print("X Offset: ");
  Serial.println(mag_x_offset);
  Serial.print("Y Scale: ");
  Serial.println(mag_y_scale);
  Serial.print("Y Offset: ");
  Serial.println(mag_y_offset);
  Serial.print("Z Scale: ");
  Serial.println(mag_z_scale);
  Serial.print("Z Offset: ");
  Serial.println(mag_z_offset);

  number.decimal = mag_x_scale;
  EEPROM.write(0, number.bytes[0]);
  EEPROM.write(1, number.bytes[1]);
  EEPROM.write(2, number.bytes[2]);
  EEPROM.write(3, number.bytes[3]);

  number.decimal = mag_x_offset;
  EEPROM.write(4, number.bytes[0]);
  EEPROM.write(5, number.bytes[1]);
  EEPROM.write(6, number.bytes[2]);
  EEPROM.write(7, number.bytes[3]);

  number.decimal = mag_y_scale;
  EEPROM.write(8, number.bytes[0]);
  EEPROM.write(9, number.bytes[1]);
  EEPROM.write(10, number.bytes[2]);
  EEPROM.write(11, number.bytes[3]);

  number.decimal = mag_y_offset;
  EEPROM.write(12, number.bytes[0]);
  EEPROM.write(13, number.bytes[1]);
  EEPROM.write(14, number.bytes[2]);
  EEPROM.write(15, number.bytes[3]);

  number.decimal = mag_z_scale;
  EEPROM.write(16, number.bytes[0]);
  EEPROM.write(17, number.bytes[1]);
  EEPROM.write(18, number.bytes[2]);
  EEPROM.write(19, number.bytes[3]);

  number.decimal = mag_z_offset;
  EEPROM.write(20, number.bytes[0]);
  EEPROM.write(21, number.bytes[1]);
  EEPROM.write(22, number.bytes[2]);
  EEPROM.write(23, number.bytes[3]);
}

void calibrate_sensor() {
  Serial.print("Calibrating sensor");
  for (int i = 0; i < 2000; i++) {
    byte data[6];
    if (i % 200 == 0) Serial.print(".");
    for (int i = 0; i < 6; i++) {
      Wire.beginTransmission(GYRO_ADDR);
      Wire.write((40 + i));
      Wire.endTransmission();
      Wire.requestFrom(GYRO_ADDR, 1);
      if (Wire.available() == 1) data[i] = Wire.read();
    }

    gyro_cal[1] += ((data[1] * 256) + data[0]);
    gyro_cal[2] += ((data[3] * 256) + data[2]);
    gyro_cal[3] += ((data[5] * 256) + data[4]);
  }

  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;
  gyro_cal[3] /= 2000;

  Serial.println();
  Serial.println("Gyroscope calibration done!");
  for (int i = 1; i < 4; i++) Serial.println(gyro_cal[i]);
}

void setup_sensor() {
  Wire.beginTransmission(GYRO_ADDR);      //GYROSCOPE
  Wire.write(0x20);                       //Control register 1
  Wire.write(0x4F);                       //Write 01001111 --> 190Hz, 12.5 cutoff, Power up
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDR);      //GYROSCOPE
  Wire.write(0x23);                       //Control register 4
  Wire.write(0x10);                       //Write 00010000 --> 500dps
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDR);       //ACCELEROMETER
  Wire.write(0x20);                       //Control register 1
  Wire.write(0x77);                       //Write 01110111 --> 200Hz, Power up
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDR);       //ACCELEROMETER
  Wire.write(0x21);                       //Control register 2
  Wire.write(0x18);                       //Write 00011000 --> +/- 8g
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDR);       //MAGNETOMETER
  Wire.write(0x24);                       //Control register 5
  Wire.write(0x74);                       //Write 01110100 --> Disable temp, high res, 100Hz
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDR);       //MAGNETOMETER
  Wire.write(0x25);                       //Control register 6
  Wire.write(0x40);                       //Write 01000000 -->  +/-8 gauss
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDR);       //MAGNETOMETER
  Wire.write(0x26);                       //Control register 7
  Wire.write(0x00);                       //Write 00000000 --> Continuous-conversion mode
  Wire.endTransmission();
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}

