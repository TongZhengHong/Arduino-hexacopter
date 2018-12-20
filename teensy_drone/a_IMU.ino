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

void calibrate_sensors() {
  Serial.print("Calibrating sensor");
  for (int i = 0; i < 2000; i++) {
    if (i % 200 == 0) Serial.print(".");

    byte data[6];
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

  Serial.print("Calibrating accelerometer offsets");
  for (int i = 0; i < 2000; i++) {
    if (i % 200 == 0) Serial.print(".");
    calibrate_accel();
    acc_cal_pitch += angle_pitch;
    acc_cal_roll += angle_roll;

    pulse_esc();
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

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

  gyro_z_raw *= -1;
  acc_z_raw *= -1;

  acc_x_raw = accel_cal[0] * acc_x_raw + accel_cal[1];
  acc_y_raw = accel_cal[2] * acc_y_raw + accel_cal[3];
  acc_z_raw = accel_cal[4] * acc_z_raw + accel_cal[5];

  //Gyro calculations 0.0000875 = (0.005 * 0.0175)
  angle_roll += gyro_y_raw * 0.0000875;
  angle_pitch += gyro_x_raw * 0.0000875;

  angle_roll_acc = (float) (atan2(acc_z_raw, acc_x_raw)) * RAD_TO_DEG;
  angle_pitch_acc = (float) (atan2(acc_y_raw, acc_z_raw)) * RAD_TO_DEG;
  angle_roll_acc -= (float) 90.0;

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
}

void calculate_pitch_roll() {
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x / 65.5) * 0.3);     //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y / 65.5) * 0.3);     //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) * 0.3);         //Gyro pid input is deg/sec.

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

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

  gyro_z_raw *= -1;
  acc_z_raw *= -1;

  calculate_moving_average();

  acc_x = accel_cal[0] * acc_x + accel_cal[1];
  acc_y = accel_cal[2] * acc_y + accel_cal[3];
  acc_z = accel_cal[4] * acc_z + accel_cal[5];

  //Gyro calculations 0.0000875 = (0.005 * 0.0175)
  angle_roll += gyro_y * 0.0000875;
  angle_pitch += gyro_x * 0.0000875;
  angle_yaw += gyro_z * 0.0000875;

  //Accelerometer angle calculations
  angle_roll_acc = (float) (atan2(acc_y, acc_z)) * RAD_TO_DEG;
  angle_pitch_acc = (float) (atan2(acc_z, acc_x))* RAD_TO_DEG;
  angle_roll_acc -= (float) 90.0;

  angle_roll_acc -= acc_cal_roll;                                       //Accelerometer calibration value for roll.
  angle_pitch_acc -= acc_cal_pitch;                                     //Accelerometer calibration value for pitch.

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.

  roll = angle_roll;
  pitch = angle_pitch;

  roll_level_adjust = roll * 15;                                      //Calculate the roll angle correction
  pitch_level_adjust = pitch * 15;                                    //Calculate the pitch angle correction

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

