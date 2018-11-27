int imu_address = 104;

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

void calibrateSensors() {
  Serial.println("Calibrating sensor...");
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
    Wire.write(0x43);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(imu_address, 6);                                      //Request 14 bytes from the gyro.

    while (Wire.available() < 6);                                          //Wait until the 6 bytes are received.
    gyro_cal[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[3] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

    pulse_esc();
  }

  gyro_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  Serial.println("Gyroscope calibration done!");
  for (int i = 1; i < 4; i++) Serial.println(gyro_cal[i]);
}

void calculate_pitch_roll() {
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll/65.5) * 0.3);       //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch/65.5) * 0.3);       //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw/65.5) * 0.3);           //Gyro pid input is deg/sec.

  Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(imu_address, 14);                                     //Request 14 bytes from the gyro.

  while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
  acc_y = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  acc_x = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
  gyro_pitch = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_roll = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_yaw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

  gyro_pitch -= gyro_cal[1];
  gyro_roll -= gyro_cal[2];
  gyro_yaw -= gyro_cal[3];

  //Gyro calculations 0.000076336 = (0.005 / 65.5)
  angle_pitch += gyro_pitch * 0.000076336 * -1;            
  angle_roll += gyro_roll * 0.000076336;
  
  //0.0000013325 = 0.000076336 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.0000013325);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.0000013325);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* -57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  Serial.println(angle_pitch);
  Serial.println(angle_roll);
  Serial.println();
}