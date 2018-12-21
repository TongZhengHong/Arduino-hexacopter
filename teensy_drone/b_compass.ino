bool first_start = true;

void calculate_heading() {
  uint8_t data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(ACC_ADDR);
    Wire.write((8 + i));
    Wire.endTransmission();
    Wire.requestFrom(ACC_ADDR, 1);

    if (Wire.available() == 1) data[i] = Wire.read();
  }

  mag_x_raw = (int16_t) data[1] << 8 | (int16_t) data[0];
  mag_y_raw = (int16_t) data[3] << 8 | (int16_t) data[2];
  mag_z_raw = (int16_t) data[5] << 8 | (int16_t) data[4];

  //Apply magnetometer calibration scales and offsets to raw output
  mag_x = (mag_x_raw - mag_cal[1]) * mag_cal[0];
  mag_y = (mag_y_raw - mag_cal[3]) * mag_cal[2];
  mag_z = (mag_z_raw - mag_cal[5]) * mag_cal[4];

  //Tilt compensation
  float roll_angle = roll * DEG_TO_RAD;
  float pitch_angle = pitch * DEG_TO_RAD; 
  
  compass_x = mag_x * cos(pitch_angle)
              + mag_y * sin(roll_angle) * sin(pitch_angle)
              - mag_z * cos(roll_angle) * sin(pitch_angle);

  compass_y = mag_y * cos(roll_angle)
              + mag_z * sin(roll_angle);

  if (compass_x == 0) {
    if (compass_y < 0) compass_heading = 90;
    else compass_heading = 0;
  } else compass_heading = atan2(compass_y, compass_x) * RAD_TO_DEG;

  if (compass_heading < 0) {
    compass_heading = 360 + compass_heading;
  }

  if (first_start) {
    angle_yaw = compass_heading;
    first_start = false;
  }

  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;

  heading = angle_yaw * 0.99 + compass_heading * 0.01;
  
//  Serial.println(heading);
}

