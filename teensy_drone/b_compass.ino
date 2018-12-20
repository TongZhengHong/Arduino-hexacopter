
void calculate_heading() {
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

  //Apply magnetometer calibration scales and offsets to raw output
  mag_x = (mag_x_raw - mag_x_offset) * mag_x_scale;
  mag_y = (mag_y_raw - mag_y_offset) * mag_y_scale;
  mag_z = (mag_z_raw - mag_z_offset) * mag_z_scale;

  //Tilt compensation
  float roll_angle = roll * DEG_TO_RAD;
  float pitch_angle = pitch * DEG_TO_RAD; 
  
  compass_x = mag_x * cos(pitch_angle)
              + mag_y * sin(roll_angle) * sin(pitch_angle)
              - mag_z * cos(roll_angle) * sin(pitch_angle);

  compass_y = mag_y * cos(roll_angle)
              + mag_z * sin(roll_angle);

  Serial.println(roll_angle);
  Serial.println(pitch_angle);

  if (compass_x == 0) {
    if (compass_y < 0) compass_heading = 90;
    else compass_heading = 0;
  } else compass_heading = atan2(compass_y, compass_x) * RAD_DEG;

  if (compass_heading < 0) {
    compass_heading = 360 + compass_heading;
  }

  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;

  Serial.println(angle_yaw);

  heading = angle_yaw * 0.98 + compass_heading * 0.02;
  Serial.println(heading);
}

