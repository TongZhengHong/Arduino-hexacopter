
void set_pid_offsets() {
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1508)
      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if (receiver_input_channel_1 < 1492)
      pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust; //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;               //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
  }

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1508)
      pid_pitch_setpoint = 1508 - receiver_input_channel_2;
    else if (receiver_input_channel_2 < 1492)
      pid_pitch_setpoint = 1492 - receiver_input_channel_2;

    pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
  }

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508) {
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
      //      prev_heading = heading;
    } else if (receiver_input_channel_4 < 1492) {
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
      //      prev_heading = heading;
    }
    //    else {
    //      pid_yaw_setpoint = prev_heading;
    //      heading_hold = true;
    //    }
  }
  //    Serial.print(pid_roll_setpoint);
  //    Serial.print(" ");
  //    Serial.print(pid_pitch_setpoint);
  //    Serial.print(" ");
  //    Serial.print(pid_yaw_setpoint);
  //    Serial.println();
}

void calculate_pid() {
  //* ========================================= Roll calculations =========================================
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //* ========================================= Pitch calculations =========================================
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //* ========================================= Yaw calculations =========================================
  //  if (heading_hold) {
  //    float diff = heading - pid_yaw_setpoint;
  //    if (diff > 180) pid_yaw_setpoint += 360;
  //    else if (diff < -180) pid_yaw_setpoint -= 360;
  //    pid_error_temp = (pid_yaw_setpoint - heading) * 2;
  //    heading_hold = false;
  //  } else pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;

  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

//  Serial.print(" ");
//  Serial.println(pid_output_roll);
//  Serial.println(pid_output_pitch);
//  Serial.println(pid_output_yaw);
//  Serial.println();
}
