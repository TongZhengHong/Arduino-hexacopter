//TODO: Check motor +/- directions

int throttle;
int esc_1, esc_2, esc_3, esc_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long esc_timer, esc_loop_timer, loop_timer;

void calculate_esc_output () {
  int battery_voltage = calculate_battery();
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw ;//Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1680 && battery_voltage > 1280) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1680 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1680 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1680 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1680 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < 1110) esc_1 = 1110;                                         //Keep the motors running.
    if (esc_2 < 1110) esc_2 = 1110;                                         //Keep the motors running.
    if (esc_3 < 1110) esc_3 = 1110;                                         //Keep the motors running.
    if (esc_4 < 1110) esc_4 = 1110;                                         //Keep the motors running.

    if (esc_1 > 2000) esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000) esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000) esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000) esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  } else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
}

void set_escs() {
  loop_timer = micros();
  PORTD |= B11110000;                   //Set digital pin 4 - 7 as HIGH

  timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16)  {                //Stay in this loop until output 4 - 7 are LOW.
    esc_loop_timer = micros();          //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B01111111; //Set digital output 7 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B10111111; //Set digital output 6 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B11011111; //Set digital output 5 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B11101111; //Set digital output 4 to low if the time is expired.
  }
}
