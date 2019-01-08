int throttle;
int esc_1, esc_2, esc_3, esc_4, esc_5, esc_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6;
unsigned long esc_timer, esc_loop_timer, loop_timer;

void calculate_esc_output () {
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1750) throttle = 1750;                                   //We need some room to keep full control at full throttle.

#ifdef QUADCOPTER
    esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
#endif

#ifdef HEXCOPTER
    esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle /*==============*/ + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    esc_5 = throttle /*==============*/ - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    esc_6 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
#endif

    if (battery_voltage < 1680 && battery_voltage > 1320) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-4 pulse for voltage drop.
      esc_5 += esc_5 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-5 pulse for voltage drop.
      esc_6 += esc_6 * ((1680 - battery_voltage) / (float) 3500);            //Compensate the esc-6 pulse for voltage drop.
    }

    //if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    //if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
    if (esc_5 < 1100) esc_5 = 1100;                                         //Keep the motors running.
    if (esc_6 < 1100) esc_6 = 1100;                                         //Keep the motors running.

    if (esc_1 > 1900) esc_1 = 1900;                                          //Limit the esc-1 pulse to 1900us.
    if (esc_2 > 1900) esc_2 = 1900;                                          //Limit the esc-2 pulse to 1900us.
    if (esc_3 > 1900) esc_3 = 1900;                                          //Limit the esc-3 pulse to 1900us.
    if (esc_4 > 1900) esc_4 = 1900;                                          //Limit the esc-4 pulse to 1900us.
    if (esc_5 > 1900) esc_5 = 1900;                                          //Limit the esc-5 pulse to 1900us.
    if (esc_6 > 1900) esc_6 = 1900;                                          //Limit the esc-6 pulse to 1900us.
  } else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-4.
    esc_5 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-5.
    esc_6 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc-6.
  }
}

void set_escs() {
  loop_timer = micros();
  
#ifdef QUADCOPTER
  GPIOD_PSOR |= 180;    //0000 0000 0000 0000 0000 0000 1011 0100 --> Setting pins 7,6,5,20 as HIGH

  timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the falling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the falling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the falling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the falling edge of the esc-4 pulse.

  while (GPIOD_PDOR >= 4)  {                //Stay in this loop until output 4 - 7 are LOW.
    esc_loop_timer = micros();          //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) GPIOD_PCOR |= 4;    //0000 0000 0000 0000 0000 0000 0000 0100 --> Setting pins 7 (PTD2) as LOW
    if (timer_channel_2 <= esc_loop_timer) GPIOD_PCOR |= 16;   //0000 0000 0000 0000 0000 0000 0001 0000 --> Setting pins 6 (PTD4) as LOW
    if (timer_channel_3 <= esc_loop_timer) GPIOD_PCOR |= 128;  //0000 0000 0000 0000 0000 0000 1000 0000 --> Setting pins 5 (PTD7) as LOW
    if (timer_channel_4 <= esc_loop_timer) GPIOD_PCOR |= 32;   //0000 0000 0000 0000 0000 0000 0010 0000 --> Setting pins 20 (PTD5) as LOW
  }
#endif

#ifdef HEXCOPTER
  GPIOD_PSOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as HIGH

  timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the falling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the falling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the falling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the falling edge of the esc-4 pulse.
  timer_channel_5 = esc_5 + loop_timer; //Calculate the time of the falling edge of the esc-5 pulse.
  timer_channel_6 = esc_6 + loop_timer; //Calculate the time of the falling edge of the esc-6 pulse.

  while (GPIOD_PDOR >= 4)  {                //Stay in this loop until output 4 - 7 are LOW.
    esc_loop_timer = micros();          //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) GPIOD_PCOR |= 8;    //0000 0000 0000 0000 0000 0000 0000 1000 --> Setting pins 8 (PTD3) as LOW
    if (timer_channel_2 <= esc_loop_timer) GPIOD_PCOR |= 4;    //0000 0000 0000 0000 0000 0000 0000 0100 --> Setting pins 7 (PTD2) as LOW
    if (timer_channel_3 <= esc_loop_timer) GPIOD_PCOR |= 16;   //0000 0000 0000 0000 0000 0000 0001 0000 --> Setting pins 6 (PTD4) as LOW
    if (timer_channel_4 <= esc_loop_timer) GPIOD_PCOR |= 128;  //0000 0000 0000 0000 0000 0000 1000 0000 --> Setting pins 5 (PTD7) as LOW
    if (timer_channel_5 <= esc_loop_timer) GPIOD_PCOR |= 32;   //0000 0000 0000 0000 0000 0000 0010 0000 --> Setting pins 20 (PTD5) as LOW
    if (timer_channel_6 <= esc_loop_timer) GPIOD_PCOR |= 64;   //0000 0000 0000 0000 0000 0000 0100 0000 --> Setting pins 21 (PTD6) as LOW
  }
#endif
}
