volatile int receiver_input[7];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;

void convert_transmitter_values() {
  receiver_input_channel_1 = convert_receiver_channel(1); //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2); //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us

//  Serial.print(receiver_input_channel_1);
//  Serial.print(", ");
//  Serial.print(receiver_input_channel_2);
//  Serial.print(", ");
//  Serial.print(receiver_input_channel_3);
//  Serial.print(", ");
//  Serial.print(receiver_input_channel_4);
//  Serial.println();
}

int convert_receiver_channel(byte function) {
  int low, center, high, actual;
  int difference;

  actual = receiver_input[function];                                             //Read the actual receiver value for the corresponding function
  low = (eeprom_data[function * 2 + 15] << 8) | eeprom_data[function * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[function * 2 - 1] << 8) | eeprom_data[function * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[function * 2 + 7] << 8) | eeprom_data[function * 2 + 6];   //Store the high value for the specific receiver input channel

  if (actual < center)  { //The actual receiver value is lower than the center value
    if (actual < low)
      actual = low;                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low); //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 - difference;
  }
  else if (actual > center)  { //The actual receiver value is higher than the center value
    if (actual > high)
      actual = high;                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center); //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 + difference;
  }
  else
    return 1500;
}

void receiver_change() {
  current_time = micros();
  //* ========================================= Channel 1 =========================================
  if (GPIOD_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTD1
    if (last_channel_1 == 0) {                                              //Input 14 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_1 == 1) {                                         //Input 14 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //* ========================================= Channel 2 =========================================
  if (GPIOC_PDIR & 1) { //0000 0000 0000 0000 0000 0000 0000 0001 --> PTC0
    if (last_channel_2 == 0) {                                              //Input 15 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  } else if (last_channel_2 == 1) {                                          //Input 15 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //* ========================================= Channel 3 =========================================
  if (GPIOB_PDIR & 1) { //0000 0000 0000 0000 0000 0000 0000 0001 --> PTB0
    if (last_channel_3 == 0) {                                              //Input 16 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  } else if (last_channel_3 == 1) {                                          //Input 16 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
  }
  //* ========================================= Channel 4 =========================================
  if (GPIOB_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTB1
    if (last_channel_4 == 0) {                                              //Input 17 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  } else if (last_channel_4 == 1) {                                          //Input 17 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //* ========================================= Channel 5 =========================================
  if (GPIOC_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTC1
    if (last_channel_5 == 0) {                                              //Input 22 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  } else if (last_channel_5 == 1) {                                          //Input 22 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
  //* ========================================= Channel 6 =========================================
  if (GPIOC_PDIR & 4) { //0000 0000 0000 0000 0000 0000 0000 0100 --> PTC2
    if (last_channel_6 == 0) {                                              //Input 23 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  } else if (last_channel_6 == 1) {                                          //Input 23 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}
