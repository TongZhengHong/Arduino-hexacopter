#include <EEPROM.h>

int start;
byte eeprom_data[27];
unsigned long startTime, difference;

volatile int receiver_input[6];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;

unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, current_time;

//Transmitter variables
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

void setup() {
  Serial.begin(115200);
  delay(1000);

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 11)to trigger an interrupt on state change.

  DDRD |= B11110000;

  for (start = 0; start <= 26; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[24] != 'J' || eeprom_data[25] != 'M' || eeprom_data[26] != 'B')
    delay(10);

  Serial.println("Connect battery");
  for (int i = 1; i < 6; i++) {
    delay(1000);
    Serial.print((String) i + " ");
  }

  Serial.println("HIGH");
  startTime = millis();
  difference = 0;
  while (difference < 5000) {
    difference = millis() - startTime;
    PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
    delayMicroseconds(2000);
    PORTD &= B00001111;
    delay(3);
  }

  Serial.println("LOW");
  startTime = millis();
  difference = 0;
  while (difference < 5000) {
    difference = millis() - startTime;
    PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
  }

  Serial.println("DONE");
}

void loop() {
  //For manual calibration:
  //receiver_input_channel_3 = convert_receiver_channel(3);
  
  PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
  delayMicroseconds(1000);
  PORTD &= B00001111;
  delay(3);
}

ISR(PCINT0_vect) {
  current_time = micros();
  //* ========================================= Channel 1 =========================================
  if (PINB & B00010000) {                                                   //Is input 8 high?
    if (last_channel_1 == 0) {                                              //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //* ========================================= Channel 2 =========================================
  if (PINB & B00001000) {                                                  //Is input 9 high?
    if (last_channel_2 == 0) {                                              //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //* ========================================= Channel 3 =========================================
  if (PINB & B00000100) {                                                  //Is input 10 high?
    if (last_channel_3 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
  }
  //* ========================================= Channel 4 =========================================
  if (PINB & B00000010) {                                                  //Is input 11 high?
    if (last_channel_4 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //* ========================================= Channel 5 =========================================
  if (PINB & B00000001) {                                                  //Is input 12 high?
    if (last_channel_5 == 0) {                                              //Input 12 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if (last_channel_5 == 1) {                                           //Input 12 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
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
