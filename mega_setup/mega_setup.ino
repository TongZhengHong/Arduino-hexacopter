#include <Wire.h>
#include <EEPROM.h>

int error = 0;

//Transmitter variables
unsigned long timer, timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, timer_7, timer_8, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6, last_channel_7, last_channel_8;
volatile int receiver_input[9];

//EEPROM variables
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;

void setup(){
  Serial.begin(9600);

  for(int i = 0; i < 9; i++){
    receiver_input[i] = 0;
  }

  PCICR |= (1 << PCIE2);     // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port K.
  
  PCMSK2 |= (1 << PCINT16);  // set PCINT16 (digital input A8) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT17);  // set PCINT17 (digital input A9)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 (digital input A10)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19);  // set PCINT19 (digital input A11)to trigger an interrupt on state change
  
  PCMSK2 |= (1 << PCINT20);  // set PCINT20 (digital input A12) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT21);  // set PCINT21 (digital input A13)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT22);  // set PCINT22 (digital input A14)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT23);  // set PCINT23 (digital input A15)to trigger an interrupt on state change
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if(error == 0)  {
    wait_for_receiver();  //Wait for transmitter to be turned on within 10 seconds
  }

  if(error == 0)  {
    record_center_position(); //Find out the center positions of transmitter (including throttle)
  }

  if(error == 0)  { 
    register_min_max();   //Register the min and max values of the receiver channels
  }
  
  if(error == 0){
    save_eeprom_data();   //If all is good, store the information in the EEPROM
  }
  
  if(error == 0) {
    Serial.println("Setup SUCCESS!");
    Serial.println("You may close the program now :)");
  } else {
    Serial.println("There is an error during setup. Please try again.");
  }
  
  while(1);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void save_eeprom_data(){
  Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);

    EEPROM.write(0, center_channel_1 & 0b11111111);
    EEPROM.write(1, center_channel_1 >> 8);
    EEPROM.write(2, center_channel_2 & 0b11111111);
    EEPROM.write(3, center_channel_2 >> 8);
    EEPROM.write(4, center_channel_3 & 0b11111111);
    EEPROM.write(5, center_channel_3 >> 8);
    EEPROM.write(6, center_channel_4 & 0b11111111);
    EEPROM.write(7, center_channel_4 >> 8);
    EEPROM.write(8, high_channel_1 & 0b11111111);
    EEPROM.write(9, high_channel_1 >> 8);
    EEPROM.write(10, high_channel_2 & 0b11111111);
    EEPROM.write(11, high_channel_2 >> 8);
    EEPROM.write(12, high_channel_3 & 0b11111111);
    EEPROM.write(13, high_channel_3 >> 8);
    EEPROM.write(14, high_channel_4 & 0b11111111);
    EEPROM.write(15, high_channel_4 >> 8);
    EEPROM.write(16, low_channel_1 & 0b11111111);
    EEPROM.write(17, low_channel_1 >> 8);
    EEPROM.write(18, low_channel_2 & 0b11111111);
    EEPROM.write(19, low_channel_2 >> 8);
    EEPROM.write(20, low_channel_3 & 0b11111111);
    EEPROM.write(21, low_channel_3 >> 8);
    EEPROM.write(22, low_channel_4 & 0b11111111);
    EEPROM.write(23, low_channel_4 >> 8);
    Serial.println(F("Done!"));
    
    //Write the EEPROM signature
    EEPROM.write(24, 'J'); 
    EEPROM.write(25, 'M');
    EEPROM.write(26, 'B');
        
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if('J' != EEPROM.read(24))error = 1;
    if('M' != EEPROM.read(25))error = 1;
    if('B' != EEPROM.read(26))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!!"));
    else Serial.println(F("Verification done"));
}

void register_min_max(){
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("Gently move all the sticks simultaneously to their extends"));
  Serial.println(F("When ready put the sticks back in their center positions"));

  byte zero = 0;
  low_channel_1 = receiver_input[1];
  low_channel_2 = receiver_input[2];
  low_channel_3 = receiver_input[3];
  low_channel_4 = receiver_input[4];
  
  while(receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20) delay(250); //check if channel 1 centered
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){
    if(receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20)  zero |= 0b00000001;
    if(receiver_input[2] < center_channel_2 + 20 && receiver_input[2] > center_channel_2 - 20)  zero |= 0b00000010;
    if(receiver_input[3] < center_channel_3 + 20 && receiver_input[3] > center_channel_3 - 20)  zero |= 0b00000100;
    if(receiver_input[4] < center_channel_4 + 20 && receiver_input[4] > center_channel_4 - 20)  zero |= 0b00001000;
    
    if(receiver_input[1] < low_channel_1) low_channel_1 = receiver_input[1];
    if(receiver_input[2] < low_channel_2) low_channel_2 = receiver_input[2];
    if(receiver_input[3] < low_channel_3) low_channel_3 = receiver_input[3];
    if(receiver_input[4] < low_channel_4) low_channel_4 = receiver_input[4];
    
    if(receiver_input[1] > high_channel_1)  high_channel_1 = receiver_input[1];
    if(receiver_input[2] > high_channel_2)  high_channel_2 = receiver_input[2];
    if(receiver_input[3] > high_channel_3)  high_channel_3 = receiver_input[3];
    if(receiver_input[4] > high_channel_4)  high_channel_4 = receiver_input[4];
    
    delay(100);
  }
  
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("High, low and center values found during setup"));
  Serial.print(F("Digital input 08 values: "));
  Serial.print(low_channel_1);
  Serial.print(F(" - "));
  Serial.print(center_channel_1);
  Serial.print(F(" - "));
  Serial.println(high_channel_1);
  Serial.print(F("Digital input 09 values: "));
  Serial.print(low_channel_2);
  Serial.print(F(" - "));
  Serial.print(center_channel_2);
  Serial.print(F(" - "));
  Serial.println(high_channel_2);
  Serial.print(F("Digital input 10 values: "));
  Serial.print(low_channel_3);
  Serial.print(F(" - "));
  Serial.print(center_channel_3);
  Serial.print(F(" - "));
  Serial.println(high_channel_3);
  Serial.print(F("Digital input 11 values: "));
  Serial.print(low_channel_4);
  Serial.print(F(" - "));
  Serial.print(center_channel_4);
  Serial.print(F(" - "));
  Serial.println(high_channel_4);
}

void record_center_position(){
  delay(2000);
  Serial.println(F("Place all sticks in the center position within 10 seconds."));
  for(int i = 9; i > 0; i--)  {
    delay(1000);
    Serial.print(i);
    Serial.print(" ");
  }
  Serial.println(" ");

  //Store the central stick positions
  center_channel_1 = receiver_input[1];
  center_channel_2 = receiver_input[2];
  center_channel_3 = receiver_input[3];
  center_channel_4 = receiver_input[4];

  Serial.println(F(""));
  Serial.println(F("Center positions stored."));
  Serial.print(F("Input A8 = "));
  Serial.println(receiver_input[1]);
  Serial.print(F("Input A9 = "));
  Serial.println(receiver_input[2]);
  Serial.print(F("Input A10 = "));
  Serial.println(receiver_input[3]);
  Serial.print(F("Input A11 = "));
  Serial.println(receiver_input[4]);
  Serial.println(F(""));
  Serial.println(F(""));
  
  delay(2000);
}

void wait_for_receiver(){
  Serial.println("Turn on your transmitter in the next 10 seconds.");
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15)  {
    if(receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;
    if(receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;
    if(receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;
    if(receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if(zero == 0)  {
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!"));
  }
  else Serial.println(F(" OK"));
}

ISR(PCINT2_vect) {
  current_time = micros();
  //============================================= Channel 1 =============================================
  if (PINK & B00000001) {                                      //Is input A8 high?
    if (last_channel_1 == 0) {                                 //Input A8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1) {                              //Input A8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input[1] = current_time - timer_1;                //Channel 1 is current_time - timer_1
  }

  //============================================= Channel 2 =============================================
  if (PINK & B00000010) {                                      //Is input A9 high?
    if (last_channel_2 == 0) {                                 //Input A9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1) {                              //Input A9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2
  }

  //============================================= Channel 3 =============================================
  if (PINK & B00000100) {                                      //Is input A10 high?
    if (last_channel_3 == 0) {                                 //Input A10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1) {                              //Input A10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3
  }

  //============================================= Channel 4 =============================================
  if (PINK & B00001000) {                                      //Is input A11 high?
    if (last_channel_4 == 0) {                                 //Input A11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1) {                              //Input A11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4
  }

  //============================================= Channel 5 =============================================
  if (PINK & B00010000) {                                      //Is input A12 high?
    if (last_channel_5 == 0) {                                 //Input A12 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_5 to current_time
    }
  }
  else if (last_channel_5 == 1) {                              //Input A12 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_input[5] = current_time - timer_5;                //Channel 5 is current_time - timer_5
  }

  //============================================= Channel 6 =============================================
  if (PINK & B00100000) {                                      //Is input A13 high?
    if (last_channel_6 == 0) {                                 //Input A13 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time;                                  //Set timer_6 to current_time
    }
  }
  else if (last_channel_6 == 1) {                              //Input A13 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_input[6] = current_time - timer_6;                //Channel 6 is current_time - timer_6
  }

  //============================================= Channel 7 =============================================
  if (PINK & B01000000) {                                      //Is input A14 high?
    if (last_channel_7 == 0) {                                 //Input A14 changed from 0 to 1
      last_channel_7 = 1;                                      //Remember current input state
      timer_7 = current_time;                                  //Set timer_7 to current_time
    }
  }
  else if (last_channel_7 == 1) {                              //Input A14 is not high and changed from 1 to 0
    last_channel_7 = 0;                                        //Remember current input state
    receiver_input[7] = current_time - timer_7;                //Channel 7 is current_time - timer_7
  }

  //============================================= Channel 8 =============================================
  if (PINK & B10000000) {                                      //Is input A15 high?
    if (last_channel_8 == 0) {                                 //Input A15 changed from 0 to 1
      last_channel_8 = 1;                                      //Remember current input state
      timer_8 = current_time;                                  //Set timer_8 to current_time
    }
  }
  else if (last_channel_8 == 1) {                              //Input A15 is not high and changed from 1 to 0
    last_channel_8 = 0;                                        //Remember current input state
    receiver_input[8] = current_time - timer_8;                //Channel 8 is current_time - timer_8
  }
}
