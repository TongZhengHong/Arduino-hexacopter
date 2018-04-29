#include <EEPROM.h>

byte eeprom_data[27];
int start;

//Transmitter variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
volatile int receiver_input[5];

void setup(){
    Serial.begin(9600);
    delay(500);

    for (start = 0; start <= 26; start++)
        eeprom_data[start] = EEPROM.read(start);
    while (eeprom_data[24] != 'J' || eeprom_data[25] != 'M' || eeprom_data[26] != 'B')
        delay(10);

    PCICR |= (1 << PCIE2); // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port K.

    PCMSK2 |= (1 << PCINT20); // set PCINT20 (digital input A12) to trigger an interrupt on state change
    PCMSK2 |= (1 << PCINT21); // set PCINT21 (digital input A13)to trigger an interrupt on state change
    PCMSK2 |= (1 << PCINT22); // set PCINT22 (digital input A14)to trigger an interrupt on state change
    PCMSK2 |= (1 << PCINT23); // set PCINT23 (digital input A15)to trigger an interrupt on state change

    DDRA |= B00111111; // Set digital pins 22 - 27 to OUTPUT

    Serial.println("SETUP DONE!");
    delay(3000);

    Serial.println("Waiting for receiver...");
    Serial.println("Please turn on your transmitter and ensure that the throttle is in the LOWEST position.");
    //Wait until the receiver is active and the throtle is set to the lower position.
    while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
        receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
        receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
        start++;                                                //While waiting increment start whith every loop.

        delay(3); //Wait 3 milliseconds before the next loop.
        if (start == 125)    {
        digitalWrite(13, !digitalRead(13)); //Change the led status.
        start = 0;                          //Start again at 0.
        }
    }
    start = 0; //Set start back to 0.
}

void loop(){
    receiver_input_channel_1 = convert_receiver_channel(1); //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2); //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us

    Serial.println(receiver_input_channel_3);
    PORTA |= B00111111;
    delayMicroseconds(receiver_input_channel_3);
    PORTA &= B11000000;
    delayMicroseconds(100);
}

int convert_receiver_channel(byte function){
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

ISR(PCINT2_vect){
  current_time = micros();
  //*============================================= Channel 1 =============================================
  if (PINK & B10000000)  { //Is input A15 high?
    if (last_channel_1 == 0)    {                         //Input A8 changed from 0 to 1
      last_channel_1 = 1;     //Remember current input state
      timer_1 = current_time; //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1)  {              //Input A8 is not high and changed from 1 to 0
    last_channel_1 = 0;                         //Remember current input state
    receiver_input[1] = current_time - timer_1; //Channel 1 is current_time - timer_1
  }

  //*============================================= Channel 2 =============================================
  if (PINK & B01000000)  { //Is input A14 high?
    if (last_channel_2 == 0)    {                         //Input A9 changed from 0 to 1
      last_channel_2 = 1;     //Remember current input state
      timer_2 = current_time; //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1)  {              //Input A9 is not high and changed from 1 to 0
    last_channel_2 = 0;                         //Remember current input state
    receiver_input[2] = current_time - timer_2; //Channel 2 is current_time - timer_2
  }

  //*============================================= Channel 3 =============================================
  if (PINK & B00100000)  { //Is input A13 high?
    if (last_channel_3 == 0)    {                         //Input A10 changed from 0 to 1
      last_channel_3 = 1;     //Remember current input state
      timer_3 = current_time; //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1)  {              //Input A10 is not high and changed from 1 to 0
    last_channel_3 = 0;                         //Remember current input state
    receiver_input[3] = current_time - timer_3; //Channel 3 is current_time - timer_3
  }

  //*============================================= Channel 4 =============================================
  if (PINK & B00010000)  { //Is input A12 high?
    if (last_channel_4 == 0)    {                         //Input A11 changed from 0 to 1
      last_channel_4 = 1;     //Remember current input state
      timer_4 = current_time; //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1)  {              //Input A11 is not high and changed from 1 to 0
    last_channel_4 = 0;                         //Remember current input state
    receiver_input[4] = current_time - timer_4; //Channel 4 is current_time - timer_4
  }
}