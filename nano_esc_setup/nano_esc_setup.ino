#include <Wire.h>
#include <EEPROM.h>

#define imu_address 104

int start;
byte eeprom_data[50], command, loop_counter;
unsigned long startTime, difference, main_loop_timer;

//IMU variables
double gyro_cal[4], accel_cal[6];
double acc_cal_roll, acc_cal_pitch;

long acc_x, acc_y, acc_z;

//Transmitter variables
volatile int receiver_input[6];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, current_time;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

//ESC variables
unsigned long zero_timer, esc_loop_timer;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
int esc_1, esc_2, esc_3, esc_4;

//Vibration
float acc_total_vector[20], acc_av_vector;
unsigned long vibration_counter, vibration_total_result;

typedef union {
  double decimal;
  uint8_t bytes[4];
} converter;

converter number;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 11)to trigger an interrupt on state change.

  DDRD |= B11110000;

  for (start = 0; start <= 50; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[48] != 'J' || eeprom_data[49] != 'M' || eeprom_data[50] != 'B')
    delay(10);

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 24];
    number.bytes[1] = eeprom_data[i * 4 + 25];
    number.bytes[2] = eeprom_data[i * 4 + 26];
    number.bytes[3] = eeprom_data[i * 4 + 27];
    accel_cal[i] = number.decimal;
  }

  setupSensor();

  Serial.println("Enter a command: ");
}

void loop() {
  receiver_input_channel_3 = convert_receiver_channel(3);

  if (Serial.available() > 0) {
    command = Serial.read();
    delay(100);
    while (Serial.available() > 0) loop_counter = Serial.read();                         //Empty the Serial buffer.

    if (command == 'r') Serial.println("Calibrate ESC range.");
    else if (command == '1') Serial.println("Test motor 1 (Right Front CCW.)");
    else if (command == '2') Serial.println("Test motor 2 (Right Rear CW.)");
    else if (command == '3') Serial.println("Test motor 3 (Left Rear CCW.)");
    else if (command == '4') Serial.println("Test motor 4 (Left Front CW.)");
    else if (command == '5') Serial.println("Test all motors together");
    else if (command == '0') Serial.println("Nothing entered");
    else Serial.println("No such command");
  }

  if (command == 49 || command == 50 || command == 51 || command == 52 || command == 53) {
    if (command == 49 || command == 53) esc_1 = receiver_input_channel_3;                  //If motor 1 is requested set the pulse for motor 1 equal to the throttle channel.
    else esc_1 = 1000;                                                                //If motor 1 is not requested set the pulse for the ESC to 1000us (off).
    if (command == 50 || command == 53) esc_2 = receiver_input_channel_3;                  //If motor 2 is requested set the pulse for motor 1 equal to the throttle channel.
    else esc_2 = 1000;                                                                //If motor 2 is not requested set the pulse for the ESC to 1000us (off).
    if (command == 51 || command == 53) esc_3 = receiver_input_channel_3;                  //If motor 3 is requested set the pulse for motor 1 equal to the throttle channel.
    else esc_3 = 1000;                                                                //If motor 3 is not requested set the pulse for the ESC to 1000us (off).
    if (command == 52 || command == 53) esc_4 = receiver_input_channel_3;                  //If motor 4 is requested set the pulse for motor 1 equal to the throttle channel.
    else esc_4 = 1000;                                                                //If motor 4 is not requested set the pulse for the ESC to 1000us (off).

    set_esc();

    Wire.beginTransmission(imu_address);                                           //Start communication with the gyro.
    Wire.write(0x3B);                                                               //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                         //End the transmission.
    Wire.requestFrom(imu_address, 6);                                              //Request 6 bytes from the gyro.
    while (Wire.available() < 6);                                                   //Wait until the 6 bytes are received.
    acc_x = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_x variable.
    acc_y = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_y variable.
    acc_z = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_z variable.

    acc_total_vector[0] = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

    acc_av_vector = acc_total_vector[0];                                            //Copy the total vector to the accelerometer average vector variable.

    for (int i = 16; i > 0; i--) {                                          //Do this loop 16 times to create an array of accelrometer vectors.
      acc_total_vector[i] = acc_total_vector[i - 1];                        //Shift every variable one position up in the array.
      acc_av_vector += acc_total_vector[i];                                     //Add the array value to the acc_av_vector variable.
    }

    acc_av_vector /= 16;                                                            //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

    if (vibration_counter < 20) {                                                   //If the vibration_counter is less than 20 do this.
      vibration_counter++;                                                         //Increment the vibration_counter variable.
      vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);           //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
    } else {
      vibration_counter = 0;                                                        //If the vibration_counter is equal or larger than 20 do this.
      Serial.println(vibration_total_result / 50);
      vibration_total_result = 0;                                                   //Reset the vibration_total_result variable.
    }
  }

  if (command == 114) {
    Serial.println(command);
    Serial.print("Starting ESC calibration in: ");
    for (int i = 5; i > 0; i--) {
      Serial.print(i);
      Serial.print(" ");
      delay(1000);
    }

    Serial.println();
    Serial.println("Sending 2000us pulse");
    Serial.println("Connect your battery.");
    startTime = millis();
    difference = 0;
    while (difference < 7000) {
      difference = millis() - startTime;
      PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
      delayMicroseconds(2000);
      PORTD &= B00001111;
      delay(3);
    }

    Serial.println("Sending 1000us pulse");
    startTime = millis();
    difference = 0;
    while (difference < 10000) {
      difference = millis() - startTime;
      PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
      delayMicroseconds(1000);
      PORTD &= B00001111;
      delay(3);
    }

    Serial.println("ESC calibration completed!");

    unsigned long start = millis();
    unsigned long difference = 0;
    while (difference < 30000) {
      difference = millis() - start;
      
      receiver_input_channel_3 = convert_receiver_channel(3);
      PORTD |= B11110000; //Set digital pin 4 - 7 as HIGH
      delayMicroseconds(receiver_input_channel_3);
      PORTD &= B00001111;
      delay(3);
    }
    Serial.println("Testing over!");
    command = 48;
  }
}

void set_esc() {
  zero_timer = micros();
  PORTD |= B11110000;                                            //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = esc_1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

  while (PORTD >= 16) {                                          //Execute the loop until digital port 4 to 7 is low.
    esc_loop_timer = micros();                                   //Check the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B01111111;    //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B10111111;    //When the delay time is expired, digital port 5 is set low.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B11011111;    //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B11101111;    //When the delay time is expired, digital port 7 is set low.
  }
}

ISR(PCINT0_vect) {
  current_time = micros();
  //* ========================================= Channel 1 =========================================
  if (PINB & B00000001) {                                                   //Is input 8 high?
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
  if (PINB & B00000010) {                                                  //Is input 9 high?
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
  if (PINB & B00001000) {                                                  //Is input 11 high?
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
  if (PINB & B00010000) {                                                  //Is input 12 high?
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

