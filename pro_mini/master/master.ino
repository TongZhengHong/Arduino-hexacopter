#include <Wire.h>

//IMU variables
double gyro_cal[4];
double acc_cal_roll, acc_cal_pitch;

long acc_x, acc_y, acc_z, acc_total_vector;
float temperature;
float gyro_roll, gyro_pitch, gyro_yaw;

float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;

unsigned long difference, main_loop_timer;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

typedef union {
  int integer;
  byte one_byte;
  uint8_t two_bytes[2];
} converter;

converter number;
bool screen_free = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(9);
  TWBR = 12;
  Wire.onReceive(receive_event);
  Serial.println("START");

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 11)to trigger an interrupt on state change.

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("TEST");
  if (screen_free) {
    Wire.beginTransmission(8);
    number.integer = receiver_input_channel_1 / 10;
    Wire.write(number.one_byte);
    number.integer = receiver_input_channel_2 / 10;
    Wire.write(number.one_byte);
    number.integer = receiver_input_channel_3 / 10;
    Wire.write(number.one_byte);
    number.integer = receiver_input_channel_4 / 10;
    Wire.write(number.one_byte);
    Wire.endTransmission();
  }

  if (screen_free) {
    Wire.beginTransmission (8);
    Wire.write(true);
    Wire.endTransmission();
  }

  calculate_pitch_roll();

  if (screen_free) {
    Wire.beginTransmission (8);
    Wire.write(false);
    Wire.endTransmission();
  }

  maintain_loop_time();
}

void receive_event(int no_of_bytes) {
  while (Wire.available() < 1);
  screen_free = Wire.read();
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
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
    receiver_input_channel_1 = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
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
    receiver_input_channel_2 = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
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
    receiver_input_channel_3 = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
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
    receiver_input_channel_4 = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

void calculate_pitch_roll() {
  Wire.beginTransmission(104);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(104, 14);                                     //Request 14 bytes from the gyro.

  while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
  acc_y = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  acc_x = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
  gyro_pitch = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_roll = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_yaw = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

  gyro_pitch -= gyro_cal[1];
  gyro_roll -= gyro_cal[2];
  gyro_yaw -= gyro_cal[3];

  gyro_roll *= -1;
  //gyro_pitch *= -1;
  gyro_yaw *= -1;

  acc_x *= -1;
  //acc_y *= -1;
  acc_z *= -1;

  //Gyro calculations 0.000076336 = (0.005 / 65.5)
  angle_pitch += gyro_pitch * 0.000076336;
  angle_roll += gyro_roll * 0.000076336;

  //Accelerometer angle calculations
  angle_pitch_acc = (float) (atan2(acc_y, acc_z)) * 57.296;
  angle_roll_acc = (float) (atan2(acc_z, acc_x)) * 57.296;

  angle_roll_acc += (float) 90.0;
  if (angle_pitch_acc > 90) angle_pitch_acc -= (float) 180;
  else angle_pitch_acc += (float) 180;

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= acc_cal_pitch; //2.7;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= acc_cal_roll; //1.8;                                                    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  //  Serial.print(angle_pitch);
  //  Serial.print(" ");
  //  Serial.println(angle_roll);
}

