#include <Wire.h>
#include <EEPROM.h>

//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>

#define QUADCOPTER
//#define HEXCOPTER

#define GYRO_ADDR 0x6B
#define ACC_ADDR 0x1D

//RF24 radio(9, 10); // CE, CSN
//const byte address[6] = "00001";

//* /////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.4;  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.00;  //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.3;    //G ain setting for the roll D-controller
uint16_t pid_max_roll = 350;       //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_roll = 100;     //Eliminate I controller windup

#ifdef QUADCOPTER
float pid_p_gain_pitch = pid_p_gain_roll; //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; //Gain setting for the pitch D-controller.
uint16_t pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_pitch = pid_max_i_roll;     //Eliminate I controller windup
#endif

#ifdef HEXCOPTER
float pid_p_gain_pitch = 0.0; //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.0; //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0.0; //Gain setting for the pitch D-controller.
uint16_t pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_pitch = pid_max_i_roll;     //Eliminate I controller windup
#endif

float pid_p_gain_yaw = 1.0;  //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0; //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;  //Gain setting for the pitch D-controller.
uint16_t pid_max_yaw = 400;       //Maximum output of the PID-controller (+/-)
//* /////////////////////////////////////////////////////////////////////////////////////////////

//Misc. variables
uint8_t start;
float voltage;
uint16_t battery_voltage;
uint32_t difference, main_loop_timer;
byte eeprom_data[75];

//Transmitter variables
uint16_t receiver_input_channel_1 = 0, receiver_input_channel_2 = 0,
         receiver_input_channel_3 = 0, receiver_input_channel_4 = 0,
         receiver_input_channel_5 = 0, receiver_input_channel_6 = 0;

//IMU variables
float gyro_cal[4], accel_cal[6], mag_cal[6];
float acc_cal_roll, acc_cal_pitch;

int16_t acc_x_raw, acc_y_raw, acc_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

int16_t acc_x_mem[16], acc_y_mem[16], acc_z_mem[16];
int16_t gyro_x_mem[8], gyro_y_mem[8], gyro_z_mem[8];
int32_t acc_x_sum, acc_y_sum, acc_z_sum, gyro_x_sum, gyro_y_sum, gyro_z_sum;
uint8_t gyro_loop_counter = 0, acc_loop_counter = 0;

int32_t acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

int16_t roll, pitch;
float angle_roll, angle_pitch, angle_yaw;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;

//Compass variables
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
float mag_x, mag_y, mag_z;
float compass_x, compass_y, compass_heading;

float mag_x_offset, mag_y_offset, mag_z_offset;
float mag_x_scale, mag_y_scale, mag_z_scale;

float prev_heading, heading;
bool heading_hold = false;

//PID variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

typedef union {
  float decimal;
  uint8_t bytes[4];
} converter;

converter number;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(2000);

  for (start = 0; start <= 74; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[72] != 'J' || eeprom_data[73] != 'M' || eeprom_data[74] != 'B')
    delay(10);

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 24];
    number.bytes[1] = eeprom_data[i * 4 + 25];
    number.bytes[2] = eeprom_data[i * 4 + 26];
    number.bytes[3] = eeprom_data[i * 4 + 27];
    accel_cal[i] = number.decimal;
  }

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 48];
    number.bytes[1] = eeprom_data[i * 4 + 49];
    number.bytes[2] = eeprom_data[i * 4 + 50];
    number.bytes[3] = eeprom_data[i * 4 + 51];
    mag_cal[i] = number.decimal;
  }

  attachInterrupt(digitalPinToInterrupt(14), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), receiver_change, CHANGE);

  PORTD_PCR2 = (1 << 8); //configuring pin 7 as GPIO
  PORTD_PCR3 = (1 << 8); //configuring pin 8 as GPIO
  PORTD_PCR4 = (1 << 8); //configuring pin 6 as GPIO
  PORTD_PCR7 = (1 << 8); //configuring pin 5 as GPIO
  PORTD_PCR5 = (1 << 8); //configuring pin 20 as GPIO
  PORTD_PCR6 = (1 << 8); //configuring pin 21 as GPIO
  GPIOD_PDDR |= 252; //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as outputs

  PORTC_PCR5 = (1 << 8);  //configuring LED pin as GPIO
  GPIOC_PDDR = (1 << 5);  //configuring LED pin as an output
  //GPIOC_PSOR = (1 << 5);  //setting LED pin high

  Serial.println("Welcome to flight controller setup!");
  Serial.println("Turn on your transmitter and place throttle at lowest position!");
//  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
//    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
//    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
//    start++;                                                //While waiting increment start whith every loop.
//
//    //    Serial.println(receiver_input_channel_3);
//    //    Serial.println(receiver_input_channel_4);
//    //    Serial.println();
//
//    pulse_esc();
//    if (start == 125) {
//      digitalWrite(13, !digitalRead(13));                   //Change the led status.
//      start = 0;                                            //Start again at 0.
//    }
//  }
//  start = 0;
  Serial.println("Transmitter detected!");

  delay(500);

  digitalWrite(13, HIGH);
  setup_sensor();
  calibrate_sensors();
  digitalWrite(13, LOW);

  Serial.print("Connect your battery in: ");
  for (int i = 5; i > 0; i--) {
    Serial.print((String) i + " ");
    delay(900);
    pulse_esc();
  }
  Serial.println();
  calculate_battery();
  Serial.println("Battery left: " + (String) battery_voltage);
  Serial.println("Setup DONE!");

  delay(1000);

  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);

//  radio.begin();
//  radio.openWritingPipe(address);
//  radio.setPALevel(RF24_PA_MIN);
//  radio.stopListening();
}

void loop() {
  convert_transmitter_values();

  check_start_stop();

  calculate_pitch_roll();

//  calculate_heading();

  set_pid_offsets();

  calculate_pid();

  calculate_esc_output();

  set_escs();

  calculate_battery();
//  radio.write(&roll, sizeof(roll));

  maintain_loop_time();
}

void check_start_stop() {
  //Serial.println("Start value: " + (String) start);
  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050 && receiver_input_channel_1 > 1950 && receiver_input_channel_2 < 1050)  {
    if (start == 0) {
      start = 1;
    }
    else if (start == 2) { //Stop motors --> check if already started
      start = 3;
    }
  }

  //When yaw stick is back in the center position start the motors (step 2).
  if (receiver_input_channel_4 > 1450 && receiver_input_channel_1 < 1550 && receiver_input_channel_2 > 1450)  {
    if (start == 1)  {
      Serial.println("START MOTORS");
      start = 2; //start motors

      angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
      angle_roll = angle_roll_acc;   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
      prev_heading = heading;

      //Reset the PID controllers
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    }
    else if (start == 3)    {
      Serial.println("STOP MOTORS");
      start = 0; //Stop motors
    }
  }
}

void calculate_battery() {
  float diodeForward = 0.4;
  float reading_error = -0.34;
  float potDivider = 5.0244; // 1 / (82/(82+330))

  int sensorValue = analogRead(A14);
  float analog_voltage = sensorValue * (3.3 / 1024);
  voltage = voltage * 0.95 + analog_voltage * 0.05;
  battery_voltage = 100 * ((voltage - reading_error) * potDivider + diodeForward);
}

void pulse_esc() {
  GPIOD_PSOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as HIGH
  delayMicroseconds(1000);
  GPIOD_PCOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as LOW
  delay(3);
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}
