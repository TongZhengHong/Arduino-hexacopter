#include <Wire.h>
#include <EEPROM.h>

#define RATES

//* /////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.5;  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;  //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.2;    //Gain setting for the roll D-controller
int pid_max_roll = 350;       //Maximum output of the PID-controller (+/-)
int pid_max_i_roll = 200;     //Eliminate I controller windup

float pid_p_gain_pitch = pid_p_gain_roll; //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)
int pid_max_i_pitch = pid_max_i_roll;     //Eliminate I controller windup

float pid_p_gain_yaw = 0.0;  //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0; //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;  //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;       //Maximum output of the PID-controller (+/-)
//* /////////////////////////////////////////////////////////////////////////////////////////////

//Misc. variables
int start;
int difference, main_loop_timer;
byte eeprom_data[27];

//Transmitter variables
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

//IMU variables
double gyro_cal[4];

long acc_x, acc_y, acc_z, acc_total_vector;
float temperature;
double gyro_roll, gyro_pitch, gyro_yaw;

float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;

//PID variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

void setup () {
  Wire.begin();
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for (start = 0; start <= 26; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[24] != 'J' || eeprom_data[25] != 'M' || eeprom_data[26] != 'B')
    delay(10);

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 11)to trigger an interrupt on state change.

  DDRD |= B11110000;

//  Serial.println("Turn on your transmitter and place throttle at lowest position!");
//  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
//    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
//    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
//    start++;                                                //While waiting increment start whith every loop.
//
////    Serial.println(receiver_input_channel_3);
////    Serial.println(receiver_input_channel_4);
////    Serial.println();
//
//    pulse_esc();
//    if (start == 125) {
//      digitalWrite(13, !digitalRead(13));                   //Change the led status.
//      start = 0;                                            //Start again at 0.
//    }
//  }
  start = 0;
  Serial.println("Transmitter detected!");

  setupSensor();
  calibrateSensors();
  digitalWrite(13, LOW);

  int battery = calculate_battery();
  Serial.println("Battery left: " + (String) battery);
  Serial.println("Setup DONE!");
  for (int i = 3; i > 0; i--) {
    Serial.print((String) i + " ");
    delay(1000);
    pulse_esc();
  }
}

void loop () {
  convert_transmitter_values();
  
  calculate_pitch_roll();

  check_start_stop();

  set_pid_offsets();

  calculate_pid();

  calculate_esc_output();

  set_escs();

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
    if (start == 1)    {
      Serial.println("START MOTORS");
      start = 2; //start motors

      angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
      angle_roll = angle_roll_acc;   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

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

int calculate_battery() {
  float diodeForward = 0.5;
  int potDivider = 167;

  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023);
  int battery_voltage = (voltage + diodeForward) * potDivider;

  return battery_voltage;
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}

void pulse_esc() {
  PORTD |= B11110000;                                     //Sends 1000 pulse to ESCs to prevent them from beeping
  delayMicroseconds(1000);
  PORTD &= B00001111;
  delay(3);                                               //Wait 3 milliseconds before the next loop.
}
