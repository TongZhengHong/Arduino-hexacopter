#include <Wire.h>
#include <EEPROM.h>

//* /////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.8;  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;  //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.2;    //Gain setting for the roll D-controller
int pid_max_roll = 350;       //Maximum output of the PID-controller (+/-)
int pid_max_i_roll = 200;     //Eliminate I controller windup

float pid_p_gain_pitch = pid_p_gain_roll; //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)
int pid_max_i_pitch = pid_max_i_roll;     //Eliminate I controller windup

float pid_p_gain_yaw = 0.5;  //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0; //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;  //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;       //Maximum output of the PID-controller (+/-)
//* /////////////////////////////////////////////////////////////////////////////////////////////

//Misc. variables
int start;
int difference, main_loop_timer;
byte eeprom_data[27];
byte page_number = 0;
bool screen_free = false;

//Transmitter variables
int receiver_input_channel_1 = 0, receiver_input_channel_2 = 0,
    receiver_input_channel_3 = 0, receiver_input_channel_4 = 0;

//IMU variables
double gyro_cal[4];
double acc_cal_roll, acc_cal_pitch;

long acc_x, acc_y, acc_z, acc_total_vector;
float temperature;
float gyro_roll, gyro_pitch, gyro_yaw;

float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;

//PID variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

typedef union {
  int integer;
  byte one_byte;
  uint8_t two_bytes[2];
} converter;

void setup () {
  Serial.begin(115200);
  Wire.begin(9);
  TWBR = 12;
  Wire.onReceive(receive_event);

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

  for (int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println("start");
  Wire.beginTransmission(8);
  byte number = 1;
  Wire.write(number);
  Wire.endTransmission();

  Serial.println("Turn on your transmitter and place throttle at lowest position!");
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start++;                                                //While waiting increment start whith every loop.

    //    Serial.println(receiver_input_channel_3);
    //    Serial.println(receiver_input_channel_4);
    //    Serial.println();

    pulse_esc();
    if (start == 125) {
      digitalWrite(13, !digitalRead(13));                   //Change the led status.
      start = 0;                                            //Start again at 0.
    }
  }
  start = 0;
  Serial.println("Transmitter detected!");

  Wire.beginTransmission (8);
  Wire.write(2);
  Wire.endTransmission();
  
  setupSensor();
  calibrateSensors();
  digitalWrite(13, LOW);

  int battery = calculate_battery();
  Serial.println("Battery left: " + (String) battery);
  Serial.println("Setup DONE!");
  
  Wire.beginTransmission (8);
  Wire.write(3);
  Wire.endTransmission();

  for (int i = 3; i > 0; i--) {
    Serial.print((String) i + " ");
    delay(1000);
    pulse_esc();
  }
  Wire.beginTransmission (8);
  Wire.write(4);
  Wire.endTransmission();
}

void loop () {
  convert_transmitter_values();

  calculate_pitch_roll();

  check_start_stop();

  set_pid_offsets();

  calculate_pid();

  calculate_esc_output();

  set_escs();

  display_data();

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

void display_data() {
  if (!screen_free) return;

  Wire.requestFrom(8, 1);   //Request ONE byte of data from pro mini of current page
  while (Wire.available()) page_number = Wire.read();
  
  converter number;
  switch (page_number) {
    case 2:   //Total bytes transfered: 4
      number.integer = angle_roll * 100;
      Wire.beginTransmission(8);
      Wire.write(number.two_bytes[0]);
      Wire.write(number.two_bytes[1]);
      number.integer = angle_pitch * 100;
      Wire.write(number.two_bytes[0]);
      Wire.write(number.two_bytes[1]);
      Wire.endTransmission();
      break;
    case 1:   //Total bytes transfered: 4
      int data[4];
      data[0] = receiver_input_channel_1 / 10;
      data[1] = receiver_input_channel_2 / 10;
      data[2] = receiver_input_channel_3 / 10;
      data[3] = receiver_input_channel_4 / 10;

      Wire.beginTransmission(8);
      for (int i = 0; i < 4; i++) {
        number.integer = data[i];
        Wire.write(number.one_byte);
      }
      Wire.endTransmission();
      break;
    case 3:   //Total bytes transfered: 2
      number.integer = calculate_battery();
      Wire.beginTransmission(8); // transmit to device #8
      Wire.write(number.two_bytes[0]);
      Wire.write(number.two_bytes[1]);
      Wire.endTransmission();
      break;
  }
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

void receive_event(int bytes) {
  while (Wire.available() < 1); 
  screen_free = Wire.read();
}
