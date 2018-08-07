//* /////////////////////////////////////////////////////////////////////////////////////////////
//*Convention:
//*  Channel 1: ROLL
//*  Channel 2: PITCH
//*  Channel 3: THROTTLE
//*  Channel 4: YAW
//* /////////////////////////////////////////////////////////////////////////////////////////////

//TODO: Ensure stable angles during flight
//TODO: Check PID outputs --> Test flight for I controller (Make sure max output is correct)
//TODO: Battery voltage integration

#include <Wire.h>
#include <EEPROM.h>

int imu_address = 104;

//* /////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.2;  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;  //Gain setting for the roll I-controller
float pid_d_gain_roll = 0;    //Gain setting for the roll D-controller
int pid_max_roll = 400;       //Maximum output of the PID-controller (+/-)
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
byte eeprom_data[27];
float battery_voltage;
int start, difference, main_loop_timer;
bool angle_first_start = true;

//Sensor variables
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
long acc_x, acc_y, acc_z, acc_total_vector;
float roll_level_adjust, pitch_level_adjust;
float temperature;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_cal[4];

//Transmitter variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6, last_channel_7, last_channel_8;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, timer_7, timer_8, current_time;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
volatile int receiver_input[9];

//ESC variables
int esc_1, esc_2, esc_3, esc_4, esc_5, esc_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6;
unsigned long esc_timer, esc_loop_timer, loop_timer;
int throttle;

//PID variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(500);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for (start = 0; start <= 26; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[24] != 'J' || eeprom_data[25] != 'M' || eeprom_data[26] != 'B')
    delay(10);

  Serial.println("Initialising pins...");

  PCICR |= (1 << PCIE2); // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port K.

  PCMSK2 |= (1 << PCINT16); // set PCINT16 (digital input A8) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT17); // set PCINT17 (digital input A9)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT18); // set PCINT18 (digital input A10)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19); // set PCINT19 (digital input A11)to trigger an interrupt on state change

  PCMSK2 |= (1 << PCINT20); // set PCINT20 (digital input A12) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT21); // set PCINT21 (digital input A13)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT22); // set PCINT22 (digital input A14)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT23); // set PCINT23 (digital input A15)to trigger an interrupt on state change

  DDRA |= B00111111; // Set digital pins 22 - 27 to OUTPUT

  Serial.println("CALLIBRATING sensors...");

  setupSensor();
  calibrateSensors();

  Serial.println("Waiting for receiver...");
  Serial.println("Please turn on your transmitter and ensure that the throttle is in the LOWEST position.");
  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start++;                                                //While waiting increment start whith every loop.

    pulse_esc();
    if (start == 125) {
      digitalWrite(13, !digitalRead(13));                   //Change the led status.
      start = 0;                                            //Start again at 0.
    }
  }
  start = 0;                                                //Set start back to 0.

  //Battery voltage calculation
  battery_voltage = analogRead(A0) * (5000 / 1023.0) + 20;
  Serial.println("Current battery reading: " + (String) battery_voltage);

  digitalWrite(13, LOW);
  Serial.println("SETUP DONE!");

  for (int i = 5; i > 0; i--) {
    Serial.print((String) i + " ");
    pulse_esc();
    delay(1000);
  }
}

void pulse_esc() {
  PORTA |= B00111111;                                     //Sends 1000 pulse to ESCs to prevent them from beeping
  delayMicroseconds(1000);
  PORTA &= B11000000;
  delay(3);                                               //Wait 3 milliseconds before the next loop.
}

//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//
//* //////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////// *//
//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//

void loop() {
  convert_transmitter_values();

  calculate_pitch_roll();

  check_start_stop();

  set_pid_offsets();

  calculate_pid();

  calculate_esc_output();

  set_escs();

  check_battery_voltage();

  difference = micros() - main_loop_timer;
  while (difference < 4000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}

//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//
//* //////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////// *//
//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//

void calculate_pitch_roll() {
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);       //Gyro pid input is deg/sec.

  Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 3Bh and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(imu_address, 14);                                     //Request 14 bytes from the gyro.

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

  //Gyro calculations 0.000061069 = 1 / (0.004 / 65.5)
  angle_roll += gyro_pitch * -0.000061069;
  angle_pitch += gyro_roll * -0.000061069;

  //Convert 0.000061069 into radians = 0.0000010658
  angle_roll += angle_pitch * sin(gyro_yaw * 0.0000010658);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.0000010658);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer calculations
  float acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

  //180 / PI = 57.296 --> Convert into degrees
  if (abs(acc_y) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float) acc_y / acc_total_vector) * -57.296;       //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float) acc_x / acc_total_vector) * 57.296;       //Calculate the roll angle.
  }

  angle_pitch_acc += 1.1; //Accelerometer calibration value for pitch.
  angle_roll_acc += 1.6;   //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.99 + angle_roll_acc * 0.01;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  Serial.print("PITCH: ");
    Serial.println(angle_pitch);
    Serial.print("ROLL: ");
    Serial.println(angle_roll);
    Serial.println("");
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

void set_pid_offsets() {
  //* ///////////////////////////////////// Roll setpoints /////////////////////////////////////////////////
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1504)
      pid_roll_setpoint = receiver_input_channel_1 - 1504;
    else if (receiver_input_channel_1 < 1496)
      pid_roll_setpoint = receiver_input_channel_1 - 1496;

    pid_roll_setpoint = map(pid_roll_setpoint, 0, 496, 0, 50);    //! Output: 0 to 40 degrees of roll
  }

  //* ///////////////////////////////////// Pitch setpoints /////////////////////////////////////////////////
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1504)
      pid_pitch_setpoint = receiver_input_channel_2 - 1504;
    else if (receiver_input_channel_2 < 1496)
      pid_pitch_setpoint = receiver_input_channel_2 - 1496;

    pid_pitch_setpoint = map(pid_pitch_setpoint, 0, 496, 0, 50);  //! Output: 0 to 40 degrees of pitch
  }

  //* ///////////////////////////////////// Yaw setpoints /////////////////////////////////////////////////
  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ((500-8)/3 = 164d/s).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  /*Serial.print(pid_roll_setpoint);
    Serial.print(", ");
    Serial.print(pid_pitch_setpoint);
    Serial.print(", ");
    Serial.print(pid_yaw_setpoint);
    Serial.println("");*/
}

void calculate_pid() {
  //* //////////////////////////////////////// Roll calculation //////////////////////////////////////////
  pid_error_temp = angle_roll - pid_roll_setpoint;        //? Calculate error //angle_roll - pid_roll_setpoint
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  if (pid_i_mem_roll > pid_max_i_roll)
    pid_i_mem_roll = pid_max_i_roll;
  else if (pid_i_mem_roll < pid_max_i_roll * -1)
    pid_i_mem_roll = pid_max_i_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //* //////////////////////////////////////// Pitch calculation //////////////////////////////////////////
  pid_error_temp = angle_pitch - pid_pitch_setpoint;      //? Calculate error //angle_pitch - pid_pitch_setpoint
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  if (pid_i_mem_pitch > pid_max_i_pitch)
    pid_i_mem_pitch = pid_max_i_pitch;
  else if (pid_i_mem_pitch < pid_max_i_pitch * -1)
    pid_i_mem_pitch = pid_max_i_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //* //////////////////////////////////////// Yaw calculation ///////////////////////////////////////////
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)
    pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)
    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)
    pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
  /*
    Serial.print(pid_output_roll);
    Serial.print(", ");
    Serial.print(pid_output_pitch);
    Serial.print(", ");
    Serial.print(pid_output_yaw);
    Serial.println("\n");*/
}

void calculate_esc_output() {
  throttle = receiver_input_channel_3; //We need the throttle signal as a base signal.

  if (start == 2)  { //The motors are started.
    if (throttle > 1850) throttle = 1850; //We need some room to keep full control at full throttle.
    esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (CW)
    esc_2 = throttle /*==============*/ + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (CCW)
    esc_3 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (CW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (CCW)
    esc_5 = throttle /*==============*/ - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 5 (CW)
    esc_6 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 6 (CCW)

    /*if (battery_voltage < 830 && battery_voltage > 600)    {                                                           //Is the battery connected?
      esc_1 += esc_1 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-4 pulse for voltage drop.
      esc_5 += esc_5 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-5 pulse for voltage drop.
      esc_6 += esc_6 * ((830 - battery_voltage) / (float)3500); //Compensate the esc-6 pulse for voltage drop.
      }*/

    if (esc_1 < 1100)      esc_1 = 1100; //Keep the motors running.
    if (esc_2 < 1100)      esc_2 = 1100; //Keep the motors running.
    if (esc_3 < 1100)      esc_3 = 1100; //Keep the motors running.
    if (esc_4 < 1100)      esc_4 = 1100; //Keep the motors running.
    if (esc_5 < 1100)      esc_5 = 1100; //Keep the motors running.
    if (esc_6 < 1100)      esc_6 = 1100; //Keep the motors running.

    if (esc_1 > 2000)      esc_1 = 2000; //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)      esc_2 = 2000; //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)      esc_3 = 2000; //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)      esc_4 = 2000; //Limit the esc-4 pulse to 2000us.
    if (esc_5 > 2000)      esc_5 = 2000; //Limit the esc-5 pulse to 2000us.
    if (esc_6 > 2000)      esc_6 = 2000; //Limit the esc-6 pulse to 2000us.
  }  else  {
    esc_1 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-1.
    esc_2 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-2.
    esc_3 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-3.
    esc_4 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-4.
    esc_5 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-5.
    esc_6 = 1000;                        //If start is not 2 keep a 1000us pulse for esc-6.
  }

  /*Serial.print(esc_1);
    Serial.print(", ");
    Serial.print(esc_2);
    Serial.print(", ");
    Serial.print(esc_3);
    Serial.print(", ");
    Serial.print(esc_4);
    Serial.print(", ");
    Serial.print(esc_5);
    Serial.print(", ");
    Serial.print(esc_6);
    Serial.println("");*/
}

void set_escs() {
  //TODO: Check time taken for loop cycle!
  loop_timer = micros();
  PORTA |= B00111111;                   //Set digital pin 22 - 27 as HIGH
  timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the faling edge of the esc-4 pulse.
  timer_channel_5 = esc_5 + loop_timer; //Calculate the time of the faling edge of the esc-5 pulse.
  timer_channel_6 = esc_6 + loop_timer; //Calculate the time of the faling edge of the esc-6 pulse.

  while (PORTA >= 1)  {                 //Stay in this loop until output 22 - 27 are LOW.
    esc_loop_timer = micros();          //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTA &= B11111110; //Set digital output 22 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer) PORTA &= B11111101; //Set digital output 23 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer) PORTA &= B11111011; //Set digital output 24 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer) PORTA &= B11110111; //Set digital output 25 to low if the time is expired.
    if (timer_channel_5 <= esc_loop_timer) PORTA &= B11101111; //Set digital output 26 to low if the time is expired.
    if (timer_channel_6 <= esc_loop_timer) PORTA &= B11011111; //Set digital output 27 to low if the time is expired.
  }
}

void check_battery_voltage() {
  float constant = (5000 / 1023.0) * 0.08;
  battery_voltage = battery_voltage * 0.92 + analogRead(A0) * constant + 2;

  if (battery_voltage < 680 && battery_voltage > 600) {
    //digitalWrite(13, HIGH);
  }

  //Serial.println("Battery left: " + (String) battery_voltage);
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

void convert_transmitter_values() {
  receiver_input_channel_1 = convert_receiver_channel(1); //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2); //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us

  /*for (int i = 1; i < 9; i++) {
      Serial.print(receiver_input[i]);
      Serial.print(", ");
    }
    Serial.println("");*/
  
    /*Serial.print(receiver_input_channel_1);
    Serial.print(", ");
    Serial.print(receiver_input_channel_2);
    Serial.print(", ");
    Serial.print(receiver_input_channel_3);
    Serial.print(", ");
    Serial.print(receiver_input_channel_4);
    Serial.println("");*/
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
    while (1)delay(10);                                                      //Stay in this loop for ever
  }

  Wire.beginTransmission(imu_address);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro
}

void calibrateSensors() {
  //* //////////////////////////////////////// Gyroscope calibration //////////////////////////////////////////
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(imu_address);                                   //Start communication with the gyro.
    Wire.write(0x43);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(imu_address, 6);                                      //Request 14 bytes from the gyro.

    while (Wire.available() < 6);                                          //Wait until the 6 bytes are received.
    gyro_cal[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_cal[3] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.

    PORTA |= B00111111;                                     //Sends 1000 pulse to ESCs to prevent them from beeping
    delayMicroseconds(1000);
    PORTA &= B11000000;
    delay(3);                                               //Wait 3 milliseconds before the next loop.
  }

  gyro_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  Serial.println("Gyroscope calibration done!");
  for (int i = 0; i < 3; i++) Serial.println(gyro_cal[i]);
}

ISR(PCINT2_vect) {
  current_time = micros();
  //*============================================= Channel 1 =============================================
  if (PINK & B00000001)  {                      //Is input A8 high?
    if (last_channel_1 == 0) {                  //Input A8 changed from 0 to 1
      last_channel_1 = 1;                       //Remember current input state
      timer_1 = current_time;                   //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1)  {              //Input A8 is not high and changed from 1 to 0
    last_channel_1 = 0;                         //Remember current input state
    receiver_input[1] = current_time - timer_1; //Channel 1 is current_time - timer_1
  }

  //*============================================= Channel 2 =============================================
  if (PINK & B00000010)  {                      //Is input A9 high?
    if (last_channel_2 == 0) {                  //Input A9 changed from 0 to 1
      last_channel_2 = 1;                       //Remember current input state
      timer_2 = current_time;                   //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1)  {              //Input A9 is not high and changed from 1 to 0
    last_channel_2 = 0;                         //Remember current input state
    receiver_input[2] = current_time - timer_2; //Channel 2 is current_time - timer_2
  }

  //*============================================= Channel 3 =============================================
  if (PINK & B00000100)  {                      //Is input A10 high?
    if (last_channel_3 == 0) {                  //Input A10 changed from 0 to 1
      last_channel_3 = 1;                       //Remember current input state
      timer_3 = current_time;                   //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1)  {              //Input A10 is not high and changed from 1 to 0
    last_channel_3 = 0;                         //Remember current input state
    receiver_input[3] = current_time - timer_3; //Channel 3 is current_time - timer_3
  }

  //*============================================= Channel 4 =============================================
  if (PINK & B00001000)  {                      //Is input A11 high?
    if (last_channel_4 == 0) {                  //Input A11 changed from 0 to 1
      last_channel_4 = 1;                       //Remember current input state
      timer_4 = current_time;                   //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1)  {              //Input A11 is not high and changed from 1 to 0
    last_channel_4 = 0;                         //Remember current input state
    receiver_input[4] = current_time - timer_4; //Channel 4 is current_time - timer_4
  }

  //*============================================= Channel 5 =============================================
  if (PINK & B00010000)  {                      //Is input A12 high?
    if (last_channel_5 == 0) {               //Input A12 changed from 0 to 1
      last_channel_5 = 1;                       //Remember current input state
      timer_5 = current_time;                   //Set timer_5 to current_time
    }
  }
  else if (last_channel_5 == 1)  {              //Input A12 is not high and changed from 1 to 0
    last_channel_5 = 0;                         //Remember current input state
    receiver_input[5] = current_time - timer_5; //Channel 5 is current_time - timer_5
  }

  //*============================================= Channel 6 =============================================
  if (PINK & B00100000)  {                      //Is input A13 high?
    if (last_channel_6 == 0) {                  //Input A13 changed from 0 to 1
      last_channel_6 = 1;                       //Remember current input state
      timer_6 = current_time;                   //Set timer_6 to current_time
    }
  }
  else if (last_channel_6 == 1)  {              //Input A13 is not high and changed from 1 to 0
    last_channel_6 = 0;                         //Remember current input state
    receiver_input[6] = current_time - timer_6; //Channel 6 is current_time - timer_6
  }

  //*============================================= Channel 7 =============================================
  if (PINK & B01000000)  {                      //Is input A14 high?
    if (last_channel_7 == 0) {                  //Input A14 changed from 0 to 1
      last_channel_7 = 1;                       //Remember current input state
      timer_7 = current_time;                   //Set timer_7 to current_time
    }
  }
  else if (last_channel_7 == 1)  {              //Input A14 is not high and changed from 1 to 0
    last_channel_7 = 0;                         //Remember current input state
    receiver_input[7] = current_time - timer_7; //Channel 7 is current_time - timer_7
  }

  //*============================================= Channel 8 =============================================
  if (PINK & B10000000)  {                      //Is input A15 high?
    if (last_channel_8 == 0) {                  //Input A15 changed from 0 to 1
      last_channel_8 = 1;                       //Remember current input state
      timer_8 = current_time;                   //Set timer_8 to current_time
    }
  }
  else if (last_channel_8 == 1)  {              //Input A15 is not high and changed from 1 to 0
    last_channel_8 = 0;                         //Remember current input state
    receiver_input[8] = current_time - timer_8; //Channel 8 is current_time - timer_8
  }
}
