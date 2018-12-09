#include<Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time, main_current_time, main_timer_loop;
volatile int receiver_input[9];

float angle_pitch, angle_roll, angle_yaw, angle_pitch_acc, angle_roll_acc;
float gyro_cal[4];
float roll_level_adjust, pitch_level_adjust;

void setupSensor() {
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial2.begin(9600);

  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }

  setupSensor();

  attachInterrupt(digitalPinToInterrupt(35), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(37), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(38), receiver_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(9), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(10), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(11), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(12), receiver_change, CHANGE);

  PORTA_PCR12 = (1 << 8); //configuring pin 3 as GPIO
  PORTA_PCR13 = (1 << 8); //configuring pin 4 as GPIO
  PORTA_PCR5 = (1 << 8); //configuring pin 25 as GPIO
  PORTA_PCR14 = (1 << 8); //configuring pin 26 as GPIO
  PORTA_PCR15 = (1 << 8); //configuring pin 27 as GPIO
  PORTA_PCR16 = (1 << 8); //configuring pin 28 as GPIO
  GPIOA_PDDR |= 127008; //0000 0000 0000 0001 1111 0000 0010 0000 --> Setting pins 3,4,25,26,27,28 as outputs

  PORTC_PCR5 = (1 << 8);  //configuring the pin as GPIO
  GPIOC_PDDR = (1 << 5);  //configuring the pin as an output
  GPIOC_PSOR = (1 << 5);  //setting LED pin high
  for (int i = 0; i < 2000; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    gyro_cal[1] += (float)gyro.gyro.x;
    gyro_cal[2] += (float)gyro.gyro.y;
    gyro_cal[3] += (float)gyro.gyro.z;
  }

  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;
  gyro_cal[3] /= 2000;
  Serial.println("Cal done!");

  //      233333333 receiver_input[1] = 1050;

  //  unsigned long loop_timer = micros();
  //  unsigned long loop_end = 5000000 + loop_timer;
  //  while (loop_timer <= loop_end) {
  //    loop_timer = micros();
  //    delayMicroseconds(100);
  //    testESC2(1000);
  //  }

  GPIOC_PCOR = (1 << 5);  //setting LED pin low
  //  delay(5000);
  //  GPIOC_PSOR = (1 << 5);  //setting LED pin high
  //  Serial.println("Insert battery");
  //
  //  loop_timer = micros();
  //  loop_end = 4000000 + loop_timer;
  //  while (loop_timer <= loop_end) {
  //    loop_timer = micros();
  //    delayMicroseconds(100);
  //    testESC2(2000);
  //  }
  //  Serial.println("OUT OF LOOP!");
  //
  //  loop_timer = micros();
  //  loop_end = 5000000 + loop_timer;
  //  while (loop_timer <= loop_end) {
  //    loop_timer = micros();
  //    delayMicroseconds(100);
  //    testESC2(1000);
  //  }
  //
  //  GPIOC_PCOR = (1 << 5);  //setting LED pin low

  Serial.println("LOOP START!");
}

void loop() {
  main_current_time = micros();

  sensors_event_t accel1, mag1, gyro1, temp1;

  lsm.getEvent(&accel1, &mag1, &gyro1, &temp1);

  float gyro_pitch = (float)gyro1.gyro.x - gyro_cal[1];
  float gyro_roll = (float)gyro1.gyro.y - gyro_cal[2];
  float gyro_yaw = (float)gyro1.gyro.z - gyro_cal[3];

  angle_pitch += gyro_pitch * 0.055;
  angle_roll += gyro_roll * 0.055;
  angle_yaw += gyro_yaw * 0.055;

  float constant = 0.02 * (3.142 / 180);
  angle_pitch -= angle_roll * sin(gyro_yaw * constant);
  angle_roll += angle_pitch * sin(gyro_yaw * constant);

  float accl_x = accel1.acceleration.x;
  float accl_y = accel1.acceleration.y;
  float accl_z = accel1.acceleration.z;

  long acc_total_vector = sqrt((accl_x * accl_x) + (accl_y * accl_y) + (accl_z * accl_z));

  if (abs(accl_y) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)accl_y / acc_total_vector) * 57.296;        //Calculate the pitch angle.
  }
  if (abs(accl_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)accl_x / acc_total_vector) * -57.296;        //Calculate the roll angle.
  }

  angle_pitch_acc -= 1.7;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 5.1;                                                    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01;
  angle_roll = angle_roll * 0.99 + angle_roll_acc * 0.01;

  Serial.println(angle_pitch);
  Serial.println(angle_roll);
  //Serial.println(angle_yaw);

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;

  float mag_x = (float)mag1.magnetic.x;
  float mag_y = (float)mag1.magnetic.y;
  float mag_z = (float)mag1.magnetic.z;

//  Serial.println(mag_x);
//  Serial.print(" | ");
//  Serial.print(mag_y);
//  Serial.print(" | ");
//  Serial.print(mag_z);
//  Serial.println("");

  //testESC();
  testESC2(receiver_input[2]);

  //  Serial.print("Channel 1: ");
  //  Serial.print(receiver_input[1]);
  //  Serial.print(" Channel 2: ");
  //  Serial.print(receiver_input[2]);
  //  Serial.print(" Channel 3: ");
  //  Serial.print(receiver_input[3]);
  //  Serial.print(" Channel 4: ");
  //  Serial.println(receiver_input[4]);
  //  Serial.println("");

  //      while(Serial2.available())
  //While there are characters to come from the GPS
  //  {
  //    gps.encode(Serial2.read());//This feeds the serial NMEA data into the library one char at a time
  //  }
  //  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  //  {
  //    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
  //    Serial.println("Satellite Count:");
  //    Serial.println(gps.satellites.value());
  //    Serial.println("Latitude:");
  //    Serial.println(gps.location.lat(), 6);
  //    Serial.println("Longitude:");
  //    Serial.println(gps.location.lng(), 6);
  //    Serial.println("Speed MPH:");
  //    Serial.println(gps.speed.mph());
  //    Serial.println("Altitude Feet:");
  //    Serial.println(gps.altitude.feet());
  //    Serial.println("");
  //  }

  delay(50);

  main_timer_loop = micros() - main_current_time;
  //Serial.println(main_timer_loop);
}

void receiver_change() {
  current_time = micros();
  if (GPIOC_PDIR & 2048) { //0000 0000 0000 0000 0000 0100 0000 0000
    if (last_channel_1 == 0) {                                              //Input 38 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_1 == 1) {                                          //Input 38 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  if (GPIOC_PDIR & 1024) { //0000 0000 0000 0000 0000 0100 0000 0000
    if (last_channel_2 == 0) {                                              //Input 38 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_2 == 1) {                                          //Input 38 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 1 is current_time - timer_1.
  }
  if (GPIOC_PDIR & 512) { //0000 0000 0000 0000 0000 0100 0000 0000
    if (last_channel_3 == 0) {                                              //Input 38 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_3 == 1) {                                          //Input 38 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 1 is current_time - timer_1.
  }
  if (GPIOC_PDIR & 256) { //0000 0000 0000 0000 0000 0100 0000 0000
    if (last_channel_4 == 0) {                                              //Input 38 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_4 == 1) {                                          //Input 38 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 1 is current_time - timer_1.
  }
}

void testESC2(int duration) {
  GPIOA_PSOR |= 127008; //0000 0000 0000 0001 1111 0000 0010 0000 --> Setting pins 3,4,25,26,27,28 as HIGH

  delayMicroseconds(duration);

  GPIOA_PCOR |= 127008; //0000 0000 0000 0001 1111 0000 0010 0000 --> Setting pins 3,4,25,26,27,28 as LOW
}

void testESC() {
  unsigned long loop_timer = micros();
  GPIOA_PSOR |= 127008; //0000 0000 0000 0001 1111 0000 0010 0000 --> Setting pins 3,4,25,26,27,28 as HIGH
  unsigned long timer_channel_1 = 1200 + loop_timer;

  while (GPIOA_PDOR >= 32) { //least significant bit (A5)
    unsigned long esc_loop_timer = micros();
    if (timer_channel_1 <= esc_loop_timer)GPIOA_PCOR |= 127008; //0000 0000 0000 0001 1111 0000 0010 0000 --> Setting pins 3,4,25,26,27,28 as LOW
  }
}
