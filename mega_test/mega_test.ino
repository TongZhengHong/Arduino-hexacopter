#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

//Transmitter variables
byte last_channel_3;
unsigned long timer_3, current_time;
volatile int receiver_input[9];

//Sensor variables
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
long acc_x, acc_y, acc_z, acc_total_vector;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_cal[4], acc_pitch_cal, acc_roll_cal;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(500);

  if (!lsm.begin())  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }

  setupSensor();
  calibrateSensors();

  PCICR |= (1 << PCIE2);     // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port K.
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 (digital input A10)to trigger an interrupt on state change

  DDRA |= B00111111;         //Configure digital pin 22 - 27 as output.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  calculate_pitch_roll();
  Serial.println("Roll angle: " + (String) angle_roll);
  Serial.println("Pitch angle: " + (String) angle_pitch);
  Serial.println();

  delay(100);

  // Serial.println(receiver_input[3]);
  // PORTA |= B00111111;
  // delayMicroseconds(receiver_input[3]);
  // PORTA &= B11000000;
  // delayMicroseconds(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pitch_roll(){
  // gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll * 0.3);    //Gyro pid input is deg/sec.
  // gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch * 0.3); //Gyro pid input is deg/sec.
  // gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);       //Gyro pid input is deg/sec.

  sensors_event_t accel1, mag1, gyro1, temp1;
  lsm.getEvent(&accel1, &mag1, &gyro1, &temp1);

  gyro_pitch = (double)gyro1.gyro.x - gyro_cal[1];
  gyro_roll = (double)gyro1.gyro.y - gyro_cal[2];
  gyro_yaw = (double)gyro1.gyro.z - gyro_cal[3];

  angle_pitch += gyro_pitch * 0.1;
  angle_roll += gyro_roll * 0.1;

  float constant = 0.02 * (3.142 / 180);
  angle_pitch -= angle_roll * sin(gyro_yaw * constant);
  angle_roll += angle_pitch * sin(gyro_yaw * constant);

  float accl_x = accel1.acceleration.x;
  float accl_y = accel1.acceleration.y;
  float accl_z = accel1.acceleration.z;

  long acc_total_vector = sqrt((accl_x * accl_x) + (accl_y * accl_y) + (accl_z * accl_z));

  if (abs(accl_y) < acc_total_vector)  {                               //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)accl_y / acc_total_vector) * 57.296; //Calculate the pitch angle.
  }
  if (abs(accl_x) < acc_total_vector)  {                               //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)accl_x / acc_total_vector) * -57.296; //Calculate the roll angle.
  }

  angle_pitch_acc -= acc_pitch_cal; //Accelerometer calibration value for pitch.
  angle_roll_acc -= acc_roll_cal;   //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02; //Correct the drift of gyro pitch angle with the accl pitch angle.
  angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;    //Correct the drift of gyro roll angle with the accl roll angle.
}

ISR(PCINT2_vect) {
  current_time = micros();
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
}

void calibrateSensors(){
  ////////////////////////////////////////// Gyroscope calibration //////////////////////////////////////////
  for (int i = 0; i < 2000; i++)  {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    gyro_cal[1] += (float)gyro.gyro.x;
    gyro_cal[2] += (float)gyro.gyro.y;
    gyro_cal[3] += (float)gyro.gyro.z;
  }

  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;
  gyro_cal[3] /= 2000;

  Serial.println("Gyroscope calibration done!");
  for (int i = 0; i < 3; i++) Serial.println(gyro_cal[i]);

  ////////////////////////////////////////// Accelometer calibration ////////////////////////////////////////
  for (int i = 0; i < 2000; i++)  {
    sensors_event_t accel1, mag1, gyro1, temp1;
    lsm.getEvent(&accel1, &mag1, &gyro1, &temp1);

    gyro_pitch = (double)gyro1.gyro.x - gyro_cal[1];
    gyro_roll = (double)gyro1.gyro.y - gyro_cal[2];
    gyro_yaw = (double)gyro1.gyro.z - gyro_cal[3];

    angle_pitch += gyro_pitch * 0.005;
    angle_roll += gyro_roll * 0.005;

    float constant = 0.02 * (3.142 / 180);
    angle_pitch -= angle_roll * sin(gyro_yaw * constant);
    angle_roll += angle_pitch * sin(gyro_yaw * constant);

    float accl_x = accel1.acceleration.x;
    float accl_y = accel1.acceleration.y;
    float accl_z = accel1.acceleration.z;

    long acc_total_vector = sqrt((accl_x * accl_x) + (accl_y * accl_y) + (accl_z * accl_z));

    if (abs(accl_y) < acc_total_vector)    {                                                                    //Prevent the asin function to produce a NaN
      angle_pitch_acc = asin((float)accl_y / acc_total_vector) * 57.296; //Calculate the pitch angle.
    }
    if (abs(accl_x) < acc_total_vector)    {                                                                    //Prevent the asin function to produce a NaN
      angle_roll_acc = asin((float)accl_x / acc_total_vector) * -57.296; //Calculate the roll angle.
    }

    angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01; //Correct the drift of gyro pitch angle with the accl pitch angle.
    angle_roll = angle_roll * 0.99 + angle_roll_acc * 0.01;    //Correct the drift of gyro roll angle with the accl roll angle.

    acc_pitch_cal += angle_pitch;
    acc_roll_cal += angle_roll;
  }

  acc_pitch_cal /= 2000;
  acc_roll_cal /= 2000;

  angle_pitch = 0;
  angle_roll = 0;

  Serial.println("Accelerometer calibration done!");
  Serial.println(acc_pitch_cal);
  Serial.println(acc_roll_cal);
}

void setupSensor(){
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