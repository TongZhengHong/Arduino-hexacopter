#include <math.h>
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

//AHRS variables
#define sampleFreq  250.0f    // sample frequency in Hz
#define betaDef   0.1f    // 2 * proportional gain
float beta = betaDef;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

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
  //  calculate_pitch_roll();
  //  Serial.println("Roll angle: " + (String) angle_roll);
  //  Serial.println("Pitch angle: " + (String) angle_pitch);
  //  Serial.println();

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;

  MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

  delayMicroseconds(1000);

  // Serial.println(receiver_input[3]);
  // PORTA |= B00111111;
  // delayMicroseconds(receiver_input[3]);
  // PORTA &= B11000000;
  // delayMicroseconds(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pitch_roll() {
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

void calibrateSensors() {
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

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  Serial.print(q0);
  Serial.print(", ");
  Serial.print(q1);
  Serial.print(", ");
  Serial.print(q2);
  Serial.print(", ");
  Serial.print(q3);
  Serial.print(", ");
  Serial.println("");
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}