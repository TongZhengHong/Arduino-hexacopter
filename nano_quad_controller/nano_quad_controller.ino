//* /////////////////////////////////////////////////////////////////////////////////////////////
//*Convention:
//*  Channel 1: ROLL
//*  Channel 2: PITCH
//*  Channel 3: THROTTLE
//*  Channel 4: YAW
//* /////////////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>

byte eeprom_data[27];
int main_loop_timer, difference, start;
int battery_voltage;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
volatile int receiver_input[5];

double output_gain = 0.5;
int roll_output, pitch_output, yaw_output;

int throttle;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long loop_timer, esc_loop_timer;
int esc_1, esc_2, esc_3, esc_4;

void setup(){
    Serial.begin(115200); 

    for (start = 0; start <= 26; start++)
        eeprom_data[start] = EEPROM.read(start);
    while (eeprom_data[24] != 'J' || eeprom_data[25] != 'M' || eeprom_data[26] != 'B')
        delay(10);

    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

    DDRD |= B11110000;                                                        //Configure digital port  4, 5, 6 and 7 as output.
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); 

    Serial.println("Waiting for receiver...");
    Serial.println("Please turn on your transmitter and ensure that the throttle is in the LOWEST position.");
    //Wait until the receiver is active and the throtle is set to the lower position.
    while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
        receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
        receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
        pulse_esc();
    }

    digitalWrite(13, LOW);
    Serial.println("SETUP DONE! Connect your battery in the next five seconds");
    for (int i = 5; i > 0; i--) {
        Serial.print((String) i + " ");
        pulse_esc();
        delay(1000);
    }

    //Battery voltage calculation
    float voltage = analogRead(A0) * (5000 / 1023.0) + 20;
    battery_voltage = (voltage + 0.5) * 167;
    Serial.println("Current battery reading: " + (String) battery_voltage);
}

//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//
//* //////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////// *//
//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//

void loop(){
    convert_transmitter_values();

    check_start_stop();

    calculate_outputs();

    calculate_esc_output();

    set_esc();

    calculate_battery();

    difference = micros() - main_loop_timer;
    while (difference < 4000) {
        difference = micros() - main_loop_timer;
    }
}

//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//
//* //////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////// *//
//* ////////////////////////////////////////////////////////////////////////////////////////////////////// *//

void convert_transmitter_values() {
    receiver_input_channel_1 = convert_receiver_channel(1); //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2); //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
}

void check_start_stop() {
    //Serial.println("Start value: " + (String) start);
    //! For starting and stopping motors: Left and right sticks OUTWARDS!
    if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050 && receiver_input_channel_1 > 1950 && receiver_input_channel_2 < 1050)  {
        if (start == 0) {
            start = 1;
        }
        else if (start == 2) { //Stop motors --> check if already started
            start = 3;
        }
    }

    if (receiver_input_channel_4 > 1450 && receiver_input_channel_1 < 1550 && receiver_input_channel_2 > 1450)  {
        if (start == 1)    {
            Serial.println("START MOTORS");
            start = 2; //start motors
        }
        else if (start == 3)    {
            Serial.println("STOP MOTORS");
            start = 0; //Stop motors
        }
    }
}

void calculate_outputs() {
    roll_output = receiver_input_channel_1 - 1500;
    pitch_output = receiver_input_channel_2 - 1500;
    yaw_output = receiver_input_channel_4 - 1500;

    roll_output *= output_gain;
    pitch_output *= output_gain;
    yaw_output *= output_gain;
}

void calculate_esc_output() {
    throttle = receiver_input_channel_3;

    if (start == 2) {
        if (throttle > 1850) throttle = 1850;
        esc_1 = throttle - pitch_output - roll_output - yaw_output; //*(CW)
        esc_2 = throttle + pitch_output - roll_output + yaw_output; //*(ACW)
        esc_3 = throttle + pitch_output + roll_output - yaw_output; //*(CW)
        esc_4 = throttle - pitch_output + roll_output + yaw_output; //*(ACW)

        /*if (battery_voltage < 830 && battery_voltage > 600)    {                                                           //Is the battery connected?
            esc_1 += esc_1 * ((830 - battery_voltage) / (float) 3500); //Compensate the esc-1 pulse for voltage drop.
            esc_2 += esc_2 * ((830 - battery_voltage) / (float) 3500); //Compensate the esc-2 pulse for voltage drop.
            esc_3 += esc_3 * ((830 - battery_voltage) / (float) 3500); //Compensate the esc-3 pulse for voltage drop.
            esc_4 += esc_4 * ((830 - battery_voltage) / (float) 3500); //Compensate the esc-4 pulse for voltage drop.
        }*/

        if (esc_1 < 1050)      esc_1 = 1050;
        if (esc_2 < 1050)      esc_2 = 1050;
        if (esc_3 < 1050)      esc_3 = 1050;
        if (esc_4 < 1050)      esc_4 = 1050;

        if (esc_1 > 2000)      esc_1 = 2000;
        if (esc_2 > 2000)      esc_2 = 2000;
        if (esc_3 > 2000)      esc_3 = 2000; 
        if (esc_4 > 2000)      esc_4 = 2000;
    } else {
        esc_1 = 1000; 
        esc_2 = 1000; 
        esc_3 = 1000; 
        esc_4 = 1000;
    }
}

void set_esc() {
    loop_timer = micros();
    PORTD |= B11110000;                   //Set digital pin 22 - 27 as HIGH

    timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the faling edge of the esc-1 pulse.
    timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the faling edge of the esc-2 pulse.
    timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the faling edge of the esc-3 pulse.
    timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the faling edge of the esc-4 pulse.
    
    while (PORTD >= 16)  {                 //Stay in this loop until output 22 - 27 are LOW.
        esc_loop_timer = micros();
        if (timer_channel_1 <= esc_loop_timer) PORTD &= B11101111; //Set digital output 22 to low if the time is expired.
        if (timer_channel_2 <= esc_loop_timer) PORTD &= B11011111; //Set digital output 23 to low if the time is expired.
        if (timer_channel_3 <= esc_loop_timer) PORTD &= B10111111; //Set digital output 24 to low if the time is expired.
        if (timer_channel_4 <= esc_loop_timer) PORTD &= B01111111; //Set digital output 25 to low if the time is expired.
    }
}

void calculate_battery() {
    float diodeForward = 0.5;
    int potDivider = 167;

    int sensorValue = analogRead(A0);
    float voltage = sensorValue * (5.0/1023);
    battery_voltage = (voltage + diodeForward) * potDivider;
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

ISR(PCINT0_vect){
    current_time = micros();
    //* ========================================= Channel 1 =========================================
    if(PINB & B00000001){                                                     //Is input 8 high?
        if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
            last_channel_1 = 1;                                                   //Remember current input state.
            timer_1 = current_time;                                               //Set timer_1 to current_time.
        }
    }
    else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
        last_channel_1 = 0;                                                     //Remember current input state.
        receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
    }
    //* ========================================= Channel 2 =========================================
    if(PINB & B00000010){                                                    //Is input 9 high?
        if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
            last_channel_2 = 1;                                                   //Remember current input state.
            timer_2 = current_time;                                               //Set timer_2 to current_time.
        }
    }
    else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
        last_channel_2 = 0;                                                     //Remember current input state.
        receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
    }
    //* ========================================= Channel 3 =========================================
    if(PINB & B00000100){                                                    //Is input 10 high?
        if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
            last_channel_3 = 1;                                                   //Remember current input state.
            timer_3 = current_time;                                               //Set timer_3 to current_time.
        }
    }
    else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
        last_channel_3 = 0;                                                     //Remember current input state.
        receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
    }
    //* ========================================= Channel 4 =========================================
    if(PINB & B00001000){                                                    //Is input 11 high?
        if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
            last_channel_4 = 1;                                                   //Remember current input state.
            timer_4 = current_time;                                               //Set timer_4 to current_time.
        }
    }
    else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
        last_channel_4 = 0;                                                     //Remember current input state.
        receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
    }
}

void pulse_esc() {
    PORTD |= B00001111;
    delayMicroseconds(1000);
    PORTD &= B11110000;
    delay(3);
}
