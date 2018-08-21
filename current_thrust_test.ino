//TODO: Check receiver channel correct reading --> maybe need to convert channel readings
//! Change motor pin! Connect channel 3 of receiver to pin 8! 
//! Use Arduino UNOO

int motor_pin = 13;
volatile int receiver_thrust = 1100;
int current_time, timer_1, last_channel_1;

void setup(){
    Serial.begin(115200);

    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    
    pinMode(motor_pin, OUTPUT);

    Serial.println("Setup Done");
    Serial.println("Ensure throttle at lowest position!");
    while(receiver_thrust > 1050){
        delay(3);
    }

    Serial.println("Connect battery to ESC in the next 5 seconds!");
    for(int i = 5; i > 0; i--){
        Serial.print(i);
        Serial.print(" ");
        delay(1000);
    }
}

void loop(){
    //Serial.println(receiver_thrust);
    //return;

    digitalWrite(motor_pin, HIGH);
    delayMicroseconds(receiver_thrust);
    digitalWrite(motor_pin, LOW);
    delay(1);
}

ISR(PCINT0_vect){
  current_time = micros();
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_thrust = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
}