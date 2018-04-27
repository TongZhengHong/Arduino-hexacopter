//Transmitter variables
byte last_channel_3;
unsigned long timer_3, current_time;
volatile int receiver_input[9];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  PCICR |= (1 << PCIE2);     // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port K.
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 (digital input A10)to trigger an interrupt on state change

  DDRA |= B00111111;         //Configure digital pin 22 - 29 as output.
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(receiver_input[3]);
  PORTA |= B00111111;
  delayMicroseconds(receiver_input[3]);
  PORTA &= B11000000;
  delayMicroseconds(100);
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