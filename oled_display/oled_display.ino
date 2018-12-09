#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_ADDR 0x3C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

bool start = true;
byte start_sequence = 0;

//OLED screen
byte page_number = 1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void update_display(String m1, String m2 = "", String m3 = "", String m4 = "");

//Buttons
byte next_button = 3, prev_button = 2;
bool next_clicked = false, prev_clicked = false;

//timer
unsigned long difference, main_loop_timer;

//data
float angle_pitch, angle_roll;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int battery_voltage;

typedef union {
  int integer;
  byte one_byte;
  uint8_t two_bytes[2];
} converter;

void setup() {
  Serial.begin(115200);  Serial.println("TEST1");
  Wire.begin(8);
  TWBR = 12;

  Serial.println("TEST");
  Wire.onRequest(send_page);
  Wire.onReceive(update_info);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  update_display("SETUP");
  while (start) {
    if (start_sequence == 1) {
      update_display("SETUP", "Waiting for radio...");
    } else if (start_sequence == 2) {
      update_display("SETUP", "Calibrating sensors...");
    } else if (start_sequence == 3) {
      update_display("SETUP", "DONE!");
    } else if (start_sequence == 4) {
      start = false;
    }
  }
}

void loop() {
  check_button_click();
  set_OLED_screen(page_number);

  Wire.beginTransmission (9);
  Wire.write(true);
  Wire.endTransmission();

  difference = micros() - main_loop_timer;
  while (difference < 42000) difference = micros() - main_loop_timer;
  //Serial.println(difference);
  main_loop_timer = micros();

  Wire.beginTransmission (9);
  Wire.write(false);
  Wire.endTransmission();

  delay(8);
}

void check_button_click() {
  if (digitalRead(prev_button) == LOW) prev_clicked = true;
  else if (prev_clicked && digitalRead(prev_button) == HIGH) {
    prev_clicked = false;
    if (page_number > 1) page_number--;
  }
  else if (digitalRead(next_button) == LOW) next_clicked = true;
  else if (next_clicked && digitalRead(next_button) == HIGH) {
    next_clicked = false;
    if (page_number < 3) page_number++;
    else page_number = 1;
  }
}

void set_OLED_screen(int page) {
  switch (page) {
    case 2:
      update_display("ANGLES: ",
                     "PITCH: " + (String) angle_pitch,
                     "ROLL: " + (String) angle_roll);
      break;
    case 1:
      update_display("ROLL: " + (String) receiver_input_channel_1,
                     "PITCH: " + (String) receiver_input_channel_2,
                     "THROTTLE: " + (String) receiver_input_channel_3,
                     "YAW: " + (String) receiver_input_channel_4);
      break;
    case 3:
      update_display("BATTERY VOLTAGE: ", (String) battery_voltage + "V");
      break;
    default:
      update_display("ERROR");
      break;
  }
}

void update_display(String m1, String m2 = "", String m3 = "", String m4 = "") {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(m1);
  display.setCursor(0, 8);
  display.print(m2);
  display.setCursor(0, 16);
  display.print(m3);
  display.setCursor(0, 24);
  display.print(m4);

  display.display();
}

void send_page() {
  Wire.write(page_number);  //Send 1 byte over to main arduino
}

void update_info(int bytes) {
  if (start) {
    while (Wire.available() < 1);
    start_sequence = Wire.read();
    Serial.println(start_sequence);
    return;
  }

  converter number;
  switch (page_number) {
    case 2: //angles
      while (Wire.available() < 4);
      number.two_bytes[0] = Wire.read();
      number.two_bytes[1] = Wire.read();
      angle_roll = number.integer / 100.0;

      number.two_bytes[0] = Wire.read();
      number.two_bytes[1] = Wire.read();
      angle_pitch = number.integer / 100.0;
      break;
    case 1: //transmitter
      while (Wire.available() < 4);
      number.one_byte = Wire.read();
      receiver_input_channel_1 = number.integer * 10;
      number.one_byte = Wire.read();
      receiver_input_channel_2 = number.integer * 10;
      number.one_byte = Wire.read();
      receiver_input_channel_3 = number.integer * 10;
      number.one_byte = Wire.read();
      receiver_input_channel_4 = number.integer * 10;
      break;
    case 3: //battery voltage
      while (Wire.available() < 2);
      number.two_bytes[0] = Wire.read();
      number.two_bytes[1] = Wire.read();
      battery_voltage = number.integer;
      break;
  }
}


