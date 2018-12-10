#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_ADDR 0x3C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

bool gyro_updating = false;
unsigned long difference, main_loop_timer, difference2;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void update_display(String m1, String m2 = "", String m3 = "", String m4 = "");

typedef union {
  int integer;
  byte one_byte;
  uint8_t two_bytes[2];
} converter;

converter number;
float result1, result2;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Wire.onReceive(receive_event);

  Serial.begin(115200);
  Serial.println("BEGIN");

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
}

void loop() {
  // put your main code here, to run repeatedly:
  update_display("ROLL: " + (String) receiver_input_channel_1,
                 "PITCH: " + (String) receiver_input_channel_2,
                 "THROTTLE: " + (String) receiver_input_channel_3,
                 "YAW: " + (String) receiver_input_channel_4);

  Serial.println(receiver_input_channel_1);
  Serial.println(receiver_input_channel_2);
  Serial.println(receiver_input_channel_3);
  Serial.println(receiver_input_channel_4);
  Serial.println();

  if (!gyro_updating) {
    Wire.beginTransmission (9);
    Wire.write(true);
    Wire.endTransmission();
  }

  maintain_loop_time();

  if (!gyro_updating) {
    Wire.beginTransmission (9);
    Wire.write(false);
    Wire.endTransmission();
  }

  delay(8);
}

void receive_event(int no_of_bytes) {
  if (no_of_bytes == 1) {
    while (Wire.available() < 1);
    gyro_updating = Wire.read();
    return;
  }

  while (Wire.available() < 4);
  number.one_byte = Wire.read();
  receiver_input_channel_1 = number.integer * 10;
  number.one_byte = Wire.read();
  receiver_input_channel_2 = number.integer * 10;
  number.one_byte = Wire.read();
  receiver_input_channel_3 = number.integer * 10;
  number.one_byte = Wire.read();
  receiver_input_channel_4 = number.integer * 10;
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 42000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
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

