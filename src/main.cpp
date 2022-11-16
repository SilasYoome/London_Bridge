#include <Arduino.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(A1, A2); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

#define home_button_pinA 11
#define home_button_pinB 9
#define start_button 12
#define bridge_close false
#define bridge_open true
#define ultrasonic_trig_pin 10
#define ultrasonic_echo_pin 3

bool bridge_state = bridge_close;
bool flag = true;

bool readingA;
bool readingB;

bool buttonState;
bool lastButtonState = HIGH;

bool play_muisc_flag = false;

bool ultrasonic_switch = false;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

float ultrasonic_num = 0.00;

void forward(int num);
void stop2(int num);
void back(int num);
bool init_state();
float ultrasonic_distance();

void setup() {
  Serial.begin(9600);
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    while(true);
  }
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  myDFPlayer.loop(1);  //Play the first mp3
  pinMode(ultrasonic_trig_pin, OUTPUT);
  pinMode(ultrasonic_echo_pin, INPUT);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(home_button_pinA, INPUT_PULLUP);
  pinMode(home_button_pinB, INPUT_PULLUP);
  pinMode(start_button, INPUT);
  Serial.println("init");
}

void loop() {
  readingA = digitalRead(home_button_pinA);
  readingB = digitalRead(home_button_pinB);
  if (ultrasonic_switch) {
    Serial.println("untrasonic");
    ultrasonic_num = ultrasonic_distance();
    Serial.println(ultrasonic_num);
  }
  if (ultrasonic_num <= 8.0 && ultrasonic_num > 0.0) {
    flag = true;
  }


  if (flag) {
    if (bridge_state == bridge_close) {
      if(play_muisc_flag){
        myDFPlayer.start();
        play_muisc_flag = 0;
      }
      Serial.println("close bridge");
      ultrasonic_switch = false;
      if (init_state()) {
        flag = false;
        bridge_state = bridge_open;
        ultrasonic_switch = true;
        myDFPlayer.stop();
        play_muisc_flag = 1;
      }
    }
    else if (bridge_state == bridge_open) {
      Serial.println("open bridge");
      ultrasonic_switch = false;
      ultrasonic_num = 0.00;
      forward(1);
      forward(2);
      myDFPlayer.start();
      for (int i = 0;i < 40;i++) {
        delay(1000);
      }
      stop2(1);
      stop2(2);
      flag = false;
      bridge_state = bridge_close;
      ultrasonic_switch = true;
      myDFPlayer.stop();
    }
  }
}

bool init_state() {
  static bool state_a = false;
  static bool state_b = false;
  if (readingA == HIGH) {
    back(1);
    state_a = false;
  }
  else if (readingA == LOW) {
    stop2(1);
    Serial.println("button a close");
    state_a = true;
  }
  if (readingB == HIGH)
  {
    back(2);
    state_b = false;
  }
  else if (readingB == LOW)
  {
    stop2(2);
    Serial.println("button b close");
    state_b = true;
  }
  if (state_a == true && state_b == true)
  {
    state_a = state_b = false;
    return true;
  }
  return false;
}


void forward(int num) {
  if (num == 1) {
    digitalWrite(7, HIGH);
    analogWrite(5, 250);
  }
  else if (num == 2)
  {
    digitalWrite(4, LOW);
    analogWrite(6, 250);
  }
}

void back(int num) {
  if (num == 1) {
    digitalWrite(7, LOW);
    analogWrite(5, 230);
  }
  else if (num == 2)
  {
    digitalWrite(4, HIGH);
    analogWrite(6, 250);
  }
}

void stop2(int num) {
  if (num == 1) {
    analogWrite(5, 0);
  }
  else if (num == 2)
  {
    analogWrite(6, 0);
  }
}

float ultrasonic_distance() {
  digitalWrite(ultrasonic_trig_pin, LOW);
  digitalWrite(ultrasonic_echo_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(ultrasonic_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic_trig_pin, LOW);
  unsigned long sonic_duration = pulseIn(ultrasonic_echo_pin, HIGH);
  float distance_cm = (sonic_duration / 2.0) / 29.1;
  return distance_cm;
}