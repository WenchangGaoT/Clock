#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>               
#include <TimeLib.h>
#include <ESP32Servo.h>
#include "ESPAsyncWebServer.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// PINs
#define SERVO_PIN 26 
#define BUTTON_PIN 21 

// Global variables
Servo servoMotor;  
WiFiUDP ntpUDP;
int servoInitPos = 0;
int servoFinalPos = 20;
AsyncWebServer server(80);
int cur_speed;            
unsigned long last_unix_epoch; // Unix epoch time spot of the most recent reset()
int buttonLastState;
bool knocked;

// Setting up wifi info 
const char *ssid     = "P60Art";
const char *password = "fsj13579";

const char *server_id = "ESP";
const char *server_pwd = "espespespesp";
 
// NTPClient for querying accurate Internet time
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", -14400, 60000);
 
char Time[ ] = "TIME:00:00:00";
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;
 

int str2int(String s) {
  int ret = 0;
  Serial.println(s);
  for (int i = 0; i < s.length(); i++) {
    ret *= 10;
    ret += (s[i]-'0');
  }
  return ret;
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
 
  Serial.begin(115200);
  
  // Setting up WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi.");
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nSuccessfully connected to WiFI.\n");

  // Setting up server
  WiFi.softAP(server_id, server_pwd);
  Serial.print("\nSetting up server...");
  // WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);
  Serial.print("\nSuccessfully set up server.\n");

  // This creates a new thread.
  // Calls back when receive inputs.
  server.on("/test",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
      // uint8_t new_speed = *data_in;
      String val = String((char*)data_in, len);
      int new_speed = str2int(val);
      int speed = new_speed;
      Serial.print("Accelerate request\n");
      Serial.println(speed);
      if(speed != cur_speed) {
        // reset everything then change speed
        accelerate(speed);
      }
      request->send_P(200, "text/plain", String("accelerated").c_str());
  });
  server.begin();

  // Initializations
  timeClient.begin(); // Setting up time client
  servoMotor.attach(SERVO_PIN);  // Setting up servo
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Setting up button
  cur_speed = 1;
  last_unix_epoch = timeClient.getEpochTime();
  buttonLastState = HIGH;
  knocked = false;

  // Initialize servo position
  servoMotor.write(servoInitPos);
  delay(1000);
}

void reset() {
  timeClient.update();
  cur_speed = 1;
  last_unix_epoch = timeClient.getEpochTime();
}

// Pre-defined servo behavior for making sound signal
void moveServo(){
  Serial.println("Moving arm!");
  servoMotor.write(servoFinalPos);
  delay(250);
  servoMotor.write(servoInitPos);
  delay(1000);
}

void triggerServo(byte time[5]){
    Serial.print("\nMoving servos to show hour...\n");
    for(int i = 0; i < time[0]; i++) moveServo();
    // Serial.print("\nMoved %d times!", &time[0]);
    // Do something?
    Serial.print("Moving servos to show 10 minute mark...\n");
    delay(3000);
    for(int i = 0; i < time[1]/10; i++) moveServo();
    delay(3000);
    Serial.print("Moving servo to show minute marker...\n");
    for(int i = 0; i < time[1]%10; i++) moveServo();
}

void printGlobalTime() {
  timeClient.update();
  unsigned long unix_epoch = timeClient.getEpochTime();
  minute_ = minute(unix_epoch);
  hour_   = hour(unix_epoch);
  day_    = day(unix_epoch);
  month_  = month(unix_epoch);
  year_   = year(unix_epoch);
 
  Time[12] = second_ % 10 + 48;
  Time[11] = second_ / 10 + 48;
  Time[9]  = minute_ % 10 + 48;
  Time[8]  = minute_ / 10 + 48;
  Time[6]  = hour_   % 10 + 48;
  Time[5]  = hour_   / 10 + 48;

  Serial.print(Time);
}

void getLocalTime(byte* time_arr) {
  // Calculate locally accelerated time and return with the arguments passed in
  // time_arr[0]: hour in local time
  // time_arr[1]: minute in local time 
  // time_arr[2]: second in local time
  timeClient.update();
  unsigned long unix_epoch = timeClient.getEpochTime();
  unsigned long real_time_passed = unix_epoch - last_unix_epoch;
  unsigned long local_unix_epoch = last_unix_epoch + (unsigned long)cur_speed*real_time_passed;
  time_arr[0] = hour(local_unix_epoch) % 12;
  time_arr[1] = minute(local_unix_epoch) % 60;
  time_arr[2] = second(local_unix_epoch) % 60;
}

void printLocalTime() {
  byte time[5];
  getLocalTime(time);
  Time[12] = time[2] % 10 + 48;
  Time[11] = time[2] / 10 + 48;
  Time[9]  = time[1] % 10 + 48;
  Time[8]  = time[1] / 10 + 48;
  Time[6]  = time[0] % 10 + 48;
  Time[5]  = time[0] / 10 + 48;
  Serial.print(Time);
}

void accelerate(int speed) {
  timeClient.update();
  last_unix_epoch = timeClient.getEpochTime();
  cur_speed = speed;
}
 
void loop() {
  byte time[5];
  int buttonCurState = digitalRead(BUTTON_PIN);
  // getLocalTime(time);

  // Check whether button is hit
  if(buttonLastState == LOW && buttonCurState == HIGH) {
    Serial.print("Local Time: \n");
    printLocalTime();
    Serial.print("\nGlobal Time: \n");
    printGlobalTime();

    getLocalTime(time);

    triggerServo(time);

    delay(2000); // Wait 2 seconds after button clicking event

  }

  // Check local time
  getLocalTime(time);
  if(time[1] % 15 == 0 && !knocked) {
    // Move servo once every 15 minutes (in local clock);
    Serial.print("Local Time: \n");
    printLocalTime();
    Serial.print("\nGlobal Time: \n");
    printGlobalTime();
    triggerServo(time);
    knocked = true;
  } 
  else {
    if(time[1] % 15 != 0) knocked = false;
  }
  buttonLastState = buttonCurState;
}