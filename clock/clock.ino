#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>               
#include <TimeLib.h>
#include <ESP32Servo.h>
#include "ESPAsyncWebServer.h"

// PINs
#define SERVO_PIN 26 
#define BUTTON_PIN 21 

// Global variables
Servo servoMotor;  
WiFiUDP ntpUDP;
int cur_speed;            
unsigned long last_unix_epoch; // Unix epoch time spot of the most recent reset()
int buttonLastState;
 
// Setting up wifi info 
const char *ssid     = "Fios-MXtV9";
const char *password = "fan355soy883cps";

const char *server_id = "ESP";
const char server_pwd = "esp";
 
// NTPClient for querying accurate Internet time
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", -14400, 60000);
 
char Time[ ] = "TIME:00:00:00";
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;
 
void setup() {
 
  Serial.begin(115200);
  
  // Setting up WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi.");
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nSuccessfully connected to WiFI.");

  // Setting up server
  print("\nSetting up server...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  print("\nSuccessfully set up server.");

  // This creates a new thread.
  // Calls back when receive inputs.
  server.on("/test",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
      uint8_t new_speed = *data_in;
      int speed = (int)new_speed;
      if(speed != cur_speed) {
        // reset everything then change speed
        accelerate(speed);
      }
  })

  // Initializations
  timeClient.begin(); // Setting up time client
  servoMotor.attach(SERVO_PIN);  // Setting up servo
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Setting up button
  cur_speed = 1;
  last_unix_epoch = timeClient.getEpochTime();
}

void reset() {
  timeClient.update();
  cur_speed = 1;
  last_unix_epoch = timeClient.getEpochTime();
}

// Pre-defined servo behavior for making sound signal
void moveServo(){
  Serial.println("Moving arm!");
  servoMotor.write(0);
  delay(1000);
  servoMotor.write(100);
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
  time_arr[0] = hour(local_unix_epoch);
  time_arr[1] = minute(local_unix_epoch);
  time_arr[2] = second(local_unix_epoch);
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
  getLocalTime(time);

  // Check whether button is hit
  if(buttonLastState == LOW && buttonCurState == HIGH) {
    Serial.print("Local Time: \n");
    printLocalTime();
    Serial.print("\nGlobal Time: \n");
    printGlobalTime();
    
    Serial.print("\nMoving servos to show hour...");
    for(int i = 0; i < time[0]; i++) moveServo();
    // Do something?
    for(int i = 0; i < time[1]/10; i++) moveServo();
    delay(2000); // Wait 2 seconds after button clicking event
  }

  // Check local time
  if(time[1] % 15 == 0) {
    // Move servo once every 15 minutes (in local clock);
    Serial.print("Local Time: \n");
    printLocalTime();
    Serial.print("\nGlobal Time: \n");
    printGlobalTime();
    moveServo();
  }
}