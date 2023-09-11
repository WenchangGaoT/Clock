#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>               
#include <TimeLib.h>
#include <ESP32Servo.h>

#define SERVO_PIN 26 // ESP32 pin GPIO26 connected to servo motor

Servo servoMotor;              
 
 
const char *ssid     = "P60Art";
const char *password = "fsj13579";
 
WiFiUDP ntpUDP;
 
 
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", -14400, 60000);
 
char Time[ ] = "TIME:00:00:00";
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;
 
 
 
void setup() {
 
  Serial.begin(115200);
 
  WiFi.begin(ssid, password);
  Serial.print("Connecting.");
 
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected");
  timeClient.begin();
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

}
 
 
void loop() {
 
  timeClient.update();
  unsigned long unix_epoch = timeClient.getEpochTime();    // Get Unix epoch time from the NTP server
 
  second_ = second(unix_epoch);
  if (last_second != second_) {
 
 
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

    if (second_ % 60 == 0){
      moveServo();
    }
 
    Serial.println(Time);
    Serial.println(Date);

    last_second = second_;
 
  }
  delay(500);
}

void moveServo(){
  Serial.println("Moving arm!");
  servoMotor.write(0);

  delay(1000);

  servoMotor.write(100);
}