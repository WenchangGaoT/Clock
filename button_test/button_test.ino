#define BUTTON_PIN 21 // GPIO21 pin connected to button
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>               
#include <TimeLib.h>
#include <ESP32Servo.h>

#define SERVO_PIN 26 // ESP32 pin GPIO26 connected to servo motor
//time init
Servo servoMotor;              
 
 
const char *ssid     = "P60Art";
const char *password = "fsj13579";
 
WiFiUDP ntpUDP;
 
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", -14400, 60000);
 
char Time[ ] = "TIME:00:00:00";
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;
 


// button init
int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin

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
  // initialize the pushbutton pin as an pull-up input
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() 
{
  currentState = digitalRead(BUTTON_PIN);

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

    
 
    Serial.println(Time);
    Serial.println(Date);

    last_second = second_;
  }
  if(lastState == LOW && currentState == HIGH)
  {
    Serial.println("button hit!");
  }
  if (second_ % 60 == 0 || (lastState == LOW && currentState == HIGH))
  {
    moveServo();
  }
  // save the last state
  lastState = currentState;
}


void moveServo(){
  Serial.println("Moving arm!");
  servoMotor.write(50);

  delay(5000);

  servoMotor.write(100);
}
