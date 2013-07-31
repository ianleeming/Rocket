#include <Wire.h>
#include <LSM303.h>
#include <Servo.h>

char nsew = 'q';
char gpsdata[100];
int pps = 0;
char id[10];
long time = 0;
int latdeg = 0;
float latmin = 0;
float latdec = 0;
int londeg = 0;
float lonmin = 0;
float londec = 0;
float targetlat = 38;
float targetlon = -122.142731;
float dist = 0;
long lat = 0;
long lon = 0;
int sats = 0;
long alt = 0;
float R = 6371000;
float dlat = 0;
float dlon = 0;
float avglat = 0;
float deglength = 111320;
float nsdist = 0;
float ewdist = 0;
float bearing = 0;
int finalbearing = 0;
boolean gpson = false;
Servo servo1;
int servopos = 0;
LSM303 compass;
int hdg = 0;
int filtered_hdg = 0;

int target_hdg = 180;
int parachute_cmd;

//θ = atan2( sin(Δλ)*cos(φ2), cos(φ1) * sin(φ2) − sin(φ1) * cos(φ2) * cos(Δλ) )


void setup() {
  pinMode(13, OUTPUT); //built in led pin
  pinMode(4, OUTPUT); //pin to send on/off signals
  pinMode(5, INPUT); //pin to recieve pulses sent by GPS every second
  pinMode(6, OUTPUT); //pin to recieve signal to store waypoint
  Serial.begin(4800);
  Serial1.begin(4800);
  servo1.attach(9);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min.x = -613;
  compass.m_min.y = -660;
  compass.m_min.z = -532;
  compass.m_max.x = +504;
  compass.m_max.y = +456;
  compass.m_max.z = +536;
  delay(100);
  if (Serial1.available() > 0) { //detects if the gps is active
    gpson = true;
  }
}

void loop() {
  if (Serial.available() > 0) { //If a byte is sent to the Arduino
    Serial.read();
    digitalWrite(4, HIGH); //send a .1 second pulse to the GPS to turn it off/on.
    delay(100);
    digitalWrite(4, LOW); //you must only send one byte or the GPS will turn on and off over and over
    if (gpson == true) {
      Serial.println("GPS OFF");
      gpson = false;
    }
    else {
      Serial.println("GPS ON");
      gpson = true;
    }
  }
  
  if (Serial1.available() > 0) {
    Serial1.readBytesUntil(',', id, 6); //read only the identifier and check it
    if (id[0] == '$' && id[1] == 'G' && id[2] == 'P' && id[3] == 'G' && id[4] == 'G' && id[5] == 'A') {
      time = Serial1.parseInt() * 1000;
      time = time + Serial1.parseInt();
      lat = Serial1.parseInt() * 10000; //multiplying ensures only integer values are stored
      lat = lat + Serial1.parseInt();
      delay(3);
      Serial1.read();
      if (Serial1.read() == 'S') {
        lat = lat * -1;
      }
      lon = Serial1.parseInt() * 10000;
      lon = lon + Serial1.parseInt();
      delay(3);
      Serial1.read();
      if (Serial1.read() == 'W') {
        lon = lon * -1;
      }
      Serial1.parseInt();
      sats = Serial1.parseInt();
      Serial1.parseInt();
      Serial1.parseInt();
      alt = Serial1.parseInt() * 10;
      alt = alt + Serial1.parseInt();
      latdeg = lat / 1000000;
      latmin = lat % 1000000;
      latmin = latmin / 10000;
      latdec = latdeg + (latmin / 60);
      londeg = lon / 1000000;
      lonmin = lon % 1000000;
      lonmin = lonmin / 10000;
      londec = londeg + (lonmin / 60);
      if (digitalRead(6) == HIGH) {
        targetlat = latdec;
        targetlon = londec;
      }
      avglat = (targetlat + latdec) / 2;
      avglat = (avglat / 180) * M_PI;
      nsdist = (targetlat - latdec) * deglength;
      ewdist = (targetlon - londec) * deglength * cos(avglat);
      bearing = atan2(nsdist, ewdist);
      bearing = (bearing / M_PI) * -180;
      finalbearing = bearing + 90;
      finalbearing = (finalbearing + 360) % 360;
      dist = nsdist * nsdist + ewdist * ewdist;
      dist = sqrt(dist);
      Serial.print("Time: ");
      Serial.println(time);
      Serial.print("Lat: ");
      Serial.println(latdec, 6);
      Serial.print("Lon: ");
      Serial.println(londec, 6);
      Serial.print("Sats: ");
      Serial.println(sats);
      Serial.print("Alt: ");
      Serial.println(alt);
      Serial.print("Compass heading: ");
      Serial.println(hdg);
      Serial.print("Distance to Target: ");
      Serial.println(dist);
      Serial.print("Target Bearing: ");
      Serial.println(finalbearing);
      for (int i =  0; i < 7; i++) {
        id[i] = '\0'; //clear the identifier
      }
    }
    Serial1.readBytesUntil('\n', gpsdata, 99); //clear unwanted data from Serial1 buffer
  }
  
  compass.read();
  hdg = compass.heading((LSM303::vector){0,-1,0});
  filtered_hdg = (filtered_hdg + ((hdg - filtered_hdg + 540)%360 - 180) / 8 + 360) % 360;
  
  target_hdg = finalbearing;
  
  parachute_cmd = ((filtered_hdg - target_hdg + 540) % 360 - 180);
  if (parachute_cmd > 90) parachute_cmd = 90;
  if (parachute_cmd < -90) parachute_cmd = -90;
  servo1.write(90 + parachute_cmd);
 
  
  pps = digitalRead(5);  //Turn on the LED when there's a pulse from the GPS
  digitalWrite(13, pps);
}
