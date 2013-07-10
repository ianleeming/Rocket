char gpsdata[100];
int pps = 0;
char id[10];
long time = 0;
long lat = 0;
long lon = 0;
int sats = 0;
long alt = 0;
boolean gpson = false;


void setup() {
  pinMode(13, OUTPUT); //built in led pin
  pinMode(2, OUTPUT); //pin to send on/off signals
  pinMode(3, INPUT); //pin to recieve pulses sent by GPS every second
  Serial.begin(4800);
  Serial1.begin(4800);
  delay(100);
  if (Serial1.available() > 0) { //detects if the gps is active
    gpson = true;
  }
}

void loop() {
  if (Serial.available() > 0) { //If a byte is sent to the Arduino
    Serial.read();
    digitalWrite(2, HIGH); //send a .1 second pulse to the GPS to turn it off/on.
    delay(100);
    digitalWrite(2, LOW); //you must only send one byte or the GPS will turn on and off over and over
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
      lon = Serial1.parseInt() * 10000;
      lon = lon + Serial1.parseInt();
      Serial1.parseInt();
      sats = Serial1.parseInt();
      Serial1.parseInt();
      Serial1.parseInt();
      alt = Serial1.parseInt() * 10;
      alt = alt + Serial1.parseInt();
      Serial.print("Time: ");
      Serial.println(time);
      Serial.print("Lat: ");
      Serial.println(lat);
      Serial.print("Lon: ");
      Serial.println(lon);
      Serial.print("Sats: ");
      Serial.println(sats);
      Serial.print("Alt: ");
      Serial.println(alt);
      for (int i =  0; i < 7; i++) {
        id[i] = '\0'; //clear the identifier
      }
    }
    Serial1.readBytesUntil('\n', gpsdata, 99); //clear unwanted data from Serial1 buffer
  }
  
  pps = digitalRead(3);  //Turn on the LED when there's a pulse from the GPS
  digitalWrite(13, pps);
}
