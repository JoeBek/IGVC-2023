#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11
#define lightPin 5
#define stopPin 4

//#include <QMC5883LCompass.h>
//QMC5883LCompass compass;

int BLINK_DELAY = 500;

char c = ' ';
boolean NL = true;

unsigned long prevTime = millis();

bool blinking = false;
bool lightOn = false;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

bool usbPortLock = false;

void setup()  {
    Serial.begin(9600);

    // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    pinMode(lightPin, OUTPUT);
    pinMode(stopPin, OUTPUT);
    
    digitalWrite(stopPin, LOW);

    // Set the baud rate for the SoftwareSerial object
    mySerial.begin(9600);

    /*
    compass.init();
    compass.setCalibrationOffsets(-246.00, -1161.00, -1318.00);
    compass.setCalibrationScales(0.91, 1.06, 1.04);
    */
}

void loop() {
    if (blinking) {
      if (millis() > prevTime + BLINK_DELAY) {
        prevTime = millis();
        if (lightOn){
          digitalWrite(lightPin, LOW);
          lightOn = false;
        } else {
          digitalWrite(lightPin, HIGH);
          lightOn = true;
        }
      }
    } else {
      digitalWrite(lightPin, HIGH);
    }
    
    if (mySerial.available()) {
        c = mySerial.read();
        Serial.write(c);
        if (!usbPortLock) { 
          usbPortLock = true; //lock the usb port from other serial communication while the arduino is attempting to parrot a message
        }
        if (c == '\n') {
          usbPortLock = false; //unlock the usb port at end of line
        }
    }

    if (Serial.available()) {
      c = Serial.read();
      if (c == 'Z') {
        blinking = !blinking;
      }
      if (c == 'Q') {
        digitalWrite(stopPin, HIGH);
      }
      mySerial.write(c);
    }

/*
    compass.read();

    byte a = compass.getAzimuth();
    // Output here will be a value from 0 - 15 based on the direction of the bearing / azimuth.
    byte b = compass.getBearing(a);

    if (!usbPortLock) {
      Serial.print("Compass Value: ");
      Serial.println(b);
    }
*/
}
