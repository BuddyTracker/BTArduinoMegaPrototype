#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <LinkedList.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

//LinkedList documentation: https://github.com/ivanseidel/LinkedList

static const int RXPin = 11, TXPin = 10, wayPointButtonPin = 44;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

static void feedGPS();
unsigned long timeOfLastFix = 0;
float fieldOfView = 90; // to be adjusted via companion app
int numLEDs = 7; // must / should be an odd number
// minus 1 because of center LED, plus 0.5 so that edge LED has same "width" as others
int degreesPerLED = fieldOfView / (numLEDs - 1 + 0.5);
int minSats = 4;

//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //OLD sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800); //depends on type of pixels!

const int maxWaypoints = 3;

struct Waypoint{
    double LAT;
    double LNG;
    Waypoint(){
        LAT = NAN;
        LNG = NAN;
    }
};
Waypoint* savedWaypoints;

struct Buddy{
    double LAT;
    double LNG;
    double ID; //the BNO055 has a unique queriable ID!!
    int headingDegrees;
    int lastTimeReceived;
};
LinkedList<Buddy> buddies;

//TEST CODE
double WAYPOINT_LAT = 51.508131, WAYPOINT_LON = -0.128002; //London
//TEST CODE

void setup() {
    Serial.begin(9600);
    ss.begin(GPSBaud);
    pinMode(wayPointButtonPin, OUTPUT);
    Serial.println("Start BuddyTracker");
    
    strip.begin();
    strip.setBrightness(100);

    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    savedWaypoints = new Waypoint[maxWaypoints]();
    //TEST CODE
    savedWaypoints[0].LAT = 51.508131; //London LAT
    savedWaypoints[0].LNG = -0.128002; //London LNG
    //TEST CODE

    feedGPS();

    //ADD BNO055 calibration. (should just need to spin in a circle) NEEDS display color.
    uint8_t sys, gyro, accel, mag = 0;
    while(mag < 1){
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print("magnetometer calibration: ");
        Serial.println(mag, DEC);
        delay(100);
    }
}

void loop() {
    if(digitalRead(wayPointButtonPin) == HIGH && sizeof(savedWaypoints) < maxWaypoints){
        //set waypoint
        for(int i = 0; i < sizeof(*savedWaypoints) - 1; i += 1)
            if(isnan(savedWaypoints[i].LAT) || isnan(savedWaypoints[i].LNG)){
                savedWaypoints[i].LAT = gps.location.lat();
                savedWaypoints[i].LNG = gps.location.lng();
            }
    }
    
    feedGPS();
    
    // set all pixels to 'off'
    for(int i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, 0, 0, 0);
    }

    //Take average of multiple readings
    int numReadings = 6;
    sensors_event_t event;
    float headingDegrees = 0.0;
    for(int currentReading = 0; currentReading < numReadings; currentReading++){
        //mag.getEvent(&event);
        bno.getEvent(&event);
        //heading += atan2(event.magnetic.y, event.magnetic.x);
        headingDegrees += event.orientation.x;
        delay(10); //required to get reading. NOT sure of exact value.
    }
    headingDegrees /= numReadings;
    
    int maxOtherBuddies = 8; // for now

    //array positions correspond to one another
    int otherBuddyIDs[maxOtherBuddies];
    int otherBuddyDegrees[maxOtherBuddies];
    //ALSO NEED COLOR. Used for turning off non-existent buddies. Also app color customization (later)

    //TEST CASE
    otherBuddyDegrees[0]=
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      WAYPOINT_LAT, 
      WAYPOINT_LON);
    //TEST CASE

    /*
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    //This can be found/calculated with GPS module
    float declinationAngle = 0.22;
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*PI;
    // Check for wrap due to addition of declination.
    else if(heading > 2*PI)
        heading -= 2*PI;
    
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;
    */

    for(int i = 0; i < buddies.size(); i++){
        int degreesDifference = headingDegrees - buddies.get(i).headingDegrees;
        if(degreesDifference > 180) degreesDifference -= 360; //correct wraparound
        if(abs(degreesDifference) > fieldOfView / 2)
            break;
        strip.setPixelColor(degreesDifference / degreesPerLED + numLEDs / 2, 150);
    }
    /*int degreesDifference = headingDegrees - otherBuddyDegrees[0];
    if(degreesDifference > 180) degreesDifference -= 360; //correct wraparound
    if(!(abs(degreesDifference) > fieldOfView / 2))
        strip.setPixelColor(degreesDifference / degreesPerLED + numLEDs / 2, 0, 0, 255);*/

    strip.show();
    Serial.print("heading:  ");
    Serial.print(headingDegrees);
    Serial.print("\t\tlat:      ");
    Serial.print(gps.location.lat());
    Serial.print("\t\tlon:      ");
    Serial.println(gps.location.lng());
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void feedGPS(){
    if(millis() - timeOfLastFix > 10*1000 && gps.satellites.value() < minSats){ //set all LEDs to RED
        for(int i=0; i<strip.numPixels(); i++)
            strip.setPixelColor(i, 255, 0, 0);
        strip.show();
    }

    do{
        while (ss.available())
            gps.encode(ss.read());
        Serial.print("Number of Satelittes: ");
        Serial.println(gps.satellites.value());
    } while(gps.satellites.value() < minSats);
}
