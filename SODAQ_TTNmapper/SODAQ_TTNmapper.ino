/*
 * Configured for SODAQ ONE board 
 */
#include <Arduino.h>
#include <Sodaq_UBlox_GPS.h>
#include <TheThingsNetwork.h>

// Set your DevAddr, NwkSKey, AppSKey and the frequency plan
//const char *devAddr = "00000000";
//const char *nwkSKey = "00000000000000000000000000000000";
//const char *appSKey = "00000000000000000000000000000000";

const char *devAddr = "260417F7";
const char *nwkSKey = "709460A79D4186D657CFF59A41D1C5D1";
const char *appSKey = "628979BA757DA7ED841BCAB966A500B9";

#define UPDATE_INTERVAL 10000UL

unsigned long next_update = 0;

String payload;

uint8_t txBuffer[9];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;

#define loraSerial Serial1
#define debugSerial SerialUSB

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_AU915

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

bool gotGPSfix = false;
int led_pin;
void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

    digitalWrite(LED_RED, HIGH);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(BUTTON, INPUT);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  debugSerial.println("-- PERSONALIZE");
  ttn.personalize(devAddr, nwkSKey, appSKey);

  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.print("-- Waiting for GPS fix ... ");
  sodaq_gps.init(GPS_ENABLE);

  digitalWrite(LED_RED, LOW);
  if (sodaq_gps.scan(true, 60000)) {
    debugSerial.println(" Received.");
    debugSerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
    debugSerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    debugSerial.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
    gotGPSfix = true;
  }
  digitalWrite(LED_RED, HIGH);

}

void buildTXbuffer() {
    LatitudeBinary = ((sodaq_gps.getLat() + 90) / 180) * 16777215;
    LongitudeBinary = ((sodaq_gps.getLon() + 180) / 360) * 16777215;
    hdopGps = sodaq_gps.getHDOP()*10;
    altitudeGps = sodaq_gps.getAlt();

    txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;

    txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;
        
    txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
    txBuffer[7] = altitudeGps & 0xFF;
  
    txBuffer[8] = hdopGps & 0xFF;

}

void update(void) {
  digitalWrite(LED_BLUE, LOW);
  if (sodaq_gps.scan(true, 2000)) {
        payload = sodaq_gps.getDateTimeString() + ";";
        payload += String(sodaq_gps.getLat(), 7) + ";";
        payload += String(sodaq_gps.getLon(), 7) + ";";
        payload += String(sodaq_gps.getHDOP(), 3) + ";";
        payload += String(sodaq_gps.getNumberOfSatellites()) + ";#";
        buildTXbuffer();
        ttn.sendBytes( txBuffer, sizeof(txBuffer) );
        gotGPSfix = true;
    } else {
        gotGPSfix = false;
        payload = "#";
    }
    
    //ttn.sendBytes((const byte*)payload.c_str(), payload.length());
    digitalWrite(LED_BLUE, HIGH);
}

void loop()
{
  //debugSerial.println("-- LOOP");

  if (millis() > next_update) {
    update();
    next_update = millis() + UPDATE_INTERVAL;
  }

  // send when button is pressed
  if (digitalRead(BUTTON) == LOW) {
    update();
  }

    if (gotGPSfix) {
      led_pin = LED_GREEN;
    } else {
      led_pin = LED_RED;
    }
    digitalWrite(led_pin, LOW);
    delay (5);
    digitalWrite(led_pin, HIGH);
    delay(495);
  
}
