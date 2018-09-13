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

// Non volatile storage on RN2903, used as EEPROM in this program.
const uint16_t NVM_START_ADDR = 0x300;
const uint16_t NVM_END_ADDR = 0x3FF;

#define loraSerial Serial1
#define debugSerial SerialUSB

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_AU915

// persistent storage
struct storageParams {
    uint32_t dnctr;    // downlink frame counter
    uint32_t upctr;    // uplink frame counter
    } nvmStorage;

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
//FlashStorage(flash, storageParams);

bool gotGPSfix = false;
int led_pin;
void setup()
{
  char buf[50];
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

  nvmStorage.dnctr =54896;
  nvmStorage.upctr = 4096;

  writeNvm();

  readNvm();
  
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

void writeNvm() {
  const byte *ptr = (const byte*) &nvmStorage;
  const int data_size = sizeof(nvmStorage);
  int address, x, value;
  String cmd;
  char buf[50];

  debugSerial.println("\n -- NVM write\n");
  for (x = 0; x<data_size; x++) {
    address = 0x300 + x;
    cmd = String(address, HEX) + String(" ") + String(ptr[x], HEX);
    cmd.toUpperCase();
    cmd = String("sys set nvm ") + cmd;
    debugSerial.println(cmd);
    loraSerial.println(cmd);
    loraSerial.readBytesUntil('\n', buf, 50);
  }  
  
}

void readNvm() {
  int i,x, n, m, rdlen;
  String cmd;
  char buf[50];
  byte *ptr = (byte*) &nvmStorage;
  const int data_size = sizeof(nvmStorage);
  debugSerial.println("\n -- NVM \n");
  for (x = 0; x<data_size; x++) {
    cmd = String(x + NVM_START_ADDR, HEX);
    cmd.toUpperCase();
    cmd = String("sys get nvm ") + cmd;
    loraSerial.println(cmd);
    rdlen = loraSerial.readBytesUntil('\n', buf, 50);
    debugSerial.println(cmd + String(" - ") + String(buf));
    // convert hex string to number
    n = 0; m = 1;
    for (i = rdlen - 2; i >= 0; i--) {
      if (buf[i] >= 'A') {
        n = n + ((buf[i]-'@' + 9) * m);
      } else {
        n = n + ((buf[i]-'0') * m);
      }
      m = m * 16;
    }
    ptr[x] = n;   
  }
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
  if (gotGPSfix) {
    digitalWrite(LED_BLUE, LOW);
  }
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
