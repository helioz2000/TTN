/*
 * Configured for SODAQ ONE board 
 * 
 * Transmit location data for TTN mapper.
 * 
 * Data is either transmitted by button or when moving
 * 
 * ABP frame counter:
 * A copy of the frame counter is stored NVM on the RN2903 module.
 * The stored copy is incremented by 10 whenever the transmiited frame
 * counter reached the stored value. This will cut down on NVM storage 
 * operations whilst generating a maximum delta of 10 between two
 * transmitted frames, that is, the last frame before shutdown and 
 * first frame after startup.
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

#define BUFLEN 50
char buf[BUFLEN];

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

#define FRAME_CTR_UPDATE 10

#define SEND_WHEN_STATIONARY false

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

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

  nvmStorage.dnctr = 0;
  nvmStorage.upctr = 0;

  if (!readNvm()) {   // first ever read (blank) will return false
    nvmStorage.dnctr = 0; // we need to override the values as they are invalid
    nvmStorage.upctr = 0;
    check_frame_ctr();
  } else {
    set_frame_ctr();  // set in LoRaWAN stack
    debugSerial.println(String("Uplink Frame Counter: ") + String(nvmStorage.upctr));
    //nvmStorage.upctr += FRAME_CTR_UPDATE;
    //writeNvm();
  }

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

  //debugSerial.println("\n -- NVM write\n");
  for (x = 0; x<data_size; x++) {
    address = 0x300 + x;
    cmd = String(address, HEX) + String(" ") + String(ptr[x], HEX);
    //cmd = String(address, HEX) + String(" ") + String(0xFF, HEX);
    cmd.toUpperCase();
    cmd = String("sys set nvm ") + cmd;
    //debugSerial.println(cmd);
    loraSerial.println(cmd);
    loraSerial.readBytesUntil('\n', buf, 50);
  }  
  
}


// read data from NV memory into storage
// returns false on first ever read (data is all 0xFF)
bool readNvm() {
  int i,x, n, m, rdlen;
  bool retval = false;
  String cmd;

  byte *ptr = (byte*) &nvmStorage;
  const int data_size = sizeof(nvmStorage);
  
  for (x = 0; x<data_size; x++) {
    cmd = String(x + NVM_START_ADDR, HEX);
    cmd.toUpperCase();
    cmd = String("sys get nvm ") + cmd;
    loraSerial.println(cmd);
    rdlen = loraSerial.readBytesUntil('\n', buf, BUFLEN);
    //debugSerial.println(cmd + String(" - ") + String(buf));
    
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

    // Store 
    ptr[x] = n;
    // detect blank record (first time use)
    if (n != 0xFF) retval = true;   
  }
  return retval;
}

// keep a copy of frame counter in permanent storage
// to restore on a restart 
void check_frame_ctr() {
    int rdlen;

    // Get uplink frame counter from LoRaWAN stack
    String cmd = "mac get upctr";
    loraSerial.println(cmd);
    rdlen = loraSerial.readBytesUntil('\n', buf, BUFLEN);
    String result = buf;
    long frame_upctr = result.toInt();
    debugSerial.println(String("Uplink Frame Counter = ") + result );
    // Check if we need to update persisitent copy
    if (frame_upctr >= nvmStorage.upctr) {
        nvmStorage.upctr = frame_upctr + FRAME_CTR_UPDATE;
        writeNvm();
        debugSerial.println(String("frame_upctr = ") + String(nvmStorage.upctr) );
    }
   
}

// set the frame counter value in LoRaWAN stack
void set_frame_ctr() {
    String cmd = "mac set upctr ";
    cmd += String(nvmStorage.upctr);
    loraSerial.println(cmd);
    loraSerial.readBytesUntil('\n', buf, BUFLEN);
    //debugSerial.println(String("uplink frame counter = ") + String(nvmStorage.upctr) );

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


// transmit update if moving or if force is true
void update(bool force) {
  double velocity;
  if (gotGPSfix) {
    digitalWrite(LED_BLUE, LOW);
  }
  if (sodaq_gps.scan(true, 2000)) {
        //payload = sodaq_gps.getDateTimeString() + ";";
        //payload += String(sodaq_gps.getLat(), 7) + ";";
        //payload += String(sodaq_gps.getLon(), 7) + ";";
        //payload += String(sodaq_gps.getHDOP(), 3) + ";";
        //payload += String(sodaq_gps.getNumberOfSatellites()) + ";#";
        velocity = sodaq_gps.getSpeed();      

        // don't send unless we are moving
        if ( (velocity > 1.0) || force ) {
            buildTXbuffer();
            ttn.sendBytes( txBuffer, sizeof(txBuffer) );
            //debugSerial.println(String("velocity = ") + String(velocity) );
            check_frame_ctr();
        } else {
            debugSerial.println(String("velocity = ") + String(velocity) + String(" (not moving) not transmitting data") );
        }
        gotGPSfix = true;
    } else {
        gotGPSfix = false;
        payload = "#";
    }
    
    digitalWrite(LED_BLUE, HIGH);
}

void loop()
{
  if (millis() > next_update) {
    update(SEND_WHEN_STATIONARY);
    next_update = millis() + UPDATE_INTERVAL;
  }

  // send when button is pressed
  if (digitalRead(BUTTON) == LOW) {
    update(true);
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
