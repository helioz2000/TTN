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

 /*
  * !!!! NOTE !!!!
  * RN2903 firmware must be RN2903AU version 0.9.7rc7
  * The RN2903 model. A firmware upgrade may be necessary.
  */
  
#include <Arduino.h>
#include "SodaqUBloxGPS.h"
#include <TheThingsNetwork.h>

/* Erwin's board id sodaq_one_01 */
// device EUI: 0004A30B001A26CA
const char *devAddr = "26061E73";
const char *nwkSKey = "0A92345A41BD5211A2D0F97BE661A6D7";
const char *appSKey = "7E1D1F90FC1CA4D8B1B2D74258815FB0";

/* Erwin's board id sodaq_one_02 
const char *devAddr = "26041E61";
const char *nwkSKey = "2C06B19CEC6EA3F61F7C00105F8A76E8";
const char *appSKey = "D2557C304858148DFC06E138A6D1D79D";
*/

/* Adam's board 
const char *devAddr = "26041764";
const char *nwkSKey = "55F86A5DA2D40A7E5D59BFB101EC613B";
const char *appSKey = "820D7D5039091FE80308BB51D155016C";
*/

#define UPDATE_INTERVAL 10000UL

unsigned long next_update = 0;

String payload;

#define BUFLEN 50
char buf[BUFLEN];

#define TXBUFLEN 11
uint8_t txBuffer[TXBUFLEN];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
float velocity;

// movement detection
const int MOVE_DELAY = 3;            // number speed > threshhold detections
const float MOVE_THRESHOLD = 1.0;    // min speed to detect movement
bool moving = false;                // TRUE when moving
int move_count = MOVE_DELAY;


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

    //debugSerial.print("-- Waiting for GPS fix ... ");
    sodaq_gps.init(GPS_ENABLE);
    sodaq_gps.on();
  /*
  digitalWrite(LED_RED, LOW);
  if (sodaq_gps.scan(true, 60000)) {
    debugSerial.println(" Received.");
    debugSerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
    debugSerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    debugSerial.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
    gotGPSfix = true;
  }
  digitalWrite(LED_RED, HIGH);
  */

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
    if (sodaq_gps.hasFix()) {
        LatitudeBinary = ((sodaq_gps.getLat() + 90) / 180) * 16777215;
        LongitudeBinary = ((sodaq_gps.getLon() + 180) / 360) * 16777215;
        hdopGps = sodaq_gps.getHDOP()*10;
        altitudeGps = sodaq_gps.getAlt();
        velocity = sodaq_gps.getSpeed();  

        // Latitude
        txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
        txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;

        // Longitude
        txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
        txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;

        // Altitude
        txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;

        // HDOP
        txBuffer[8] = hdopGps & 0xFF;

        // Speed
        txBuffer[9] = (int)velocity;    
    } else {
        for (int i=0; i<sizeof(txBuffer); i++) {
            txBuffer[i] = 0;
        }
    }
    
    // Battery Voltage
    txBuffer[10] = (getBatteryVoltage() - 2500) / 10; 
}

void sendData() {
    digitalWrite(LED_BLUE, LOW);
    buildTXbuffer();
    ttn.sendBytes( txBuffer, sizeof(txBuffer) );
    check_frame_ctr();
    digitalWrite(LED_BLUE, HIGH);
    debugSerial.println("-- STATUS");
    ttn.showStatus();
}

/*
 * Check speed and set "moving" if we are on the move
 */
void evaluate_velocity() {
  // no fix = no speed readout
  if (!sodaq_gps.hasFix()) {
    moving = false;
    return;
  }
  // we have a fix, lets check the speed
  double spd = sodaq_gps.getSpeed();
  if (spd >= MOVE_THRESHOLD) {
    if (move_count > 0) {
      move_count--;
    } else {
      moving = true;
    }
  } else {
    move_count = MOVE_DELAY;
    moving = false;
  }
}


#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f // One v1
#define BATVOLT_R2 2.0f // One v1
//#define BATVOLT_R1 4.7f // One v2/3/4   R18 = 4.7M
//#define BATVOLT_R2 10.0f // One v2/3/4  R19 = 10M
#define BATVOLT_PIN BAT_VOLT    //AIN5, pin 10 on ATSAMD21

uint16_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));

    return voltage;
}



/* 
 *  transmit update to TTN
 *  force: TRUE will force a transmission, FALSE will only transmit if we have a GPS fix and we are moving 
 */
void update(bool force) {

    // if we have no fix only a force will transmit 
    if ( !sodaq_gps.hasFix() ) {
        if (force) {
            sendData();
            debugSerial.println( "Sending empty data (No GPS fix)" );
        }
        return;
    }

    // we have a fix, so we also have a speed
    double spd = sodaq_gps.getSpeed();      

    // don't send while stationary unless forced
    if ( moving || force ) {
        debugSerial.println(String("Sending - velocity = ") + String(spd) );
        sendData();
    } else {
        debugSerial.println(String("velocity = ") + String(spd) + String(" (not moving) not transmitting data") );
    } 
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
        next_update = millis() + UPDATE_INTERVAL;
    }

    if (sodaq_gps.task() == true) {
        digitalWrite(led_pin, LOW);
        evaluate_velocity();
    } else {
        digitalWrite(led_pin, HIGH);
    }

    if (sodaq_gps.hasFix()) {
        digitalWrite(LED_RED, HIGH);
        led_pin = LED_GREEN;
    } else {
        digitalWrite(LED_GREEN, HIGH);
        led_pin = LED_RED;
    }
    
    //digitalWrite(led_pin, LOW);
    //delay(495);
  
}
