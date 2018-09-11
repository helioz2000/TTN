/*
 * Configured for SODAQ ONE board 
 */

#include <TheThingsNetwork.h>

// Set your DevAddr, NwkSKey, AppSKey and the frequency plan
//const char *devAddr = "00000000";
//const char *nwkSKey = "00000000000000000000000000000000";
//const char *appSKey = "00000000000000000000000000000000";

const char *devAddr = "2604190F";
const char *nwkSKey = "9F99EE8EA4CAD7DC7004D7F163F5CF23";
const char *appSKey = "B3E8090960052E0483040797F25A302C";

#define loraSerial Serial1
#define debugSerial SerialUSB

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_AU915

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  debugSerial.println("-- PERSONALIZE");
  ttn.personalize(devAddr, nwkSKey, appSKey);

  debugSerial.println("-- STATUS");
  ttn.showStatus();
}

void loop()
{
  debugSerial.println("-- LOOP");

  // Prepare payload of 1 byte to indicate LED status
  byte payload[2];
  //payload[0] = (digitalRead(BUTTON) == HIGH) ? 1 : 0;
  payload[0] = 0xFF;
  payload[1] = 0x11;

  // Send it off
  ttn.sendBytes(payload, sizeof(payload));

  delay(10000);
}
