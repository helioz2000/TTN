/* 
 *  Configured for SODAQ ONE board
 *  
 */

#include <TheThingsNetwork.h>

// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED0012356";
const char *appKey = "18597E8AE9D25F2FCD4DE08F4E290764";

#define loraSerial Serial1
#define debugSerial SerialUSB

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan TTN_FP_AU915

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

void setup()
{
  pinMode(BUTTON, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW);  // GPS OFF

  
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;
  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
}

void loop()
{
  // debugSerial.println("-- LOOP");

  // Prepare payload 
  byte payload[] = "eb";
  //payload[0] = 'e';
  //payload[1] = 'b';

  // send when button is pressed
  if (digitalRead(BUTTON) == LOW) {
    digitalWrite(LED_BLUE, LOW);
    ttn.sendBytes(payload, sizeof(payload));
    delay(500);
    digitalWrite(LED_BLUE, HIGH);
  }

  // short flash of green LED
  delay(497);
  digitalWrite(LED_GREEN, LOW);
  delay(3);
  digitalWrite(LED_GREEN, HIGH);
}
