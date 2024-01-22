/*
NOTES: Switching to basic analog temp sensor, no humidity anymore
       Switching to basic analog light sensor
*/
/* ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⢀⡀
  ⠀⣠⣶⣿⠆⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⡎⢻⣽⢶⢀
  ⢀⣵⣔⢺⡆⠀⠑⠦⡀⠀⠀⠀⠀⠀⠀⠀⠀⣇⡀⠀⢉⠆⡜⡱⡄
  ⠀⠸⢈⡏⠀⠀⠀⢲⣮⣦⡀⠀⠀⠀⢀⠔⠊⠁⠀⠀⠹⡀⠴⠟⠽⠄
⠀  ⠀⢸⠀⣇⣆⣦⠑⢈⠁⠵⠒⠒⢞⡁⠀⠀⠀⠀⡀⠀⠀⠓⠿⣟⠄
⠀  ⠀⣸⠄⠿⠘⠚⠉⠀⠀⠀⠁⠀⠀⠀⠉⠑⠂⠔⢓⠠⠞⠀⢀⠇
⠀  ⠀⢸⡀⠀⠀⠀⠈⠒⢄⠀⠀⠀⠀⡐⢀⡇⠀⣀⣚⠀⢴⠀⡌
⠀  ⠀⠀⢳⡀⠀⠀⠀⠀⠀⠹⠆⡀⠪⠂⣊⠤⠒⠊⠙⠲⡤⠋
  ⠀⠀⠀⠀⠙⠦⡀⠀⠀⠀⠀⠀⡀⠓⠈⠀⠀⠀⠀⠒⢺⠃
⠀⠀⠀⠀  ⠀⠀⠈⢦⠀⠒⠂⠀⠀⠀⣱⣀⡀⠀⢘⡵⠁
⠀⠀⠀  ⠀⠀⠀⠀⠈⢣⡀⠀⠀⠀⠀⣻⣿⡿⠷⠊
⠀⠀  ⠀⠀⠀⠀⠀⠀⠀⠙⢢⢤⠤⠚⢶⡈
⠀  ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠂⠀⠀⠈⡧⡤⠚
  ⠀⠀⠀⠀⠀⠀⠠⣽⠷⠟⠓⠓⠀⠟⠒⠋
Chicken coop automation by Lee Woodridge */

#include <EEPROM.h>  // EEPROM library

/*
#include <Adafruit_AHTX0.h>  // AHT20 temperature & humidity sensor library

Adafruit_AHTX0 tempHumidSensor;  // I2C pins are A4 (SDA) & A5 (SCL)
*/

const int eAddress = 0;  // EEPROM address to store door position

const int SwitchUpPin = 6;    // Coop door toggle switch up position pin
const int SwitchDownPin = 7;  // Coop door toggle switch down position pin
const int Relay1Pin = 8;      // Coop heater relay pin
const int Relay2Pin = 9;      // Water heater relay pin
const int LightSensor = 14;   // Light sensor pin
/*
const int TempSensor = 00;    // Temp sensor pin
*/

const unsigned long eventTimer1 = 10 * 60 * 1000;    // Event #1: Time interval to record temp & humidity sensor data (Minutes)
const unsigned long eventTimer2 = 10 * 60 * 1000;    // Event #2: Time interval to control door based on light level when switch is in neutral position (Minutes)
const unsigned long doorDelay = 5 * 60 * 60 * 1000;  // Door opening delay for cold weather (Hours)
unsigned long eventTimer1PreviousTime = 0;
unsigned long eventTimer2PreviousTime = 0;
unsigned long doorDelayStartTime = 0;  // The time we started counting from, in case toggle position gets changed

int storedLight = 0;              // Last known light level (PUT RANGE HERE)
float storedTemp = 3.4028235E38;  // Last known temperature (°C) -- DO WE NEED FLOAT???
/*float storedHumid = 3.4028235E38;  // Last known relative humidity (%RH)*/
char doorPos = ' ';    // Door position; ' '=Unknown, 'U'=Up, 'D'=Down
char switchPos = ' ';  // Switch position; ' '=Unknown, 'U'=Up, 'N'=Neutral, 'D'=Down

const int thresholdTempDelay = -10;   // Temperature to trigger door open delay (<=)
const int thresholdTempRelay1 = -15;  // Temperature to trigger coop heater relay (<=)
const int thresholdTempRelay2 = 0;    // Temperature to trigger water heater relay (<=)
/*const int thresholdHumid = 999;       // UNUSED (!=)*/
const int thresholdLightLower = 000;  // Light level to lower door at (>=)
const int thresholdLightRaise = 000;  // Light level to raise door at (<=)

// ---

void setup() {
  Serial.begin(115200);
  delay(1000);  // Sensors need time?
  Serial.println("--- Chicken coop automation by Lee Woodridge ---");
  /*
  Serial.print(" > Checking for AHT20 temperature and humidity sensor ... ");
  if (!tempHumidSensor.begin()) {
    Serial.println("Not found! Program halted ");  // We should proceed with reduced functionality
    //while (1) delay(10);
  } else {
    Serial.println("Found!");
  }
*/
  pinMode(SwitchUpPin, INPUT_PULLUP);    // Coop door toggle switch up position pin
  pinMode(SwitchDownPin, INPUT_PULLUP);  // Coop door toggle switch down position pin
  pinMode(Relay1Pin, OUTPUT);            // Coop heater relay pin
  pinMode(Relay2Pin, OUTPUT);            // Water heater relay pin
  pinMode(LightSensor, INPUT);           // Light sensor pin

  // Get inital data from all sensors
  storedLight = analogRead(LightSensor);  // Get current light level
  getTempHumid();                         // Get current temperature & humidity*/

  doorPos = EEPROM.read(eAddress);  // Update doorPos with EEPROM door position
  Serial.print(" > The last known door position is ");
  switch (EEPROM.read(eAddress)) {
    case ' ':
      Serial.println("unknown");
      break;
    case 'U':
      Serial.println("up");
      break;
    case 'D':
      Serial.println("down");
      break;
  }
  Serial.println("--- Setup complete ---");
}

void loop() {
  unsigned long currentTime = millis();
  delay(1000);  // For debounce; How low can we go?

  if (currentTime - eventTimer1PreviousTime >= eventTimer1) {  // Event #1 : Record temp & humidity sensor data
    getTempHumid();
    Serial.println("--- Executed timed event #1 ---");
    Serial.print(" > Temperature: ");
    Serial.print(storedTemp);
    Serial.print("°C | ");
    /*Serial.print("Humidity: ");
    Serial.print(storedHumid);
    Serial.println("% RH");
    */
    eventTimer1PreviousTime = currentTime;
  }

  if (storedTemp <= thresholdTempRelay1) {  // If temperature is lower than threshold, turn on relays if not already
    if (digitalRead(Relay1Pin) != LOW) {
      digitalWrite(Relay1Pin, LOW);  // Turn on relay
    }
  } else {
    digitalWrite(Relay1Pin, HIGH);  // Turn off relay
  }

  if (storedTemp <= thresholdTempRelay2) {  // If temperature is lower than threshold, turn on relays if not already
    if (digitalRead(Relay2Pin) != LOW) {
      digitalWrite(Relay2Pin, LOW);  // Turn on relay
    }
  } else {
    digitalWrite(Relay1Pin, HIGH);  // Turn off relay
  }

  if (!digitalRead(SwitchDownPin) && digitalRead(SwitchUpPin)) {  // Switch is in up position
    switchPos = 'U';
    if (doorPos != 'U') { RaiseDoor(); }
  } else if (digitalRead(SwitchDownPin) && !digitalRead(SwitchUpPin)) {  // Switch is in down position
    switchPos = 'D';
    if (doorPos != 'D') { LowerDoor(); }
  } else {  // Switch is in neutral position

    if (switchPos != 'N') {
      Serial.println(" > Door switch in neutral position");
    }

    switchPos = 'N';

    if (currentTime - eventTimer2PreviousTime >= eventTimer2) {  // Event #2 : Check light sensor for door (We're only running this if door switch is in neutral position)
      Serial.println("--- Executed timed event #2 ---");         // Open door based on light level, with delay if under temp threshold

      storedLight = analogRead(LightSensor);  // Get the light level
      Serial.print(" > Light sensor value: ");
      Serial.print(storedLight);
      if (storedLight < 100) {
        Serial.println(" - Very bright");
      } else if (storedLight < 200) {
        Serial.println(" - Bright");
      } else if (storedLight < 500) {
        Serial.println(" - Light");
      } else if (storedLight < 800) {
        Serial.println(" - Dim");
      } else {
        Serial.println(" - Dark");
      }

      if (storedLight <= thresholdLightRaise) {  // Light is above threshold

        if (storedTemp <= thresholdTempDelay /*&& storedHumid != thresholdHumid*/) {  // Temperature is below threshold, open door with delay

          if (doorDelayStartTime = 0) {  // Preserve original counter start time if exists
            doorDelayStartTime = millis();
          }
          if (doorDelayStartTime >= doorDelay) {  // I THINK THIS WONT WORK---CHECK
            if (doorPos != 'U') { RaiseDoor(); }
            doorDelayStartTime = 0;  // Clear original start time
          }
        } else {  // Temperature is above threshold, open door
          if (doorPos != 'U') { RaiseDoor(); }
        }
        // ---
      } else if (storedLight >= thresholdLightLower) {  // Light is below threshold, close door
        if (doorPos != 'D') { LowerDoor(); }
      }
      eventTimer2PreviousTime = currentTime;
    }
  }
}

void RaiseDoor() {
  Serial.println("Raising door ...");
  // Motor go up
  doorPos = 'U';
  updateEEPROM();
}

void LowerDoor() {
  Serial.println("Lowering door ...");
  // Motor go down
  doorPos = 'D';
  updateEEPROM();
}

void updateEEPROM() {
  if (EEPROM.read(eAddress) != doorPos) {
    // EEPROM.write(eAddress, doorPos); // UNCOMMENT THIS --- ADDED TO PREVENT EEPROM WRITES SINCE THEY'RE LIMITED
    Serial.print("Door position of ");
    Serial.print(doorPos);
    Serial.print(" stored to EEPROM address ");
    Serial.println(eAddress);
  } else {
    Serial.println("No need to update door position in EEPROM");
  }
}

void getTempHumid() {
  /*
  sensors_event_t humidity, temp;
  tempHumidSensor.getEvent(&humidity, &temp);  // Populate temperature and humidity objects
  storedTemp = temp.temperature;
  storedHumid = humidity.relative_humidity;
  */
}