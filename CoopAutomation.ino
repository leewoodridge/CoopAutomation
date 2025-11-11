/*
NOTES: Switching to basic analog temp sensor, no humidity anymore
       Switching to basic analog light sensor
	   
	   - if door pos is unknown then open and reset, no need for eeprom writes
	   - powerout break light hrs count?
	   // Serial.println(digitalRead(PIN) == HIGH ? "ON" : "OFF"); // to report pin on or off
	   // int result = (a != 0) ? a : ((b != 0) ? b : 0); ;;; returns non zero value betwenn 2 vars or 0 if both 0

  // NEED TO MAKE SURE COOP LIGHT ISNT BEING PICKED UP BY SENSOR

// total daylight time record will be affected depending on when during day boots up
	   
	   // adj speed depending on temp for less wear on linact?
	   // use ramp too
	   // add delay between linear actuator movements
	   
*/
/* ⠀     ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⢀⡀
      ⠀ ⣠⣶⣿⠆⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⡎⢻⣽⢶⢀
       ⢀⣵⣔⢺⡆⠀⠑⠦⡀⠀⠀⠀⠀⠀⠀⠀⠀⣇⡀⠀⢉⠆⡜⡱⡄
       ⠀⠸⢈⡏⠀⠀⠀⢲⣮⣦⡀⠀⠀⠀⢀⠔⠊⠁⠀⠀⠹⡀⠴⠟⠽⠄
     ⠀  ⠀⣱⠀⣇⣆⣦⠑⢈⠁⠵⠒⠒⢞⡁⠀⠀⠀⠀⡀⠀⠀⠓⠿⣟⠄
     ⠀  ⠀⡎⠄⠿⠘⠚⠉⠀⠀⠀⠁⠀⠀⠀⠉⠑⠂⠔⢓⠠⠞⠀⢀⠇
     ⠀  ⠀⠣⡀⠀⠀⠀⠈⠒⢄⠀⠀⠀⠀⡐⢀⡇⠀⣀⣚⠀⢴⠀⡌
     ⠀  ⠀⠀⢳⡀⠀⠀⠀⠀⠀⠹⠆⡀⠪⠂⣊⠤⠒⠊⠙⠲⡤⠋
       ⠀⠀⠀⠀⠙⠦⡀⠀⠀⠀⠀⠀⡀⠓⠈⠀⠀⠀⠀⠒⢺⠃
⠀⠀⠀⠀       ⠀⠀⠈⢦⠀⠒⠂⠀⠀⠀⣱⣀⡀⠀⢘⡵⠁
⠀⠀⠀  ⠀     ⠀⠀⠀⠈⢣⡀⠀⠀⠀⠀⣻⣿⡿⠷⠊
⠀⠀       ⠀⠀⠀⠀⠀⠀⠀⠙⢢⢤⠤⠚⢶⡈
⠀       ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠂⠀⠀⠈⡧⡤⠚
       ⠀⠀⠀⠀⠀⠀⠠⣽⠷⠟⠓⠓⠀⠟⡎⠋
   ⣎⣱⡀⢀⣰⡀⢀⡀ ⡎⠑⣇⡀⠄⢀⣀⡇⡠⢀⡀⣀⡀ ⡎⠑⢀⡀⢀⡀⣀⡀
   ⠇⠸⠣⠼⠘⠤⠣⠜ ⠣⠔⠇⠸⠇⠣⠤⠏⠢⠣⠭⠇⠸ ⠣⠔⠣⠜⠣⠜⡧⠜
                by Lee Woodridge
*/

/*
unsigned long currentTime = 0; // Current time (milliseconds passed since boot)
*/

const int SwitchUpPin = 6;    // Coop door toggle switch up position pin
const int SwitchDownPin = 7;  // Coop door toggle switch down position pin

const int Relay1Pin = 8;  // Auxiliary light relay pin
const int Relay2Pin = 9;  // Water heater relay pin

const int LightSensor = 14;  // Light sensor pin (A0)
const int TempSensor = 15;   // Temp sensor pin (A1)

// for lin act
const int LinActPin1 = 10; // Linear actuator control pin 1
const int LinActPin2 = 11; // Linear actuator control pin 2
const int LinActENA = 12; // PWM pin for speed control

// lin act config
const uint16_t RAMP_MS = 600;        // accel/decel time (ms)
const uint16_t STEP_MS = 20;         // ramp step interval (ms)
const uint8_t PWM_MAX  = 200;        // cruise PWM (0-255)
const uint16_t MOVE_MS = 2500;       // estimated full travel time (ms) - adjust later
const uint8_t BRAKE_PWM = 120;       // optional reverse pulse strength
const uint16_t BRAKE_MS = 80;        // optional reverse pulse duration (ms)

// Internal state
uint8_t currentPwm = 0;

//

const unsigned long eventTimer1 = 10 * 60 * 1000;  // Event #1: Time interval to record temp & humidity sensor data (Minutes)
const unsigned long eventTimer2 = 10 * 60 * 1000;  // Event #2: Time interval to control door based on light level when switch is in neutral position (Minutes)

const unsigned long updatetimer = 5 * 60 * 1000;  // Minimum time between sensor updates

unsigned long eventTimer1PreviousTime = 0;  // millis for last operation
unsigned long eventTimer2PreviousTime = 0;
unsigned long lastdooropmillis = 0;

char doorPos = '?';    // Door position; '?'=Unknown, 'U'=Up, 'D'=Down
char switchPos = '?';  // Switch position; '?'=Unknown, 'U'=Up, 'N'=Neutral, 'D'=Down

const unsigned long doorDelay = 5 * 60 * 60 * 1000;  // Door opening delay for cold weather (Hours)
unsigned long doorDelayStartTime = 0;                // The time we started counting from, in case toggle position gets changed

unsigned long linactcooldown = 6 * 1000;  // Linear actuator cooldown time between operations (Seconds)

int storedLight = 0;                     // Last known light level (PUT RANGE HERE) Low=bright / high=dark
unsigned long lastlightcheckmillis = 0;  // last time light checked

const int thresholdLightLower = 000;  // Light level to lower door at (>=)
const int thresholdLightRaise = 000;  // Light level to raise door at (<=)

float storedTemp = 3.4028235E38;        // Last known temperature (°C) -- DO WE NEED FLOAT???
unsigned long lasttempcheckmillis = 0;  // last time temp checked

const int thresholdTempDelay = -10;  // Temperature to trigger door open delay (<=)
//const int thresholdTempRelay1 = -15;  // Temperature to trigger coop heater relay (<=) // THIS WILL BE LIGHTS
const int thresholdTempRelay2 = 0;  // Temperature to trigger water heater relay (<=)
/*
const int thresholdHumid = 999;       // UNUSED (!=)
*/

// for temp sensor math
const float Vcc = 5.0;      // set to 3.3 if using 3.3V
const int Rfixed = 10000;   // fixed resistor on module in ohms (commonly 10k)
const float beta = 3950.0;  // Beta value of thermistor (common: 3950)
const float T0 = 298.15;    // reference temp in Kelvin (25°C = 298.15K)
const float R0 = 10000.0;   // resistance at 25°C (ohms) — commonly 10k

// Night (and day) detection functionality

const int nightLightThreshold = 800;                             // Light level to assume night
const unsigned long requiredLightDuration = 4 * 60 * 60 * 1000;  // time at light level to assume night/day -- 4 hours in milliseconds
unsigned long totalDayLightTime = 0;                             // Total time light has been at threshold -- 16 hours (57600000 milliseconds)
unsigned long totalNightLightTime = 0;                           // Total time nighttime light level has been at threshold
bool DayEnded = false;                                           // Has the night light level been maintained for the required duration? -- set this to true? on detect high light level thresholdlightraise!!!
//int daysUptime = 0;                                              // Number of days of uptime, day flagged after 4 hrs of darkness

const unsigned long minUptime = 10 * 60 * 60 * 1000;  // minimum uptime millis() for hour-checking functionality (10hrs)

// Auxiliary light functionality

const int dayLightThreshold = 500;                                   // Light level to assume day
const unsigned long requiredDayLightDuration = 16 * 60 * 60 * 1000;  // target light hours (aux light will run until this time is met)
unsigned long supplementalLight = 0;                                 // how much time aux lighting has been on

// reporting

/*unsigned long lastrelay1opmillis = 0; // relay 1 operation -- aux light
unsigned long lastrelay2opmillis = 0; // relay 2 operation -- water heater*/

// End of declarations

/* SETUP
   ----- */

void setup() {
  Serial.begin(115200);
  delay(1000);  // Sensors need time?
  Serial.println("Auto Chicken Coop by Lee Woodridge");
  Serial.println("       ┌┌┌──────────────┐┐┐");
  Serial.println("       │││ INITIALIZING │││");
  Serial.println("       └└└──────────────┘┘┘");
  Serial.println(" - Configuring pin modes ... ");
  pinMode(SwitchUpPin, INPUT_PULLUP);    // Coop door toggle switch up position pin
  pinMode(SwitchDownPin, INPUT_PULLUP);  // Coop door toggle switch down position pin
  pinMode(Relay1Pin, OUTPUT);            // Aux light relay pin
  pinMode(Relay2Pin, OUTPUT);            // Water heater relay pin
  pinMode(LightSensor, INPUT);           // Light sensor pin
  pinMode(TempSensor, INPUT);            // Temp sensor pin
                                         /* lin act
  pinMode(LinActPin1, OUTPUT); // Linear actuator power pos
  pinMode(LinActPin2, OUTPUT); // Linear actuator pwoer neg
  pinMode(ENA, OUTPUT); // Linear actuator ENA pin (pwm speed control)
*/
  Serial.println(" - Configuring outputs to default states ... ");
  /*  
  // Start with the actuator stopped
  digitalWrite(LinActPin1, LOW); // Door actuator
  digitalWrite(LinActPin2, LOW); // ^
  */
  // turn off relays// set default relays off here
  relayControl(1, false);  // Auxiliary lighting
  relayControl(2, false);  // Water heater
  Serial.println(" - Configuring analog input reference voltage ... ");
  // analogReference(DEFAULT);

  /*// Get inital data from all sensors
  Serial.println(" - Gathering data from light & temp sensors ... ");
  getLight();
  getTemp();*/

  Serial.println("           ┌┌┌──────┐┐┐");  // Initialization complete
  Serial.println("           │││ BAWK │││");
  Serial.println("           └└└──────┘┘┘");
}

/* MAIN LOOP
   --------- */

void loop() {
  //currentTime = millis(); //unsigned long
  delay(1000);  // For debounce; How low can we go?

  if (millis() /*currentTime*/ - eventTimer1PreviousTime >= eventTimer1 || eventTimer1PreviousTime == 0) {  // Timed event #1
    getTemp();
    getLight();
    checkHeat();
    checkLight();
    Serial.println("--- Executed timed event #1 (gettemp,getlight,checkheat,checklight) ---");
    eventTimer1PreviousTime = /*currentTime*/ millis();
  }

  checkDoor();
  printInfo();

}  // END

/* DOOR CONTROL
   ------------
   Up and down switch positions will raise lower door if not already in that position, neutral switch position will enable automatic door control */

void checkDoor() {
  if (!digitalRead(SwitchDownPin) && digitalRead(SwitchUpPin) && switchPos != 'U') {  // Switch is in up position
    switchPos = 'U';
    if (doorPos != 'U') {
      actuatorControl(true);
    }
  } else if (digitalRead(SwitchDownPin) && !digitalRead(SwitchUpPin) && switchPos != 'D') {  // Switch is in down position
    switchPos = 'D';
    if (doorPos != 'D') {
      actuatorControl(false);
    }
  } else if (!digitalRead(SwitchDownPin) && !digitalRead(SwitchUpPin)) {  // Switch is in neutral position (auto)
    if (switchPos != 'N') {
      switchPos = 'N';
    }

    /* AUTOMATIC DOOR
	   --------------
       Open or close door based on light level, with delay if under temp threshold */

    if (millis() /*currentTime*/ - eventTimer2PreviousTime >= eventTimer2 || eventTimer2PreviousTime == 0) {  // Event #2
      Serial.println("--- Executed timed event #2 --- (auto door control)");
      /*
	  getLight();
	  getTemp();
	  */
	  
      if (storedLight <= thresholdLightRaise /*&& doorPos == 'D'*/) {                     // Light is above threshold, open door
        if (storedTemp <= thresholdTempDelay /*&& storedHumid != thresholdHumid*/) {  // Temperature is below threshold, open door with delay
          if (doorDelayStartTime == 0) {                                              // Only set start time if unset
            doorDelayStartTime = millis();
          }
          if (millis() - doorDelayStartTime >= doorDelay) {  // ! DOUBLE CHECK THIS IS CORRECT !
            actuatorControl(true);
            doorDelayStartTime = 0;  // Clear original start time
          }
        } else {
          actuatorControl(true);  // Temperature is above threshold, open door
        }
      } else if (storedLight >= thresholdLightLower /*&& doorPos == 'U'*/) {  // Light is below threshold, close door
        actuatorControl(false);
      }
      eventTimer2PreviousTime = /*currentTime*/ millis();
    }
    /* AUTOMATIC DOOR END
       ------------------ */
  }
}  // END

/* DISPLAY SYSTEM INFORMATION
   -------------------------- */

void printInfo() {  // SHOW SYSTEM INFO
  const char divStr[] = " | ";

  Serial.print("LIGHT: ");
  Serial.print(storedLight);  // add desc

  Serial.print("(Last: " + FormatMillis(millis() - lastlightcheckmillis, true) + ")");

  /*
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
  */

  Serial.print(divStr);

  Serial.print("TEMP: ");
  Serial.print(storedTemp, 2);
  Serial.print("°C");

  Serial.print("(Last: " + FormatMillis(millis() - lasttempcheckmillis, true) + ")"); // Last updated time

  Serial.print(divStr);

  Serial.print("AUXLIGHT: ");
  Serial.print(digitalRead(Relay1Pin) == LOW ? "On" + String(supplementalLight) + String(requiredDayLightDuration - totalDayLightTime) : "Off");
  //Serial.println("Auxiliary lighting ON (" + millisToHumanAll(requiredDayLightDuration - (totalDayLightTime + supplementalLight), true) + " remaining)");

  // display AUX 2/4" so far/tot

  //reqaux = requiredDayLightDuration - totalDayLightTime
  // supplementalLight

  Serial.print(divStr);

  Serial.print("AUXHEAT: ");
  Serial.print(digitalRead(Relay2Pin) == LOW ? "On" : "Off");

  Serial.print(divStr);

  Serial.print("DOOR/SW: ");

  switch (doorPos) {
    case 'U':
      Serial.print("Up");
      break;

    case 'D':
      Serial.print("Down");
      break;

    default:
      Serial.print("ERROR!");
      break;
  }

  switch (switchPos) {
    case 'U':
    case 'D':
      Serial.print("/Manual");
      break;

    case 'N':
      Serial.print("/Auto");
      break;

    default:
      Serial.print("/ERROR!");
      break;
  }

  // show info about cold doordelay

  Serial.print(divStr);

  Serial.print("DAYNIGHT: ");

  if (storedLight < nightLightThreshold) {
    // day
    Serial.print("Day (");
    Serial.print(totalDayLightTime);
    Serial.print(" hrs)");
  } else {
    // night
    Serial.print("Night (");
    Serial.print(totalNightLightTime);
    Serial.print(" hrs)");
  }

  Serial.print(divStr);

  //dayended yes no (time left -- 2/4hrs)
}  // END

/* GET LIGHT SENSOR DATA
   --------------------- */

void getLight() {
  if (millis() - lastlightcheckmillis <= updatetimer && lastlightcheckmillis != 0) { return; }  // Abort if updated recently
  const int samples = 8;
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(LightSensor);
    delay(5);  // Small pause for ADC settling / sensor stability
  }
  storedLight = sum / samples;
  lastlightcheckmillis = millis();
}  // END

/* GET TEMP SENSOR DATA
   -------------------- */

void getTemp() {
  if (millis() - lasttempcheckmillis <= updatetimer && lasttempcheckmillis != 0) { return; }  // Abort if updated recently
  const int samples = 8;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(TempSensor);
    delay(5);
  }
  float adc = float(sum) / samples;
  float voltage = adc * (Vcc / 1023.0);
  float Rtherm = Rfixed * voltage / (Vcc - voltage);
  float invT = 1.0 / T0 + (1.0 / beta) * log(Rtherm / R0);
  float tempK = 1.0 / invT;
  float tempC = tempK - 273.15;
  storedTemp = tempC;
  lasttempcheckmillis = millis();
}  // END

/* LIGHT RELAY CHECK
   ----------------- */

void checkLight() {
  if (storedLight > nightLightThreshold) {  // it's dark
    totalNightLightTime += eventTimer1;

    /* AUXILIARY LIGHTING
	   ------------------
	   If minimum light hours haven't been met, turn on auxiliary lighting */

    if (totalDayLightTime + supplementalLight < requiredDayLightDuration && millis() > minUptime && totalNightLightTime >= eventTimer1) {  // If total light hrs < req | If uptime > req | If dark time > 1 check cycle
      if (storedLight > dayLightThreshold) {                                                                                               // Check relay status instead?
        supplementalLight += eventTimer1;
        relayControl(1, true);
      }
    } else {
      relayControl(1, false);
    }

    /* AUXILIARY LIGHTING END
	   ---------------------- */

    if (totalNightLightTime >= requiredLightDuration /*&& !DayEnded*/) {  // Check if the light has been above the threshold for at least 4 hours // Do we need dayended?
      DayEnded = true;
      totalDayLightTime = 0;
      /*+daysUptime;*/
      Serial.println("Day has ended ─ Good night sleepy chickens");
    }
  } else {  // it's bright
    totalDayLightTime += eventTimer1;
    relayControl(1, false);  // Disable auxiliary light if bright ─ this shouldnt be needed, but just in case // add 2 check cycle req???
    if (totalDayLightTime >= requiredLightDuration /*&& DayEnded*/) {
      DayEnded = false;
      totalNightLightTime = 0;
      supplementalLight = 0;
      Serial.println("Day has begun ─ Good morning chickens");
    }
  }
}  // END

/* HEATER RELAY CHECK
   ------------------
   Toggle heater relay based on temperature */

void checkHeat() {
  if (storedTemp <= thresholdTempRelay2) {
    relayControl(2, true);
  } else {
    relayControl(2, false);
  }
}  // END /* void checkHeat() { relayControl(2, storedTemp <= thresholdTempRelay2); } */

/* LIGHT & HEAT RELAY CONTROL
   --------------------------
   Turns on or off relays if not already in that state
*/

void relayControl(uint8_t relayNum, bool b) {
  if (relayNum < 1 || relayNum > 2) {
    /*
    Serial.print("Invalid relay: ");
    Serial.println(relayNum);
*/
    return;
  }
  int desiredState = b ? LOW : HIGH;
  int pin = (relayNum == 1) ? Relay1Pin : Relay2Pin;
  if (digitalRead(pin) == desiredState) return;
  digitalWrite(pin, desiredState);
  Serial.println(String((relayNum == 1) ? "Auxiliary lighting" : "Auxiliary heater") + " " + (b ? "ON" : "OFF"));
}  // END

/* DOOR ACTUATOR CONTROL
   ---------------------
   Raise or lower door if not already in that state, enforce cooldown period between operations, RAMP, timeout ETC
*/

void actuatorControl(bool b) {
  if (millis() - lastdooropmillis >= linactcooldown || lastdooropmillis == 0) {

    if (b) {
      if (doorPos != 'U') {  // raise
        Serial.println("Raising door ...");
        // Motor go up
		
    digitalWrite(LinActPin1, LOW);
    digitalWrite(LinActPin2, HIGH);
		
        doorPos = 'U';
      } else {
        Serial.println("door already up!");  // Don't need these
      }
    } else {
      if (doorPos != 'D') {  // lower
        Serial.println("Lowering door ...");
        // Motor go down
		
    digitalWrite(LinActPin1, HIGH);
    digitalWrite(LinActPin2, LOW);
		
        doorPos = 'D';
      } else {
        Serial.println("door already down!");
      }
    }


	
  // Ramp up
  rampTo(PWM_MAX, RAMP_MS);

  // Cruise for MOVE_MS (time-based)
  unsigned long start = millis();
  while (millis() - start < MOVE_MS) {
    delay(20); // keep loop responsive
  }

  // Ramp down
  rampTo(0, RAMP_MS);

  // Optional gentle reverse pulse to settle
  pulseBrake(b); // this direction might not be right -- do we need this?

    // Stop the actuator
    //wait (calc approx how long to raise lower + 10%) will heat/cold affect?
    //digitalWrite(LinActPin1, LOW);
    //digitalWrite(LinActPin2, LOW);
    //analogWrite(ENA, 0); // Set speed to 0

    lastdooropmillis = millis();
  } else {
    Serial.println("linear actuator in cooldown mode, waiting");  // show time remaining
  }
}  // END

float easeCubic(float t) {
  if (t < 0.5) return 4.0 * t * t * t;
  float f = (2.0 * t) - 2.0;
  return 0.5 * f * f * f + 1.0;
}

void rampTo(uint8_t targetPwm, uint16_t durationMs) {
  uint8_t startPwm = currentPwm;
  if (durationMs == 0) {
    currentPwm = targetPwm;
    analogWrite(LinActENA, currentPwm);
    return;
  }
  uint16_t steps = max(1, durationMs / STEP_MS);
  unsigned long t0 = millis();
  for (uint16_t i = 1; i <= steps; ++i) {
    float tt = (float)i / (float)steps;
    float e = easeCubic(tt);
    float pwmF = startPwm + (targetPwm - startPwm) * e;
    uint8_t pwmNow = (uint8_t)round(constrain(pwmF, 0, 255));
    if (pwmNow != currentPwm) {
      currentPwm = pwmNow;
      analogWrite(LinActENA, currentPwm);
    }
    unsigned long next = t0 + i * STEP_MS;
    while (millis() < next) { /* wait */ }
  }
}

void pulseBrake(bool wasExtend) {
  // reverse briefly then stop
  if (wasExtend) {
    digitalWrite(LinActPin1, LOW);
    digitalWrite(LinActPin2, HIGH);
  } else {
    digitalWrite(LinActPin1, HIGH);
    digitalWrite(LinActPin2, LOW);
  }
  analogWrite(LinActENA, BRAKE_PWM);
  delay(BRAKE_MS);
  analogWrite(LinActENA, 0);
  digitalWrite(LinActPin1, LOW);
  digitalWrite(LinActPin2, LOW);
  currentPwm = 0;
}

/* TIME FORMATTING
   -------------------------------------
   Convert millis to human readable time */

struct TimeParts {
  unsigned long days;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  unsigned long milliseconds;
};

String FormatMillis(unsigned long ms, bool shortForm) {
  if (ms == 0) return shortForm ? "0ms" : "0 Milliseconds";

  TimeParts t;
  t.milliseconds = ms % 1000UL;
  unsigned long totalSeconds = ms / 1000UL;
  t.seconds = totalSeconds % 60UL;
  unsigned long totalMinutes = totalSeconds / 60UL;
  t.minutes = totalMinutes % 60UL;
  unsigned long totalHours = totalMinutes / 60UL;
  t.hours = totalHours % 24UL;
  t.days = totalHours / 24UL;

  String out;
  if (shortForm) {
    if (t.days) out += String(t.days) + "d";
    if (t.hours) out += String(t.hours) + "h";
    if (t.minutes) out += String(t.minutes) + "m";
    if (t.seconds) out += String(t.seconds) + "s";
    if (t.milliseconds) out += String(t.milliseconds) + "ms";
  } else {
    bool first = true;
    auto addLong = [&](unsigned long v, const char* sing, const char* plur) {
      if (!v) return;
      if (!first) out += " ";
      out += String(v) + " " + String(v == 1 ? sing : plur);
      first = false;
    };
    addLong(t.days, "Day", "Days");
    addLong(t.hours, "Hour", "Hours");
    addLong(t.minutes, "Minute", "Minutes");
    addLong(t.seconds, "Second", "Seconds");
    addLong(t.milliseconds, "Millisecond", "Milliseconds");
  }

  return out;
}  // END

// THE END