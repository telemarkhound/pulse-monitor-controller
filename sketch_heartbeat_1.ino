#include <ListLib.h>

// Globals
int ANALOG_IN = A0;

int NOMINAL = 370;                    // Typical sensor voltage when no pulse is present
int NOMINAL_BAND = 30;                // A buffer around NOMINAL to allow for noise and false indications
int MAX_MIN_BAND = 20;                // A different tolerance for detecting max/min vector changes
int CURRENT_VALUE = NOMINAL;          // Current value of the sensor, ranging from 0 to 1023
int CURRENT_BAND = 0;                 // BELOW = -1 | NOMINAL = 0 | ABOVE = 1
int LAST_MIN = 415;                   // The lowest value observed since the last vector change
int LAST_MAX = 415;                   // The highest value observed since the last vector change
String CURRENT_VECTOR = "nn";         // State change indicator using -,n,+   
                                      // For example, n+ indicates "From Nominal to Above" aka "Moving Up"

unsigned long INIT_TIME = millis();                 // Reference timestamp
unsigned long LAST_STATE_CHANGE = millis();         // For tracking time period that triggers a reset & init
unsigned long STATE_CHANGE_INIT_THRESHOLD = 4000;   // How long to wait without a state change before re-initializing

String VALVES[4];     // State of Valves 0 = Null 1 = Left 2 = Right 3 = Top

//List<int> PEAKS;          // The sensor voltage values of detected peaks
//List<int> PEAK_TIMES;     // Timings (ms) of the peak values, aligned by index

void setup() {  
  // Init the serial mode for reading the easypulse board's signal (heartbeat)
  Serial.begin(9600);

  // Setup the solenoid signals for each 
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);

  //VALVES = new List<bool>();
  //VALVES.Add(false);
  //VALVES.Add(false);
  //VALVES.Add(false);
  //VALVES.Add(false);
  VALVES[0] = "";
  VALVES[1] = "100";
  VALVES[2] = "100";
  VALVES[3] = "100";

  // Algorithm is based on tracking state the is a permutation of two elements:
  // BAND: The current value in 1 of three bands, defined by the Nominal Band, which is an average instrument value, allowing for a buffer up or down. If it''s not in the nominal band, it must be above or below it. 
  //    Nominal
  //    Below
  //    Above
  // VECTOR: The direction of change when a transition between bands occurs (1st Derivative. Well, sort of) 
  //  Expressed as a pair of values in sequence indicating a direction but no magnitude
  //    n = Nominal
  //    + = Above Nominal
  //    - = Below Nominal 
  //
  //  By tracking both of these elemnets, we can identify the 11 key points of a heartbeat through a combination of unique states and a few sequences involving duplicate states that occur

  Serial.println();
  Serial.println("starting setup()...");
  resetState();
  waitForNominal();
  Serial.println("setup() complete");
}

void loop() {
  // Check for bad state 
  if ( CURRENT_BAND != 0
    && millis() - LAST_STATE_CHANGE > STATE_CHANGE_INIT_THRESHOLD ) {
    resetState();
  }

  getCurrentState();
}

void resetState(){
  Serial.println("resetting state");

  CURRENT_VALUE = NOMINAL;
  CURRENT_BAND = 0;
  CURRENT_VECTOR = "nn"; 
  LAST_MIN = NOMINAL;
  LAST_MAX = NOMINAL; 
  LAST_STATE_CHANGE = millis();

  // Be safe and close all valves :)
  valveClose(1);
  valveClose(2);
  valveClose(3);
}

void waitForNominal() {
      // spin wait until value falls into nominal range
      Serial.println("waiting for nominal state");
      CURRENT_VALUE = analogRead(ANALOG_IN);

      while( CURRENT_VALUE > NOMINAL + NOMINAL_BAND 
          || CURRENT_VALUE < NOMINAL - NOMINAL_BAND  ) {
        CURRENT_VALUE = analogRead(ANALOG_IN);
        Serial.println(CURRENT_VALUE);
      }

      // Set nominal
      NOMINAL = CURRENT_VALUE;
      NOMINAL_BAND = CURRENT_VALUE / 20;
      Serial.println("nominal band detected at " + String(CURRENT_VALUE));
}

// Moves the current band of the vector into the first position and adds a new state
void incrementVector(String newState) {
    String newVector = CURRENT_VECTOR[1] + newState;
    CURRENT_VECTOR = newVector; 
    LAST_STATE_CHANGE = millis();

    Serial.println( String(LAST_STATE_CHANGE) +","+ CURRENT_VALUE + "," + CURRENT_VECTOR +","+ LAST_MIN +","+ LAST_MAX +","+ VALVES[1] +","+ VALVES[2] +","+ VALVES[3] );
}

// Sets State: 
// CURRENT_VALUE, CURRENT_BAND, and CURRENT_VECTOR
// Returns true if state changed, false if it did not
bool getCurrentState() {
    CURRENT_VALUE = analogRead(ANALOG_IN);

    // Track max/min
    if ( CURRENT_VALUE < LAST_MIN ){
      LAST_MIN = CURRENT_VALUE;
    }
    if ( CURRENT_VALUE > LAST_MAX ){
      LAST_MAX = CURRENT_VALUE;
    }

    // Detect inter-band vector changes
    // In a sense, there's always a band around the LAST_MIN and LAST_MAX 
    // We'll use the same NOMINAL_BAND as a damper for noise

    // Vector change from moving up to moving down when ABOVE the nominal band
    if ( CURRENT_BAND == 1 && CURRENT_VECTOR == "n+" ) { // CURRENT_VECTOR.endsWith("+") ) {
      if ( CURRENT_VALUE < LAST_MAX - MAX_MIN_BAND ){
        LAST_MIN = CURRENT_VALUE;
        incrementVector("-");

        // Fire Right
        valveClose(1);
        valveOpen(2);
        valveClose(3);

        return true;
        }
    }

    // Vector change from moving down to moving up when BELOW nominal band
    if ( CURRENT_BAND == -1 && CURRENT_VECTOR == "n-" ) { // CURRENT_VECTOR.endsWith("-") ) {
      if ( CURRENT_VALUE > LAST_MIN + MAX_MIN_BAND ){
        LAST_MAX = CURRENT_VALUE;
        incrementVector("+");

        // Fire Top
        valveClose(1);
        valveClose(2);
        valveOpen(3);

        return true;
        }
    }

    // State Change to Nominal Band
    if ( CURRENT_VALUE > ( NOMINAL - NOMINAL_BAND ) 
      && CURRENT_VALUE < ( NOMINAL + NOMINAL_BAND ) ) {
      if ( !CURRENT_VECTOR.endsWith("n") ) {
        CURRENT_BAND = 0; 
        LAST_MIN = min(NOMINAL, LAST_MAX);
        LAST_MAX = max(NOMINAL, LAST_MIN);
        incrementVector("n");
        return true;
      }
    }

    // State Change to Above Band
    if ( CURRENT_VALUE > NOMINAL + NOMINAL_BAND ) {
      if ( CURRENT_BAND == 0 && !CURRENT_VECTOR.endsWith("+") ) {
        CURRENT_BAND = 1; 
        incrementVector("+");

        // Fire Left
        valveOpen(1);
        valveClose(2);
        valveClose(3);
        
        return true;
      }
    }

    // State Change to Below Band
    if ( CURRENT_VALUE < NOMINAL - NOMINAL_BAND ) {
      if ( CURRENT_BAND == 0 && !CURRENT_VECTOR.endsWith("-") ) {
        CURRENT_BAND = -1; 
        incrementVector("-");
        
        // Shoot out the top only
        valveClose(1);
        valveClose(2);
        valveOpen(3);

        return true;
      }
    }

    // Nothing changed
    return false;
}

void valveOpen(int valve){
  if (1 <= valve <=3 ){
    digitalWrite(valve,HIGH);
    //VALVES.Replace(valve, true);
    VALVES[valve] = "500";
  }
}

void valveClose(int valve){
  if (1 <= valve <=3 
    && digitalRead(valve) == HIGH ){
    digitalWrite(valve,LOW);
    //VALVES.Replace(valve, false);
    VALVES[valve] = "100";
  }
}
