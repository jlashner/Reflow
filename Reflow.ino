#include <LiquidCrystal.h>

#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

const char* lcdMessagesReflowState[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Wait,hot",
  "Error"
};

const char* lcdMessagesReflowStatus[] = {
  "OFF", "ON"
};

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;


bool button_ready = true;

typedef  enum SWITCH
{
    SWITCH_NONE,
    SWITCH_1,
    SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 190
#define TEMPERATURE_REFLOW_MAX 250
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** PIN ASSIGNMENT *****
int thermoPin = A5;

int ssrPin = 13;
int ledPin = 2;

int buttonPin = 2;
int buttonHoldThresh = 1000;
int buttonHoldCount = 0;
bool buttonReady = true;



// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

bool start_reflow = false;

const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


double readCelsius(){
  int max_adc = 0;
  int rdg, avg;

  for (int i = 0; i < 100; i++){
    rdg = analogRead(thermoPin);
//    Serial.print(rdg);
//    Serial.print(", ");
    max_adc = max(max_adc, rdg);
  }
//  Serial.print('\n');
  float voltage = float(max_adc) / 1024.0 * 5; //[V]
//  Serial.println(max_adc);
  return voltage * 100;
}


void setup() {
  Serial.begin(57600);

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);


  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledPin, LOW);
  pinMode(ledPin, OUTPUT);
  pinMode(thermoPin, INPUT);

  pinMode(buttonPin, INPUT);           // set pin to input
  digitalWrite(buttonPin, HIGH);       // turn on pullup resistors


  lcd.begin(16, 2);
  lcd.print("hello, world!");

  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();

  reflowStatus = REFLOW_STATUS_ON;
}

void loop() {
  unsigned long now;

  if (millis() > nextRead){
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;

    input  = readCelsius();
    lcd.clear();
    lcd.print(input);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print(lcdMessagesReflowStatus[reflowStatus]);
    lcd.print(", ");
    lcd.print(lcdMessagesReflowState[reflowState]);

//    Serial.print("Input: ");
//    Serial.print(input);
//    Serial.println("C");
  }

  if (millis() > nextCheck){
    // Check input in the next seconds
    nextCheck += 1000;

    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));

      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial

//      Serial.print(timerSeconds);
//      Serial.print(" ");
//      Serial.print(setpoint);
//      Serial.print(" ");
//      Serial.print(input);
//      Serial.print(" ");
//      Serial.println(output);

    } else{
      // Turn off red LED
      digitalWrite(ledPin, HIGH);
    }
  }

  switch (reflowState){
    case REFLOW_STATE_IDLE:
      if (input >= TEMPERATURE_ROOM){
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else{
        // Put logic here to start Reflow process on button press
        if (start_reflow){
          // Send header for CSV file
          Serial.println("Time Setpoint Input Output");
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          Serial.println("Setting REFLOW state to PREHEAT");
          reflowState = REFLOW_STATE_PREHEAT;
          start_reflow = false;
        }
      }
    break;

    case REFLOW_STATE_PREHEAT:
        reflowStatus = REFLOW_STATUS_ON;
        // If minimum soak temperature is achieve
        if (input >= TEMPERATURE_SOAK_MIN)
        {
          // Chop soaking period into smaller sub-period
          timerSoak = millis() + SOAK_MICRO_PERIOD;
          // Set less agressive PID parameters for soaking ramp
          reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
          // Ramp up to first section of soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
          // Proceed to soaking state
          reflowState = REFLOW_STATE_SOAK;
        }
    break;

    case REFLOW_STATE_SOAK:
        // If micro soak temperature is achieved
        if (millis() > timerSoak)
        {
          timerSoak = millis() + SOAK_MICRO_PERIOD;
          // Increment micro setpoint
          setpoint += SOAK_TEMPERATURE_STEP;
          if (setpoint > TEMPERATURE_SOAK_MAX)
          {
            // Set agressive PID parameters for reflow ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            // Ramp up to first section of soaking temperature
            setpoint = TEMPERATURE_REFLOW_MAX;
            // Proceed to reflowing state
            reflowState = REFLOW_STATE_REFLOW;
          }
        }
    break;

    case REFLOW_STATE_REFLOW:
        // We need to avoid hovering at peak temperature for too long
        // Crude method that works like a charm and safe for the components
        if (input >= (TEMPERATURE_REFLOW_MAX - 5))
        {
          // Set PID parameters for cooling ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp down to minimum cooling temperature
          setpoint = TEMPERATURE_COOL_MIN;
          // Proceed to cooling state
          reflowState = REFLOW_STATE_COOL;
        }
    break;

    case REFLOW_STATE_COOL:
        // If minimum cool temperature is achieve
        if (input <= TEMPERATURE_COOL_MIN)
        {
          // Retrieve current time for buzzer usage
          buzzerPeriod = millis() + 1000;
          // Turn on buzzer and green LED to indicate completion
//          #ifdef  USE_MAX6675
//            digitalWrite(ledGreenPin, LOW);
//          #endif
//          digitalWrite(buzzerPin, HIGH);
          // Turn off reflow process
          reflowStatus = REFLOW_STATUS_OFF;
          // Proceed to reflow Completion state
          reflowState = REFLOW_STATE_COMPLETE;
        }
    break;

    case REFLOW_STATE_COMPLETE:
        if (millis() > buzzerPeriod)
        {
//          // Turn off buzzer and green LED
//          digitalWrite(buzzerPin, LOW);
//          #ifdef  USE_MAX6675
//            digitalWrite(ledGreenPin, HIGH);
//          #endif
          // Reflow process ended
//          reflowState = REFLOW_STATE_IDLE;
        }
    break;

    case REFLOW_STATE_TOO_HOT:
        // If oven temperature drops below room temperature
        if (input < TEMPERATURE_ROOM)
        {
          // Ready to reflow
          reflowState = REFLOW_STATE_IDLE;

        }
    break;

    case REFLOW_STATE_ERROR:
        // If thermocouple problem is still present
//        #ifdef  USE_MAX6675
//          if (isnan(input))
//        #else
//          if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) ||
//             (input == FAULT_SHORT_VCC))
//        #endif
//        {
//          // Wait until thermocouple wire is connected
//          reflowState = REFLOW_STATE_ERROR;
//        }
//        else
//        {
//          // Clear to perform reflow process
//          reflowState = REFLOW_STATE_IDLE;
//        }
    break;
  }




  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)) {
//      Serial.println("SSR PIN on");
      digitalWrite(ssrPin, HIGH);
    }
    else {
//      Serial.println("SSR PIN off");
      digitalWrite(ssrPin, LOW);
    }

  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }

  Serial.println(digitalRead(buttonPin));
  if (!digitalRead(buttonPin)){
    buttonHoldCount += 1;
    if (buttonHoldCount >= buttonHoldThresh){

      start_reflow = true;
    }
  } else{
    buttonHoldCount = 0;
    buttonReady = true;
  }





//  delay(1000);
//  Serial.println("Starting Reflow Process");

//     // If switch 1 is pressed
//  if (switchStatus == SWITCH_1)
//  {
//    // If currently reflow process is on going
//    if (reflowStatus == REFLOW_STATUS_ON)
//    {
//      // Button press is for cancelling
//      // Turn off reflow process
//      reflowStatus = REFLOW_STATUS_OFF;
//      // Reinitialize state machine
//      reflowState = REFLOW_STATE_IDLE;
//    }
  }
