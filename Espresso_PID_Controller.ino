#include <max6675.h>
#include <TimerOne.h>
#include <Encoder.h>

/**
 * Espress PID controller firmware prototype
 * Luke Zambella
 */

// PID coefficients
int kp = 20;
int ki = 5;
int kd = 0;

// Pin setup
// MAX6675
int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 5;

int relayPWM = 4;

// Encoders
int encoderA = 6;
int encoderB = 7;

// LEDs
int amberLED = 15;     // Temperature ready (+- 3 degrees) LED
int overshootLED = 16; // Temperature exceeds 10 degrees of target

// Display
int BCD_A = 8;
int BCD_B = 9;
int BCD_C = 10;
int BCD_D = 11;

int charA = 12;
int charB = 13;
int charC = 14;

// Application specific globals
int desiredTemp = 87;
int currentTemp = 0;
int cutoffTemp = 180;   // Global cutoff temperature

int integral = 0;
int lastError = 0;

int pidMin = 0;
int pidMax = 1023;
int pwmPeriod = 3000000; // microseconds

int currentLEDOut = 0; // index of current 7-seg led
int currentMS = 0;

int currentTempDigit = 0; //0 1 or 2
long lastEnc = -999;

long lastDebounceTime = 0;
long debounceDelay = 100;
int tempPeriod = 1000;

// Timer values
int displayTimerCurrent = 0;    // Set all led outputs to low
int desiredTempTimerCurrent = 0;  // Run when user changes temperature so display desired instead of current
bool isShowDesired = false;

MAX6675 thermoCouple(thermoCLK, thermoCS, thermoDO);
Encoder tempCtrl(encoderA, encoderB);

void setup() {
  // Initialize PWM timer
  pinMode(relayPWM, OUTPUT);
  Timer1.initialize(pwmPeriod);
  Timer1.pwm(relayPWM, 0);
  
  Serial.begin(9600);
  Serial.print("Espresso PID Controller 0.0.1\n");
  // Init LEDs
  pinMode(amberLED, OUTPUT);
  pinMode(overshootLED, OUTPUT);

  // Init BCD 7-seg
  pinMode(BCD_A, OUTPUT);
  pinMode(BCD_B, OUTPUT);
  pinMode(BCD_C, OUTPUT);
  pinMode(BCD_D, OUTPUT);
  
  pinMode(charA, OUTPUT);
  pinMode(charB, OUTPUT);
  pinMode(charC, OUTPUT);
}

void loop() {
  int pid = 0;
  int error = 0;
  
  // Check if desired temp is out of range
  if (desiredTemp >= cutoffTemp - 10) {
    desiredTemp = cutoffTemp - 10;
  } else if (desiredTemp <= 0) {
    desiredTemp = 0;
  }
  
    // If temp exceeds safe temperature then keep the device off
  if (currentTemp >= cutoffTemp || currentTemp >= desiredTemp + 15) {
    pid = 0;
  } else {
    // Calculate the error
    error = desiredTemp - currentTemp;
    // get the PID value
    pid = calculatePid(error);
  }
  
  // set the PWM output
  Timer1.setPwmDuty(relayPWM, pid);
  // Do things every 250ms
  if (currentMS >= tempPeriod) {
    currentTemp = thermoCouple.readCelsius();
    Serial.print("TEMPERATURE :: ");
    Serial.print(currentTemp);
    Serial.print("\n");
    currentMS = 0;
  } else if (currentMS / 5 == 0){
    currentTempDigit++;
    if (currentTempDigit >= 3) {
      currentTempDigit = 0;
    }
  }
  
  /** 
   *  Handle user interface
   */

  /*
   * Every 10ms print out the next LED value
   */
  if (displayTimerCurrent >= 10) {
    printTemperature(currentTempDigit);
    displayTimerCurrent = 0;
  } else {
    displayTimerCurrent++;
  }
  /*
   * Read the dial input
   * If the dial was turned, switch to showing desired temperature for 1 second
   */
  if ( (millis() - lastDebounceTime) > debounceDelay) {
    long enc = tempCtrl.read();
    if (lastEnc > enc) {
      desiredTemp--;
      lastEnc = enc;
      isShowDesired = true;
    } else if (lastEnc < enc) {
      isShowDesired = true;
      desiredTemp++;
      lastEnc = enc;
    }
    lastDebounceTime = millis(); //set the current time
  }
  // If temperature is within 3 degrees of target temp, keep amber led on
  if (currentTemp > desiredTemp + 3 && currentTemp <= desiredTemp +3) {
    digitalWrite(amberLED, HIGH);
  } else {
    digitalWrite(amberLED, LOW);
  }
  // If temperature is 10 degrees over target turn red LED on and amber LED off

  // Increment values
  currentMS++;
  currentTempDigit++;
  if (currentTempDigit >= 3) {
    currentTempDigit = 0;
  }

  // For viewing the desired temp vs current temp
  if (isShowDesired) {
    desiredTempTimerCurrent++;
    if (desiredTempTimerCurrent >= 1000) {
      isShowDesired = false;
      desiredTempTimerCurrent = 0;
    }
  }
  // Lock execution of the program to 1 millis
  delay(1);
}


/**
 * Calculates a PID value as an integer of a PWM duty cycle
 */
int calculatePid(int error) {
  // Get itnegral and derivative
  integral = integral + error;
  int derivative = error - lastError;
  
  // Calculate the pid value
  int pid = (kp * error) + (ki * integral) + (kd * derivative);

  // Set the pid boundries
  if (pid >= pidMax)
    pid = pidMax;
  else if (pid <= pidMin)
    pid = pidMin;

  lastError = error;

  return pid;
}

/**
 * Prints the desired temperature to a BCD 7447 7 seg display
 */
void printTemperature(int digitNum) {
  unsigned int d = 0;
  int temp = 0;
  if (isShowDesired) {
    temp = desiredTemp;
  } else {
    temp = currentTemp;
  }
  // Enable the right digit
  if (digitNum == 0) {
    d = getDigit(temp, 0);
    digitalWrite(charA, HIGH);
    digitalWrite(charB, LOW);
    digitalWrite(charC, LOW);
  } else if (digitNum == 1) {
    d = getDigit(temp, 1);
    digitalWrite(charA, LOW);
    digitalWrite(charB, HIGH);
    digitalWrite(chatC, LOW);
  } else if (temp == 2) {
    d = getDigit(desiredTemp, 2);
    digitalWrite(charA, LOW);
    digitalWrite(charB, LOW);
    digitalWrite(charC, HIGH);
  }
  
  // Write the binary equivalent to the ABCD decoder pins
  // 9 -> 0b1001
  //        DCBA
  //0b0100
  //
  digitalWrite(BCD_A, d & 0b0001 == 1 ? HIGH : LOW);
  d = d >> 1; // 0b0100
  digitalWrite(BCD_B, d & 0b0001 == 1 ? HIGH : LOW);
  d = d >> 1; // 0b0010
  digitalWrite(BCD_C, d & 0b0001 == 1 ? HIGH : LOW);
  d = d >> 1; // 0b0001
  digitalWrite(BCD_D, d & 0b0001 == 1 ? HIGH : LOW);
}

int getDigit(int num, int n)
{
  int r;
  r = num / pow(10, n);
  r = r % 10;
  return r;
}
