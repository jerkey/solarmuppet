#define BATTVOLTCOEFF 18.31 // calibrates perception of battery voltage
#define SOLARVOLTCOEFF 9.666 // calibrates perception of battery voltage
#define LOWVOLTAGEDISCONNECT 10.5 
#define LOWVOLTAGEDISCONNECTPIN 13
#define BATTFLOATVOLTAGE 14.1
// 10v = .457v at pin1 = vsolar 20k/1k ohm
// 10v = .927v at pin18 = vbatt?
// ground at pin5,  5v at pin 14
#define BUCKPIN 3 // which pin controls buck-converter transistors
// only use pin 3, 9, 10, or 11.  using 5 or 6 messes up millis() timing
#define SOLARDISCONNECTPIN 2 // turns on transistor linking solar- to ground, or relay
#define BATTVOLTPIN A0 // which pin reports battery voltage from resistors
#define SOLARVOLTPIN A1 // which pin reports solar panel voltage from resistors

#define BUCK_CUTIN 5.0  // minimum voltage to turn on transistors
#define BUCK_PERIOD 250 // milliseconds between buck converter pwm updates
#define SOLAR_CUTIN 13.0 // voltage above which the solar panel is useful
#define DISPLAY_PERIOD 750  // how many milliseconds between printdisplay()s
#define AVG_CYCLES 40  // cycles of averaging function
#define MAXBUCK 254 // maximum pwm value for buck converter

#define BAUDRATE 9600 // serial baud rate for communications with world

float battVoltage = 0;  // voltage of battery
float solarVoltage = 0;  // voltage at panel
int battAverageADC, lastBattAverageADC = 0;  // average ADC value of battvoltpin
int solarAverageADC = 0;  // average ADC value of solarvoltpin
unsigned long timeNow, lastBuck, lastDisplay = 0; // to hold system times
float buckDirection;  // amount to change PWM of buck converter while hunting
int lastBuckPWM = 0; // used to make sure we only analogWrite when integer changes
float buckPWM = 0.0;  // float to become PWM value of buck converter
float buckJump = 2.0;  // how much to change buckPWM per doBuck() cycle

void setup() {
  setPwmFrequency(BUCKPIN,1); // this sets the frequency of PWM on pins 3 and 11 to 31,250 Hz
  Serial.begin(BAUDRATE);
  pinMode(BUCKPIN,OUTPUT);  // connects to high-side transistors (Drain to Solar plus)
  pinMode(SOLARDISCONNECTPIN,OUTPUT);  // this must be high to tie solar- to ground when charging
  getVoltages();  // get initial voltage measurements
}

void loop() {
  timeNow = millis();  // get system time for this loop
  lastBattAverageADC = battAverageADC;  // save previous battery voltage for comparison
  getVoltages();  // update voltage measurements
  digitalWrite(LOWVOLTAGEDISCONNECTPIN, (battVoltage > LOWVOLTAGEDISCONNECT));
  doBuck();  // update buck converter PWM value
  if (timeNow - lastDisplay > DISPLAY_PERIOD) {
    printDisplay();  // send serial information to the world
    lastDisplay = timeNow;
  }
}

void doBuck() {
  if ((battVoltage > BUCK_CUTIN) &&  (solarVoltage > SOLAR_CUTIN)) {
    if (timeNow - lastBuck > BUCK_PERIOD) { // if it has been long enough since last time
      lastBuck = timeNow;  // we are doing it now
      if (!buckPWM) {  // the sun just came up
        digitalWrite(SOLARDISCONNECTPIN,HIGH);  // turn on minus-side FET
        buckDirection = buckJump; // we are going to track up now
        buckPWM = buckJump; // start with an initial PWM value of one buckJump
        setPWM(buckPWM);  // set the PWM value
      } 
      else { // keep hunting for the best PWM value for maximum batt. voltage
        if (battAverageADC < lastBattAverageADC) buckDirection *= -1;  // if voltage went down, reverse PWM hunting direction
        buckPWM += buckDirection; // hunt in whatever direction we are trying now
	if ((battVoltage > BATTFLOATVOLTAGE) && (buckDirection > 0)) buckDirection *= -1;
        setPWM(buckPWM);  // set the PWM value
        if (buckPWM > MAXBUCK) buckPWM = MAXBUCK;
        if (buckPWM < 1) {        
          buckDirection = buckJump; // we are going to track up now
          buckPWM = buckJump; // start with an initial PWM value of one buckJump
        } // if (buckPWM < 1)
      } // else
    } // if (timeNow - lastBuck > BUCK_PERIOD)
  }  // if (battVoltage > BUCK_CUTIN)
  else { // there is not enough power to be charging right now
    digitalWrite(SOLARDISCONNECTPIN,LOW); // dont want solar panel draining batts at night
    buckPWM = 0;  // turn off high side FET
    setPWM(buckPWM);
  }
}

void setPWM(float pwmVal) { // only do the analogWrite if integer value has actually changed
  if ((int)pwmVal != lastBuckPWM) {
    lastBuckPWM = (int)pwmVal;
    analogWrite(BUCKPIN,lastBuckPWM);
  }
}

void printDisplay() {
  Serial.print("battery: ");
  Serial.print(battVoltage);
  Serial.print(" (");
  Serial.print((float)battAverageADC / (float)AVG_CYCLES);
  Serial.print(") solar: ");
  Serial.print(solarVoltage);
  Serial.print(" (");
  Serial.print((float)solarAverageADC / (float)AVG_CYCLES);
  Serial.print(") buckPWM value: ");
  Serial.println(buckPWM);
}

void getVoltages() {
  battAverageADC = 0;
  for (int i = 0 ; i < AVG_CYCLES ; i++) battAverageADC += analogRead(BATTVOLTPIN); // average digital value
  battVoltage = ((float)battAverageADC / (float)AVG_CYCLES) / BATTVOLTCOEFF;  // convert ADC value to actual voltage

  solarAverageADC = 0;
  for (int i = 0 ; i < AVG_CYCLES ; i++) solarAverageADC += analogRead(SOLARVOLTPIN); // average digital value
  solarVoltage = ((float)solarAverageADC / (float)AVG_CYCLES) / SOLARVOLTCOEFF;  // convert ADC value to actual voltage
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}



