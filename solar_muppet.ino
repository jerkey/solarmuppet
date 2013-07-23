#define VOLTCOEFF 13.25 // calibrates perception of battery voltage
#define MAXBUCK 254 // maximum pwm value for buck converter

#define BUCKPIN 3 // which pin controls buck-converter transistors
// only use pin 3, 9, 10, or 11.  using 5 or 6 messes up millis() timing
#define SOLARDISCONNECTPIN 2 // turns on transistor linking solar- to ground
#define BATTVOLTPIN A0 // which pin reports battery voltage from resistors
#define SOLARVOLTPIN A1 // which pin reports solar panel voltage from resistors
#define BUCK_CUTIN 5.0  // minimum voltage to turn on transistors
#define BUCK_PERIOD 250 // milliseconds between buck converter pwm updates
#define SOLAR_CUTIN 13.0 // voltage above which the solar panel is useful
#define GETLOOPS 10  // how many times to getvoltage per main loop
#define DISPLAY_PERIOD 1500  // how many milliseconds between printdisplay()s
#define AVG_CYCLES 50  // cycles of averaging function

#define BAUDRATE 57600 // serial baud rate for communications with world

float battVoltage, lastBattVoltage = 0;  // voltage of battery
float solarVoltage = 0;  // voltage at panel
int battAverageADC = 0;  // average ADC value of battvoltpin
int solarAverageADC = 0;  // average ADC value of solarvoltpin
unsigned long timeNow, lastBuck, lastDisplay = 0; // to hold system times
int buckDirection = 1;  // amount to change PWM of buck converter while hunting
int lastBuckPWM, buckPWM = 0;  // PWM value of buck converter
float buckJump = 5.0;  // how much to change buckPWM per doBuck() cycle

void setup() {
  setPwmFrequency(BUCKPIN,1); // this sets the frequency of PWM on pins 3 and 11 to 31,250 Hz
  Serial.begin(BAUDRATE);
  pinMode(BUCKPIN,OUTPUT);  // connects to high-side transistors (Drain to Solar plus)
  pinMode(SOLARDISCONNECTPIN,OUTPUT);  // this must be high to tie solar- to ground when charging
  for (int i = 0 ; i < GETLOOPS ; i++) getVoltages();  // get initial voltage measurements
}

void loop() {
  timeNow = millis();  // get system time for this loop
  for (int i = 0 ; i < GETLOOPS ; i++) getVoltages();  // update voltage measurements
  doBuck();  // update buck converter PWM value
  if (timeNow - lastDisplay > DISPLAY_PERIOD) {
    printDisplay();  // send serial information to the world
    lastDisplay = timeNow;
  }
}

void doBuck() {
  if (battVoltage > BUCK_CUTIN) { // voltage is high enough to turn on transistors
    if (solarVoltage > SOLAR_CUTIN) digitalWrite(SOLARDISCONNECTPIN,HIGH);  // turn on minus-side FET
		  else {
			digitalWrite(SOLARDISCONNECTPIN,LOW); // dont want solar panel draining batts at night
			buckPWM = 0;  // turn off high side FET
			setPWM(buckPWM);
			}
    if (timeNow - lastBuck > BUCK_PERIOD) { // if it has been long enough since last time
      if (!buckPWM) {  // the sun just came up
				buckDirection = buckJump; // start with an initial PWM value
				buckPWM += buckDirection; // we are going to track up now
				setPWM(buckPWM);  // set the PWM value
      } else {
				if (battVoltage < lastBattVoltage) buckDirection *= -1;  // if voltage went down, change hunting direction
				buckPWM += buckDirection; // hunt in whatever direction we are trying now
				setPWM(buckPWM);  // set the PWM value
      }
    } // if (timeNow - lastBuck > BUCK_PERIOD)
  }  // if (battVoltage > BUCK_CUTIN)
}

void setPWM(int pwmVal) {
  if (pwmVal != lastBuckPWM) {
    lastBuckPWM = pwmVal;
    analogWrite(BUCKPIN,pwmVal);
  }
}

void printDisplay() {
  Serial.print("battery: ");
  Serial.print(battVoltage);
  Serial.print(" (");
  Serial.print(battAverageADC);
  Serial.print(") solar: ");
  Serial.print(solarVoltage);
  Serial.print(" (");
  Serial.print(solarAverageADC);
  Serial.print(") buckPWM value: ");
  Serial.println(buckPWM);
}

void getVoltages() {
  battAverageADC = average(analogRead(BATTVOLTPIN), battAverageADC); // average digital value
  battVoltage = battAverageADC / VOLTCOEFF;  // convert ADC value to actual voltage
  
  solarAverageADC = average(analogRead(BATTVOLTPIN), solarAverageADC); // average digital value
  solarVoltage = solarAverageADC / VOLTCOEFF;  // convert ADC value to actual voltage
}

float average(float val, float avg){
  if(avg == 0)
  avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
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

