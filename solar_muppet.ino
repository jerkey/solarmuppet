#define VOLTCOEFF 13.25 // calibrates perception of battery voltage
#define MAXBUCK 254 // maximum pwm value for buck converter

#define BUCKPIN 3 // which pin controls buck-converter transistors
// only use pin 3, 9, 10, or 11.  using 5 or 6 messes up millis() timing
#define SOLARDISCONNECTPIN 2 // turns on transistor linking solar- to ground
#define BATTVOLTPIN A0 // which pin reports battery voltage from resistors
#define SOLARVOLTPIN A1 // which pin reports solar panel voltage from resistors
#define BUCK_CUTIN 5.0  // minimum voltage to turn on transistors
#define SOLAR_CUTIN 13.0 // voltage above which the solar panel is useful

#define BAUDRATE 57600 // serial baud rate for communications with world

float battVoltage, lastBattVoltage = 0;  // voltage of battery
float solarVoltage = 0;  // voltage at panel
int battAverageADC = 0;  // average ADC value of battvoltpin
int solarAverageADC = 0;  // average ADC value of solarvoltpin

void setup() {
  setPwmFrequency(BUCKPIN,1); // this sets the frequency of PWM on pins 3 and 11 to 31,250 Hz
  Serial.begin(BAUDRATE);
  pinMode(BUCKPIN,OUTPUT);
  pinMode(SOLARDISCONNECTPIN,OUTPUT);  // this must be high to tie solar- to ground when charging
  getVoltages();
}

void loop() {
  getVoltages();
  doBuck();
  printDisplay();
}

void doBuck() {
  if ((battVoltage > BUCK_CUTIN) || (solarVoltage > SOLAR_CUTIN)) { // voltage is high enough to turn on transistors
    


void getVoltages() {
  battAverageADC = average(analogRead(BATTVOLTPIN), battAverageADC); // average digital value
  lastBattVoltage = battVoltage; // save the previous voltage to see if it's higher or lower
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

