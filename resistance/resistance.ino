#define boschMapSize 37
#define gmMapSize 22
#define sensorRead A7
#define maxxEcuPullupResistor 2500
#define standardVoltage 5
#define minRelayPosition 3
#define maxRelayPosition 12
#define builtInResistance 35.5d

//from https://rusefi.com/Steinhart-Hart.html
// 100F -> 1400
// 190F -> 85
// 280F -> 35
#define gmA 0.0005863816167495897d
#define gmB 0.0005691994843139358d
#define gmC -0.000003928812678091945d


#define boschA 0.001290549974604263d
#define boschB 0.00026131506246163474d
#define boschC 0.00000016232144788010084d

#define kelvinOffset 273.15d

#define maxResistanceValue 2047

unsigned int currentSimulatedResistance = 0;

void setup() {
  Serial.begin(9600);
  delay(100);

  // Volage from between sensor and maxxecu
  pinMode(sensorRead, INPUT);
  digitalWrite(sensorRead, LOW);

  // start with max resistance (coldest temperature)
  for (unsigned int i = 12; i >= 3; i--) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  delay(100);

  int resistance = temperatureToResistance(88.0d, gmA, gmB, gmC);
  configureRelays(resistance);

  delay(2000);

  // needle sweep
  for (int i = 37; i <= 138; i++){
    double resistance = temperatureToResistance(i, gmA, gmB, gmC);
    configureRelays(resistance);
    delay(5);
  }
  for (int i = 138; i >= 37; i--){
    double resistance = temperatureToResistance(i, gmA, gmB, gmC);
    configureRelays(resistance);
    delay(5);
  }
  configureRelays(maxResistanceValue);
}

void loop() {
  unsigned int sensorValue = analogRead(sensorRead);
  double boschResistance = analogInputValueToResistance(sensorValue);
  double boschTemperature = resistanceToTemperature(boschResistance, boschA, boschB, boschC);

  double newResistance = temperatureToResistance(boschTemperature, gmA, gmB, gmC);
  if (newResistance == currentSimulatedResistance) {
    delay(2000);
    return;
  }
  currentSimulatedResistance = newResistance;
  configureRelays(currentSimulatedResistance);
  delay(2000);
}

double resistanceToTemperature(int resistance, double a, double b, double c){
  double logR = log(resistance);
  return 1.0d / (a + (b * logR) + (c * pow(logR, 3.0d))) - kelvinOffset;
}

// Binary search to find a close inverse,
// as the inverse algos breaks down
// to imaginary numbers
// when trying the ABC constants I got
// and my math skills are not good enough to 
// figure out how to handle it.  
int temperatureToResistance(double temperatureInC, double a, double b, double c){
  int rMax = 2024;
  int rMin = 1;
  int r = 700;
  while(r < rMax && r > rMin){
      double t = resistanceToTemperature(r, a, b, c);
      if(t < temperatureInC){
          rMax = r;
          r = rMin + ((r - rMin)/2);
      } else {
          rMin = r;
          r = r + ((rMax - r)/2);
      }
  }
  return r;
}

void configureRelays(int resistance) {
  resistance = max(min(resistance-builtInResistance, maxResistanceValue), 0);
  unsigned int resistanceActiveFlags = translatePattern(resistance);
  for (int relay = minRelayPosition; relay <= maxRelayPosition; relay++) {
    bool disableRelay = resistanceActiveFlags & (1 << relay);
    if (disableRelay) {
      digitalWrite(relay, LOW);
    } else {
      digitalWrite(relay, HIGH);
    }
  }
}

void printBinary16(unsigned int iIn) {
  for (unsigned int mask = 0b1000000000000000; mask; mask >>= 1) {
    if (mask & iIn) {
      Serial.print('1');
    } else {
      Serial.print('0');
    }
  }
}

double analogInputValueToResistance(unsigned int readValue) {
  double voltageOnPin = (readValue * 5.0) / 1023.0d;
  return (maxxEcuPullupResistor * voltageOnPin) / (standardVoltage - voltageOnPin);
}

/**
* resistorlevel is in ohm from 0 to (1 << 11) - 1
* this translates to a physical relay grid
* 1 means the resistance should be in circut, 0 means it is bypassed
**/
unsigned int translatePattern(unsigned int resistorLevel) {
  int ret = 0;
  // resistor i1 d7 = 2 ohm
  ret |= resistorLevel & 2 ? (1 << 7) : 0;
  // resistor i2 d6 = 4 ohm
  ret |= resistorLevel & 4 ? (1 << 6) : 0;
  // resistor i3 d5 = 8 ohm
  ret |= resistorLevel & 8 ? (1 << 5) : 0;
  // resistor i4 d4 = 16 ohm
  ret |= resistorLevel & 16 ? (1 << 4) : 0;
  // resistor i5 d3 = 32 ohm
  ret |= resistorLevel & 32 ? (1 << 3) : 0;
  // resistor i6 d8 = 64 ohm
  ret |= resistorLevel & 64 ? (1 << 8) : 0;
  // resistor i7 d9 = 128 ohm
  ret |= resistorLevel & 128 ? (1 << 9) : 0;
  // resistor i8 d10 = 256 ohm
  ret |= resistorLevel & 256 ? (1 << 10) : 0;
  // resistor i9 d11 = 512 ohm
  ret |= resistorLevel & 512 ? (1 << 11) : 0;
  // resistor i10 d12 = 1024 ohm
  ret |= resistorLevel & 1024 ? (1 << 12) : 0;
  return ret;
}

