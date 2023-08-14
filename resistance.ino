#define boschMapSize 37
#define gmMapSize 22
#define sensorRead A7
#define maxxEcuPullupResistor 2500
#define standardVoltage 5
#define minRelayPosition 3
#define maxRelayPosition 12

//from https://rusefi.com/Steinhart-Hart.html
#define gmA 0.002210276888384481d
#define gmB 0.00008159559425004526d
#define gmC 0.0000009505063945735256d

#define boschA 0.001290549974604263d
#define boschB 0.00026131506246163474d
#define boschC 0.00000016232144788010084

#define kelvinOffset 273.15

typedef struct tmpToRes_t {
  int temp;
  int resistance;
} tmpToRes_t;

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

  int maxResistanceValue = 2047;

  // then sweep to highest resistance (needle sweep)
  for (int i = 0; i < maxResistanceValue; i++){
    configureRelays(i);
    delay(2);
  }

  for (int i = maxResistanceValue; i >= 0; i--){
    configureRelays(i);
    delay(2);
  }
}

void loop() {
  unsigned int sensorValue = analogRead(sensorRead);
  double boschResistance = analogInputValueToResistance(sensorValue);
  double boschTemperature = resistanceToTemperature(boschResistance, boschA, boschB, boschC);

  double newResistance = temperatureToResistance(boschTemperature, gmA, gmB, gmC);
  if (newResistance == currentSimulatedResistance) {
    delay(1500);
    return;
  }
  currentSimulatedResistance = newResistance;
  configureRelays(currentSimulatedResistance);
  delay(1500);
}

double resistanceToTemperature(int resistance, double a, double b, double c){
  double logR = log(resistance);
  return 1 / (a + (b * logR) + (c * pow(logR, 3))) - kelvinOffset;
}

double temperatureToResistance(double temperature, double a, double b, double c){
  double x = (1/c)*(a-(1/temperature));
  double y = sqrt(pow(b/(3*c), 3)+(pow(x, 2)/4));
  return exp(pow(y-(x/2), 1/3) - pow(y+(x/2), 1/3));
}

void configureRelays(int resistance) {
  unsigned int resistanceActiveFlags = translatePattern(currentSimulatedResistance);
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

