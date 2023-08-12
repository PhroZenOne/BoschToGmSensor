#define boschMapSize 37
#define gmMapSize 22
#define sensorRead A3
#define maxxEcuPullupResistor 2500
#define standardVoltage 5
#define minRelayPosition 3
#define maxRelayPosition 12

typedef struct tmpToRes_t {
  int temp;
  int resistance;
} tmpToRes_t;

tmpToRes_t boschSensorValues[boschMapSize] = {
  { -40, 44864 },
  { -35, 33674 },
  { -30, 25524 },
  { -25, 19525 },
  { -20, 15067 },
  { -15, 11724 },
  { -10, 9195 },
  { -5, 7266 },
  { 0, 5784 },
  { 5, 4636 },
  { 10, 3740 },
  { 15, 3037 },
  { 20, 2480 },
  { 25, 2038 },
  { 30, 1683 },
  { 35, 1398 },
  { 40, 1167 },
  { 45, 979 },
  { 50, 825 },
  { 55, 698 },
  { 60, 594 },
  { 65, 507 },
  { 70, 435 },
  { 75, 374 },
  { 80, 323 },
  { 85, 280 },
  { 90, 244 },
  { 95, 213 },
  { 100, 187 },
  { 105, 164 },
  { 110, 144 },
  { 115, 127 },
  { 120, 113 },
  { 125, 101 },
  { 130, 90 },
  { 135, 80 },
  { 140, 72 }
};

tmpToRes_t gmSensorValues[gmMapSize] = {
  { -50, 150394 },
  { -40, 75780 },
  { -30, 39860 },
  { -20, 21860 },
  { -10, 12460 },
  { 0, 7353 },
  { 10, 4482 },
  { 20, 2813 },
  { 25, 2252 },
  { 30, 1814 },
  { 40, 1199 },
  { 50, 811 },
  { 60, 560 },
  { 70, 395 },
  { 80, 283 },
  { 90, 206 },
  { 100, 153 },
  { 110, 115 },
  { 120, 88 },
  { 130, 68 },
  { 140, 53 },
  { 150, 42 }
};

int currentSimulatedResistance = -1;

void setup() {
  Serial.begin(9600);
  delay(100);

  // Volage from between sensor and maxxecu
  pinMode(sensorRead, INPUT);
  digitalWrite(sensorRead, LOW);

  for (unsigned int i = 12; i >= 3; i--) {
    Serial.print(i);
    Serial.println(" set to high");
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
}

void loop() {
  unsigned int sensorValue = analogRead(sensorRead);
  double boschResistance = analogInputValueToResistance(sensorValue);
  double boschTemperature = boshResistanceToTemperature(boschResistance);
  int newResistance = tempToGmResistance(boschTemperature);
  if (newResistance == currentSimulatedResistance) {
    delay(1000);
    return;
  }
  currentSimulatedResistance = newResistance;

  configureRelays(currentSimulatedResistance);
  delay(100);
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

double interpolateTemp(tmpToRes_t lowerBound, tmpToRes_t upperBound, double targetResistance) {
  int resDiff = upperBound.resistance - lowerBound.resistance;
  double percent = (lowerBound.resistance - targetResistance) / (resDiff * 1.0d);
  return ((upperBound.temp - lowerBound.temp) * percent) + lowerBound.temp;
}

double boshResistanceToTemperature(double targetResistance) {
  int low = 0;
  int high = boschMapSize;
  while (low < high) {
    int mid = low + (high - low) / 2;
    if (boschSensorValues[mid].resistance < targetResistance) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }
  tmpToRes_t selected = boschSensorValues[low];
  if (selected.resistance < targetResistance && low < boschMapSize - 1) {
    tmpToRes_t nextUpper = boschSensorValues[low + 1];
    return interpolateTemp(selected, nextUpper, targetResistance);
  }
  if (selected.resistance > targetResistance && low > 0) {
    tmpToRes_t nextLower = boschSensorValues[low - 1];
    return interpolateTemp(nextLower, selected, targetResistance);
  }
  return selected.temp;
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

int tempToGmResistance(double targetTemperature) {
  int low = 0;
  int high = gmMapSize;
  while (low < high) {
    int mid = low + (high - low) / 2;
    if (gmSensorValues[mid].temp < targetTemperature) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }
  tmpToRes_t selected = gmSensorValues[low];
  if (selected.temp < targetTemperature && low < gmMapSize - 1) {
    tmpToRes_t nextUpper = gmSensorValues[low + 1];
    return interpolateResistance(selected, nextUpper, targetTemperature);
  }
  if (selected.temp > targetTemperature && low > 0) {
    tmpToRes_t nextLower = gmSensorValues[low - 1];
    return interpolateResistance(nextLower, selected, targetTemperature);
  }
  return selected.temp;
}

int interpolateResistance(tmpToRes_t lowerBound, tmpToRes_t upperBound, double targetTemperature) {
  int resDiff = upperBound.temp - lowerBound.temp;
  double percent = (lowerBound.temp - targetTemperature) / (resDiff * 1.0d);
  return round(((upperBound.resistance - lowerBound.resistance) * percent) + lowerBound.resistance);
}
