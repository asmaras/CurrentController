#include <Arduino.h>
#include <SPI.h>
// #include <EEPROM.h>

int _currentMinimumInmA = 13000;
int _currentDeadBandInmA = 500;
int _currentHardLimitInmA = 15000;

constexpr int _currentMeasureFrequency = 1000;
int _currentIntegralRegulationRangeInmA = 4000;
int _currentIntegralRegulationFrequencyMax = 30;
int _currentIntegralRegulationFrequencyMin = 5;

constexpr int _buttonReadFrequency = 20;
constexpr int _blinkerFrequency = 200;

constexpr uint8_t _currentSensorAndButtonPin = A0;
constexpr int _currentSensorAndButtonPinThresholdForButton = 950;
constexpr int _enableCurrentRegulationNotPin = 3;
constexpr int _statusLedsPin = 4;

bool IntervalHasPassed(bool& startFirstInterval, unsigned long currentTime, unsigned long& startTime, int frequency);
void MeasureAndRegulate(unsigned long currentTime);
void HandleButtonInput(unsigned long currentTime);
void EnableRegulation();
void DisableRegulation();
void SetLowestCurrent();
void IncrementCurrent(unsigned long currentTime);
void DecrementCurrent();
void SetPotentiometers();
void Blinker(unsigned long currentTime);
void BlinkerRegulationAction(unsigned long currentTime);

struct TEST {
  int test1;
  float test2;
};

void setup() {
  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  // Set combined pin for current sensor and button as input
  pinMode(_currentSensorAndButtonPin, INPUT);
  // Before enabling current regulation we must set it to the lowest current
  SetLowestCurrent();
  // Enable regulation
  // This pin is externaly pulled up so at power-up regulation is always disabled
  // From software we can set this pin low to enable regulation
  // First set value then set pin as output
  EnableRegulation();
  pinMode(_enableCurrentRegulationNotPin, OUTPUT);
  // Set combined pin for the two status LEDs to tri-state
  pinMode(_statusLedsPin, INPUT);
  // uint8_t eepromValue = EEPROM.read(0);
  // EEPROM.write(0, eepromValue + 1);
  // TEST testStruct;
  // EEPROM.put(0, testStruct);
  // EEPROM.get(0, testStruct);
}

void loop() {
  unsigned long currentTime = micros();

  MeasureAndRegulate(currentTime);
  
  HandleButtonInput(currentTime);

  Blinker(currentTime);
}

bool IntervalHasPassed(bool& startFirstInterval, unsigned long currentTime, unsigned long& startTime, int frequency) {
  if (startFirstInterval) {
    startTime = currentTime;
    startFirstInterval = false;
    return true;
  }
  else {
    unsigned long interval = 1000000 / frequency;

    if (currentTime - startTime >= interval) {
      // If we are within the next interval, compensate for clock skew by adding exactly one interval time
      if (currentTime - startTime < interval * 2) {
        startTime += interval;
      }
      else {
        startTime = currentTime;
      }
      return true;
    }
    else {
      return false;
    }
  }
}

int _currentSensorReadingCounter = 0;
uint32_t _currentSensorReadingOversampledValue = 0;

void CurrentSensorNewOversamplingCycle() {
  _currentSensorReadingCounter = 0;
  _currentSensorReadingOversampledValue = 0;
}

bool _currentSensorReadingSuspended = false;
unsigned long _currentSensorReadingSuspendedStartTime = 0;

void SuspendCurrentSensorReading(unsigned long currentTime) {
  _currentSensorReadingSuspended = true;
  _currentSensorReadingSuspendedStartTime = currentTime;
}

void ResumeCurrentSensorReading() {
  _currentSensorReadingSuspended = false;
  CurrentSensorNewOversamplingCycle();
}

bool GetCurrentSensorAnalogValue(unsigned long currentTime, int& value) {
  if (_currentSensorReadingSuspended) {
    if (currentTime - _currentSensorReadingSuspendedStartTime >= 100 * 1000UL) {
      ResumeCurrentSensorReading();
    }
    else {
      return false;
    }
  }

  value = analogRead(_currentSensorAndButtonPin);
  if (value < _currentSensorAndButtonPinThresholdForButton) {
    return true;
  }
  else {
    SuspendCurrentSensorReading(currentTime);
    return false;
  }
}

bool GetCurrentSensorValueInmA(unsigned long currentTime, int& currentInmA) {
  // This is the mA range of the sensor covered by ADC values 0 to max
  // The maximum current rating of the sensor may be less
  constexpr uint16_t sensorRangeInmA = 50000;

  // See https://en.wikipedia.org/wiki/Oversampling
  constexpr int numberOfBits = 12; // Rest will be calculated from this
  constexpr int numberOfExtraBits = numberOfBits - 10;
  constexpr int numberOfReadings = 1 << (numberOfExtraBits * 2); // 4 ^ numberOfExtraBits
  constexpr int oversampledValueRange = 1 << numberOfBits;
  constexpr int halfwayPoint = oversampledValueRange >> 1;

  // Add 10-bit reading
  int reading10Bit;
  if(GetCurrentSensorAnalogValue(currentTime, reading10Bit)) {
    _currentSensorReadingOversampledValue += analogRead(_currentSensorAndButtonPin);
    _currentSensorReadingCounter++;
    if (_currentSensorReadingCounter == numberOfReadings) {
      // Decimate accumulated readings
      _currentSensorReadingOversampledValue >>= numberOfExtraBits;

      currentInmA = ((_currentSensorReadingOversampledValue - (uint32_t)halfwayPoint) * (uint32_t)sensorRangeInmA) / (uint32_t)oversampledValueRange;

      // Prepare for next oversampling cycle
      CurrentSensorNewOversamplingCycle();
      return true;
    }
  }

  return false;
}

int CalculateCurrentChangeFrequency(int currentInmA, int setpointCurrentInmA) {
  int deviationInmA = abs(setpointCurrentInmA - currentInmA);
  int changeFrequency =
    (
      (int32_t)(_currentIntegralRegulationFrequencyMax - _currentIntegralRegulationFrequencyMin) *
      (int32_t)deviationInmA
    ) /
    (int32_t)_currentIntegralRegulationRangeInmA;
  if (changeFrequency > _currentIntegralRegulationFrequencyMax) {
    changeFrequency = _currentIntegralRegulationFrequencyMax;
  }
  else if (changeFrequency < _currentIntegralRegulationFrequencyMin) {
    changeFrequency = _currentIntegralRegulationFrequencyMin;
  }
  return changeFrequency;
}

enum class CurrentIntegralRegulationAction {
  none,
  up,
  down
};

void MeasureAndRegulate(unsigned long currentTime) {
  static bool startFirstCurrentMeasureInterval = true;
  static unsigned long currentMeasureIntervalStartTime = 0;
  int currentInmA = 0;
  static CurrentIntegralRegulationAction currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
  bool changeCurrentImmediately = false;
  static int changeFrequency = 0;
  if (IntervalHasPassed(startFirstCurrentMeasureInterval, currentTime, currentMeasureIntervalStartTime, _currentMeasureFrequency)) {
    if (GetCurrentSensorValueInmA(currentTime, currentInmA)) {
      if (currentInmA < _currentMinimumInmA) {
        // Current too low, regulate up
        if (currentIntegralRegulationAction != CurrentIntegralRegulationAction::up) {
          changeCurrentImmediately = true;
        }
        currentIntegralRegulationAction = CurrentIntegralRegulationAction::up;
        changeFrequency = CalculateCurrentChangeFrequency(currentInmA, _currentMinimumInmA);
      }
      else if (currentInmA > _currentHardLimitInmA) {
        // Current dangerously high, go to safe mode
        // Activate the regulation disable output
        // As an extra safety also set regulation to lowest possible current
        currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
        DisableRegulation();
        SetLowestCurrent();
      }
      else if (currentInmA > _currentMinimumInmA + _currentDeadBandInmA) {
        // Current is too high, regulate down
        if (currentIntegralRegulationAction != CurrentIntegralRegulationAction::down) {
          changeCurrentImmediately = true;
        }
        currentIntegralRegulationAction = CurrentIntegralRegulationAction::down;
        changeFrequency = CalculateCurrentChangeFrequency(currentInmA, _currentMinimumInmA + _currentDeadBandInmA);
      }
      else {
        // Current is within dead band, do nothing
        currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
      }
    }
  }

  if (currentIntegralRegulationAction != CurrentIntegralRegulationAction::none) {
    static unsigned long regulateIntervalStartTime = 0;
    if (IntervalHasPassed(changeCurrentImmediately, currentTime, regulateIntervalStartTime, changeFrequency)) {
      switch (currentIntegralRegulationAction) {
      case CurrentIntegralRegulationAction::up:
        IncrementCurrent(currentTime);
        break;
      case CurrentIntegralRegulationAction::down:
        DecrementCurrent();
        break;
      case CurrentIntegralRegulationAction::none:
        break;
      }
    }
  }
}

void EnableRegulation() {
  digitalWrite(_enableCurrentRegulationNotPin, 0);
}

void DisableRegulation() {
  digitalWrite(_enableCurrentRegulationNotPin, 1);
}

uint16_t aggregatePotentiometerValue = 0;

void SetLowestCurrent() {
  aggregatePotentiometerValue = 0;
  SetPotentiometers();
}

void IncrementCurrent(unsigned long currentTime) {
  if (aggregatePotentiometerValue < 512) {
    aggregatePotentiometerValue++;
    SetPotentiometers();
  }
  BlinkerRegulationAction(currentTime);
}

void DecrementCurrent() {
  if (aggregatePotentiometerValue > 0) {
    aggregatePotentiometerValue--;
    SetPotentiometers();
  }
}

void SetPotentiometers() {
  SPI.transfer16((aggregatePotentiometerValue + 1) / 2);
  SPI.transfer16((aggregatePotentiometerValue / 2) | 0x1000);
}

enum class ButtonEvent {
  none,
  shortPress,
  longPress
};

void HandleButtonInput(unsigned long currentTime) {
  static bool buttonDepressed = false;
  ButtonEvent buttonEvent = ButtonEvent::none;
  static bool startFirstButtonReadInterval = true;
  static unsigned long buttonReadIntervalStartTime = 0;
  if (IntervalHasPassed(startFirstButtonReadInterval, currentTime, buttonReadIntervalStartTime, _buttonReadFrequency)) {
    if (analogRead(_currentSensorAndButtonPin) >= _currentSensorAndButtonPinThresholdForButton) {
      SuspendCurrentSensorReading(currentTime);
      if (!buttonDepressed) {
        buttonDepressed = true;
        buttonEvent = ButtonEvent::shortPress;
      }
    }
    else {
      buttonDepressed = false;
    }
  }

  switch (buttonEvent) {
  case ButtonEvent::none:
    break;
  case ButtonEvent::shortPress:
    _currentMinimumInmA += 100;
    break;
  case ButtonEvent::longPress:
    break;
  }
}

enum class LedIndicatorMode {
  none,
  pulseOnce,
  pulseMultipleTimes,
  showDecimal
};
enum class LedIndicatorShowDecimalState {
  showHundreds,
  showTens,
  showOnes
};
struct LedIndicatorInfo {
  LedIndicatorMode mode = LedIndicatorMode::none;
  unsigned long pulseOnceStartTime;
  uint8_t pulseMultipleTimesCounter;
  LedIndicatorShowDecimalState showDecimalState;
  int showDecimalNumber;
  uint8_t showDecimalHundredsCounter;
  uint8_t showDecimalTensCounter;
  uint8_t showDecimalOnesCounter;
};
LedIndicatorInfo _led1IndicatorInfo;
LedIndicatorInfo _led2IndicatorInfo;

void Blinker(unsigned long currentTime) {
  bool led2On = false;
  switch (_led2IndicatorInfo.mode) {
  case LedIndicatorMode::none:
    break;
  case LedIndicatorMode::pulseOnce:
    if (currentTime - _led2IndicatorInfo.pulseOnceStartTime >= 11 * 1000UL) {
      _led2IndicatorInfo.mode = LedIndicatorMode::none;
    }
    else {
      led2On = true;
    }
    break;
  case LedIndicatorMode::pulseMultipleTimes:
    break;
  case LedIndicatorMode::showDecimal:
    break;
  };
  
  static bool led1On = false;

  static bool startFirstAliveInterval = true;
  static unsigned long aliveIntervalStartTime = 0;
  if (IntervalHasPassed(startFirstAliveInterval, currentTime, aliveIntervalStartTime, 3)) {
    led1On = !led1On;
  }

  // Multiplex the LEDs on one output
  static bool startFirstMultiplexInterval = true;
  static unsigned long multiplexIntervalStartTime = 0;
  static uint8_t multiplexSelector = 0;
  if (IntervalHasPassed(startFirstMultiplexInterval, currentTime, multiplexIntervalStartTime, _blinkerFrequency)) {
    if (multiplexSelector % 2 == 0) {
      if (led1On) {
        digitalWrite(_statusLedsPin, 1);
        pinMode(_statusLedsPin, OUTPUT);
      }
      else {
        pinMode(_statusLedsPin, INPUT);
      }
    }
    else {
      if (led2On) {
        digitalWrite(_statusLedsPin, 0);
        pinMode(_statusLedsPin, OUTPUT);
      }
      else {
        pinMode(_statusLedsPin, INPUT);
      }
    }
    multiplexSelector++;
  }
}

void BlinkerRegulationAction(unsigned long currentTime) {
  _led2IndicatorInfo.mode = LedIndicatorMode::pulseOnce;
  _led2IndicatorInfo.pulseOnceStartTime = currentTime;
}