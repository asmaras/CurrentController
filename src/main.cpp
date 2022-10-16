#include <Arduino.h>
#include <SPI.h>
// #include <EEPROM.h>

// This is the mA range of the sensor covered by ADC values 0 to max
// The maximum current rating of the sensor may be less
constexpr uint16_t _currentSensorRangeInmA = 50000;

// The ADC value that corresponds to zero current
constexpr int _currentSensorAdcZeroPoint = 512;

int _currentMinimumInmA = 4000;
int _currentDeadBandInmA = 500;
int _currentHardLimitInmA = 8000;

int _powerOnDelay = 15;
constexpr int _currentMeasureFrequency = 1000;
int _currentIntegralRegulationCoefficient = 7;
int _currentIntegralRegulationFrequencyMax = 30;
int _currentIntegralRegulationFrequencyMin = 1;
constexpr int _safeModeTime = 3;

constexpr int _buttonReadFrequency = 20;
constexpr int _blinkerFrequency = 200;

constexpr uint8_t _currentSensorAndButtonPin = A0;
constexpr int _currentSensorAndButtonPinThresholdForButton = 950;
constexpr int _enableCurrentRegulationNotPin = 3;
constexpr int _statusLedsPin = 4;

bool IntervalHasPassed(bool& startFirstInterval, unsigned long& startTime, int frequency);
void MeasureAndRegulate();
void HandleButtonInput();
void EnableRegulation();
void DisableRegulation();
void SetLowestCurrent();
void IncrementCurrent();
void DecrementCurrent();
void SetPotentiometers();
void Blinker();
void BlinkerRegulationAction();

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
  // Set combined pin for the two status LEDs to tri-state
  pinMode(_statusLedsPin, INPUT);
  // Initially set regulation to the lowest current
  // During the power-on delay regulation is also disabled by hardware
  // that uses an external pull-up resistor
  // Regulation can not take place before we set a pin low from software
  SetLowestCurrent();
  // uint8_t eepromValue = EEPROM.read(0);
  // EEPROM.write(0, eepromValue + 1);
  // TEST testStruct;
  // EEPROM.put(0, testStruct);
  // EEPROM.get(0, testStruct);
  // SPI.transfer16(0x418F);
}

unsigned long _currentTime = 0;

void loop() {
  _currentTime = micros();

  // Check power-on delay
  static bool powerOnDelayCompleted = false;
  if (!powerOnDelayCompleted && (_currentTime / 1000000 >= (unsigned)_powerOnDelay)) {
    powerOnDelayCompleted = true;
    // Enable regulation
    // This pin is externaly pulled up so at power-up regulation is always disabled
    // From software we can set this pin low to enable regulation
    // First set value then set pin as output
    EnableRegulation();
    pinMode(_enableCurrentRegulationNotPin, OUTPUT);
  }

  if (powerOnDelayCompleted) {
    MeasureAndRegulate();
  }
  
  HandleButtonInput();

  Blinker();
}

bool IntervalHasPassed(bool& startFirstInterval, unsigned long& startTime, int frequency) {
  if (startFirstInterval) {
    startTime = _currentTime;
    startFirstInterval = false;
    return true;
  }
  else {
    unsigned long interval = 1000000 / frequency;

    if (_currentTime - startTime >= interval) {
      // If we are within the next interval, compensate for clock skew by adding exactly one interval time
      if (_currentTime - startTime < interval * 2) {
        startTime += interval;
      }
      else {
        startTime = _currentTime;
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
int _currentSensorReadingSuspendedCountdown = 0;

void SuspendCurrentSensorReading() {
  _currentSensorReadingSuspended = true;
  _currentSensorReadingSuspendedCountdown = 2;
}

void ResumeCurrentSensorReading() {
  _currentSensorReadingSuspended = false;
  CurrentSensorNewOversamplingCycle();
}

bool GetCurrentSensorAnalogValue(int& value) {
  if (_currentSensorReadingSuspended) {
      return false;
  }
  else {
    value = analogRead(_currentSensorAndButtonPin);
    if (value < _currentSensorAndButtonPinThresholdForButton) {
      return true;
    }
    else {
      // Because the pin is shared suspend sensor reading during button presses
      SuspendCurrentSensorReading();
      return false;
    }
  }
}

bool GetCurrentSensorValueInmA(int& currentInmA) {
  // See https://en.wikipedia.org/wiki/Oversampling
  constexpr int numberOfBits = 13; // Rest will be calculated from this
  constexpr int numberOfExtraBits = numberOfBits - 10;
  constexpr int numberOfReadings = 1 << (numberOfExtraBits * 2); // 4 ^ numberOfExtraBits
  constexpr int oversampledValueRange = 1 << numberOfBits;
  constexpr int zeroPoint = _currentSensorAdcZeroPoint << numberOfExtraBits;

  // Add 10-bit reading
  int reading10Bit;
  if(GetCurrentSensorAnalogValue(reading10Bit)) {
    _currentSensorReadingOversampledValue += reading10Bit;
    _currentSensorReadingCounter++;
    if (_currentSensorReadingCounter == numberOfReadings) {
      // Decimate accumulated readings
      _currentSensorReadingOversampledValue >>= numberOfExtraBits;

      currentInmA = ((_currentSensorReadingOversampledValue - (uint32_t)zeroPoint) * (uint32_t)_currentSensorRangeInmA) / (uint32_t)oversampledValueRange;

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
    (int32_t)_currentIntegralRegulationFrequencyMin +
    (
      (
        (int32_t)deviationInmA * (int32_t)_currentIntegralRegulationCoefficient
        ) /
      1000UL
      );
  if (changeFrequency > _currentIntegralRegulationFrequencyMax) {
    return _currentIntegralRegulationFrequencyMax;
  }
  else {
    return changeFrequency;
  }
}

bool _safeMode = false;
unsigned long _safeModeStartTime = 0;

void GoToSafeMode() {
  // Activate the regulation disable output
  // As an extra safety also set regulation to lowest possible current
  DisableRegulation();
  SetLowestCurrent();
  _safeMode = true;
  _safeModeStartTime = _currentTime;
}

enum class CurrentIntegralRegulationAction {
  none,
  up,
  down
};

void MeasureAndRegulate() {
  static bool startFirstCurrentMeasureInterval = true;
  static unsigned long currentMeasureIntervalStartTime = 0;
  int currentInmA = 0;
  static CurrentIntegralRegulationAction currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
  bool changeCurrentImmediately = false;
  static int changeFrequency = 0;
  // Perform sensor measurements at a fixed frequency
  if (IntervalHasPassed(startFirstCurrentMeasureInterval, currentMeasureIntervalStartTime, _currentMeasureFrequency)) {
    if (GetCurrentSensorValueInmA(currentInmA)) {
      if (_safeMode) {
        // In safe mode
        // Go back to regulation mode when the current stays under the minimum for some time
        if (
          currentInmA < _currentMinimumInmA &&
          _currentTime - _safeModeStartTime >= _safeModeTime * 1000000
          ) {
          _safeMode = false;
          EnableRegulation();
        }
      }
      else {
        // In normal regulation mode
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
          GoToSafeMode();
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
  }

  // Integral regulation actions are done at their own frequency, independent from sensor measurements
  if (!_safeMode && currentIntegralRegulationAction != CurrentIntegralRegulationAction::none) {
    static unsigned long regulateIntervalStartTime = 0;
    if (IntervalHasPassed(changeCurrentImmediately, regulateIntervalStartTime, changeFrequency)) {
      switch (currentIntegralRegulationAction) {
      case CurrentIntegralRegulationAction::up:
        IncrementCurrent();
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

uint16_t _aggregatePotentiometerValue = 0;

void SetLowestCurrent() {
  _aggregatePotentiometerValue = 0;
  SetPotentiometers();
}

void IncrementCurrent() {
  if (_aggregatePotentiometerValue < 512) {
    _aggregatePotentiometerValue++;
    SetPotentiometers();
  }
  BlinkerRegulationAction();
}

void DecrementCurrent() {
  if (_aggregatePotentiometerValue > 0) {
    _aggregatePotentiometerValue--;
    SetPotentiometers();
  }
}

void SetPotentiometers() {
  // MCP4251 is used, see https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22060b.pdf
  SPI.transfer16((_aggregatePotentiometerValue + 1) / 2);
  SPI.transfer16(0x1000 | (_aggregatePotentiometerValue / 2));

  // Check value of wiper 0
  // Expected are 7 bits set to "1" followed by the value
  if (SPI.transfer16(0x0C00) != (0xFE00 | (_aggregatePotentiometerValue + 1) / 2)) {
    GoToSafeMode();
  }
}

enum class ButtonEvent {
  none,
  shortPress,
  longPress
};

void HandleButtonInput() {
  static bool buttonDepressed = false;
  ButtonEvent buttonEvent = ButtonEvent::none;
  static bool startFirstButtonReadInterval = true;
  static unsigned long buttonReadIntervalStartTime = 0;
  if (IntervalHasPassed(startFirstButtonReadInterval, buttonReadIntervalStartTime, _buttonReadFrequency)) {
    if (analogRead(_currentSensorAndButtonPin) >= _currentSensorAndButtonPinThresholdForButton) {
      // Because the pin is shared suspend sensor reading during button presses
      SuspendCurrentSensorReading();

      if (!buttonDepressed) {
        buttonDepressed = true;
        buttonEvent = ButtonEvent::shortPress;
      }
    }
    else {
      // If sensor reading is suspended it may be resumed after button release has been detected a few times
      if (_currentSensorReadingSuspended) {
        if (--_currentSensorReadingSuspendedCountdown == 0) {
          ResumeCurrentSensorReading();
        }
      }

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

void Blinker() {
  bool led2On = false;
  switch (_led2IndicatorInfo.mode) {
  case LedIndicatorMode::none:
    break;
  case LedIndicatorMode::pulseOnce:
    if (_currentTime - _led2IndicatorInfo.pulseOnceStartTime >= 11 * 1000UL) {
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
  if (IntervalHasPassed(startFirstAliveInterval, aliveIntervalStartTime, 3)) {
    led1On = !led1On;
  }

  // Multiplex the LEDs on one output
  static bool startFirstMultiplexInterval = true;
  static unsigned long multiplexIntervalStartTime = 0;
  static uint8_t multiplexSelector = 0;
  if (IntervalHasPassed(startFirstMultiplexInterval, multiplexIntervalStartTime, _blinkerFrequency)) {
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

void BlinkerRegulationAction() {
  _led2IndicatorInfo.mode = LedIndicatorMode::pulseOnce;
  _led2IndicatorInfo.pulseOnceStartTime = _currentTime;
}