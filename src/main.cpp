#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

// Pure integral controller with dead band for stablility
// Integral regulation is accomplished by using the deviation between
// setpoint and actual value to control the frequency at which the
// output changes
// This effectively accumulates the deviations over time
// The deviation is multiplied by a gain factor (in Hz/A) to get a
// useful value to control the output change frequency

// Initial regulation parameters
// These can be altered from the settings menu
constexpr int _initialCurrentMinimumInmA = 4000;
constexpr int _initialCurrentDeadBandInmA = 500;
constexpr int _initialCurrentHardLimitInmA = 8000;
constexpr int _initialPowerOnDelay = 15;
constexpr int _initialCurrentIntegralRegulationGain = 7; // In Hz/A

// This is the mA range of the sensor covered by ADC values 0 to max
// The maximum current rating of the sensor may be less
// We use this value when no calibration has been done
constexpr uint16_t _currentSensorRangeInmA = 50000;

// Fixed and variable regulation parameters
int _powerOnDelay = _initialPowerOnDelay;
constexpr int _currentMeasureFrequency = 1000;
constexpr int _currentSensorOversamplingNumberOfBits = 13;
struct CurrentSensorCalibrationValues {
  uint16_t referenceCurrentInmA;
  uint16_t adcReferencePoint;
  uint16_t slopeCurrentDeltaInmA;
  uint16_t slopeAdcDelta;
} _currentSensorCalibrationValues = {};
int _currentMinimumInmA = 14000;
int _currentDeadBandInmA = 500;
int _currentHardLimitInmA = 15000;
int _currentIntegralRegulationGain = _initialCurrentIntegralRegulationGain;
constexpr int _currentIntegralRegulationFrequencyMax = 30;
constexpr int _currentIntegralRegulationFrequencyMin = 1;
constexpr int _safeModeTime = 3;

// Frequencies (note that _currentMeasureFrequency is already defined)
constexpr int _buttonReadFrequency = 20;
constexpr int _blinkerFrequency = 200;

// Pins
constexpr uint8_t _currentSensorAndButtonPin = A0;
constexpr int _currentSensorAndButtonPinThresholdForButton = 950;
constexpr int _enableCurrentRegulationNotPin = 3;
constexpr int _statusLedsPin = 4;

bool _debugMode = false;

void MeasureAndRegulate();
void HandleButtonInput();
void EnableRegulation();
void DisableRegulation();
void SetLowestCurrent();
void IncrementCurrent();
void DecrementCurrent();
void SetPotentiometers();
void Blinker();

enum class LedIndicatorLedNumber
{
  Led1 = 0,
  Led2 = 1
};

void LedIndicatorAlive();
void LedIndicatorRegulationAction();
void LedIndicatorShowDecimal(LedIndicatorLedNumber ledIndicatorLedNumber, int decimal);
void LedIndicatorBurst();

enum class EEPROMIndices {
  currentSensorCalibrationValues = 0
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
  // Show alive activity
  LedIndicatorAlive();

  EEPROM.get((int)EEPROMIndices::currentSensorCalibrationValues, _currentSensorCalibrationValues);
  // Test value slopeCurrentDeltaInmA
  // This is sure to be non-zero if calibration has been performed
  _currentSensorCalibrationValues.referenceCurrentInmA = 0;
  // if (_currentSensorCalibrationValues.slopeCurrentDeltaInmA == 0) {
    // Uncalibrated
    _currentSensorCalibrationValues.adcReferencePoint = 1 << (_currentSensorOversamplingNumberOfBits - 1);
    _currentSensorCalibrationValues.slopeCurrentDeltaInmA = _currentSensorRangeInmA;
    _currentSensorCalibrationValues.slopeAdcDelta = 1 << _currentSensorOversamplingNumberOfBits;
  // }
}

unsigned long _currentTime = 0;
bool _powerOnDelayCompleted = false;

void loop() {
  _currentTime = micros();

  // Check power-on delay
  if (!_powerOnDelayCompleted && (_currentTime / 1000000 >= (unsigned)_powerOnDelay)) {
    _powerOnDelayCompleted = true;
    // Enable regulation
    // This pin is externaly pulled up so at power-up regulation is always disabled
    // From software we can set this pin low to enable regulation
    // First set value then set pin as output
    EnableRegulation();
    pinMode(_enableCurrentRegulationNotPin, OUTPUT);
  }

  if (_powerOnDelayCompleted) {
    MeasureAndRegulate();
  }
  
  HandleButtonInput();

  Blinker();
}

bool RunAtFrequency(bool& startFirstInterval, unsigned long& startTime, int frequency) {
  // This function allows software to run at a specified frequency
  // The startFirstInterval parameter is used to immediately run software (return true) the first
  // time this function is called
  // It must be maintained by the caller between function calls
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

void ResumeCurrentSensorReadingIfSuspended() {
  if (_currentSensorReadingSuspended) {
    if (--_currentSensorReadingSuspendedCountdown == 0) {
      _currentSensorReadingSuspended = false;
      CurrentSensorNewOversamplingCycle();
    }
  }
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
  constexpr int numberOfExtraBits = _currentSensorOversamplingNumberOfBits - 10;
  constexpr int numberOfReadings = 1 << (numberOfExtraBits * 2); // 4 ^ numberOfExtraBits

  // Add 10-bit reading
  int reading10Bit;
  if(GetCurrentSensorAnalogValue(reading10Bit)) {
    _currentSensorReadingOversampledValue += reading10Bit;
    _currentSensorReadingCounter++;
    if (_currentSensorReadingCounter == numberOfReadings) {
      // Decimate accumulated readings
      _currentSensorReadingOversampledValue >>= numberOfExtraBits;

      // Calculate current
      // Things could go negative so cast unsigned to signed
      // We use parenthesis around every part to be perfectly clear about computation order
      currentInmA =
        _currentSensorCalibrationValues.referenceCurrentInmA +
        (
          (
            ((int32_t)_currentSensorReadingOversampledValue - (int32_t)_currentSensorCalibrationValues.adcReferencePoint) * // Deviation from reference point
            (int32_t)_currentSensorCalibrationValues.slopeCurrentDeltaInmA // Multiply with current delta per ADC delta (first do numerator part of this ratio to keep precision)
            ) /
          (int32_t)_currentSensorCalibrationValues.slopeAdcDelta // Multiply with current delta per ADC delta (this is the denominator part of the ratio)
          );

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
      ((int32_t)deviationInmA * (int32_t)_currentIntegralRegulationGain) / 1000UL
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
  if (!_safeMode) {
    // Set _safeMode first so that any function called below can call GoToSafeMode again without causing a loop
    _safeMode = true;
    _safeModeStartTime = _currentTime;

    // Activate the regulation disable output
    // As an extra safety also set regulation to lowest possible current
    DisableRegulation();
    SetLowestCurrent();
  }
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
  if (RunAtFrequency(startFirstCurrentMeasureInterval, currentMeasureIntervalStartTime, _currentMeasureFrequency)) {
    if (GetCurrentSensorValueInmA(currentInmA)) {
      if (_safeMode) {
        // In safe mode
        // Go back to regulation mode when the current stays under the minimum for some time
        if (
          currentInmA < _currentMinimumInmA &&
          (_currentTime - _safeModeStartTime >= _safeModeTime * 1000000UL)
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

      LedIndicatorShowDecimal(LedIndicatorLedNumber::Led1, currentInmA / 100);
    }
  }

  // Integral regulation actions are done at their own frequency, independent from sensor measurements
  if (!_safeMode && currentIntegralRegulationAction != CurrentIntegralRegulationAction::none) {
    static unsigned long regulateIntervalStartTime = 0;
    if (RunAtFrequency(changeCurrentImmediately, regulateIntervalStartTime, changeFrequency)) {
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
  LedIndicatorRegulationAction();
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

  if (!_debugMode) {
    // Check value of wiper 0
    // Expected are 7 bits set to "1" followed by the value
    if (SPI.transfer16(0x0C00) != (0xFE00 | ((_aggregatePotentiometerValue + 1) / 2))) {
      GoToSafeMode();
    }
  }
}

enum class ButtonEvent {
  none,
  shortPress,
  longPress
};

void HandleButtonInput() {
  static bool buttonDepressed = false;
  static unsigned long buttonDepressedStartTime = 0;
  static bool buttonLongPressDetected = false;
  ButtonEvent buttonEvent = ButtonEvent::none;
  static bool startFirstButtonReadInterval = true;
  static unsigned long buttonReadIntervalStartTime = 0;
  if (RunAtFrequency(startFirstButtonReadInterval, buttonReadIntervalStartTime, _buttonReadFrequency)) {
    if (analogRead(_currentSensorAndButtonPin) >= _currentSensorAndButtonPinThresholdForButton) {
      // Because the pin is shared suspend sensor reading during button presses
      SuspendCurrentSensorReading();

      if (buttonDepressed) {
        if (!buttonLongPressDetected && _currentTime - buttonDepressedStartTime >= 1000000UL) {
          buttonEvent = ButtonEvent::longPress;
          buttonLongPressDetected = true;
        }
      }
      else {
        buttonDepressed = true;
        buttonDepressedStartTime = _currentTime;
      }
    }
    else {
      // If sensor reading is suspended it may be resumed after button release has been detected a few times
      ResumeCurrentSensorReadingIfSuspended();

      if (buttonDepressed) {
        buttonDepressed = false;
        if (buttonLongPressDetected) {
          buttonLongPressDetected = false;
        }
        else {
          buttonEvent = ButtonEvent::shortPress;
        }
      }
    }
  }

  switch (buttonEvent) {
  case ButtonEvent::none:
    break;
  case ButtonEvent::shortPress:
    if (_powerOnDelayCompleted) {
      _currentMinimumInmA += 100;
    }
    else {
      _debugMode = true;
      LedIndicatorShowDecimal(LedIndicatorLedNumber::Led1, 375);
    }
    break;
  case ButtonEvent::longPress:
    LedIndicatorBurst();
    break;
  }
}

enum class LedIndicatorMode {
  none,
  singleCadence,
  recurringCadence,
  showDecimal
};
struct CadenceElement {
  uint8_t numberOfPulses;
  uint16_t onDuration;
  uint16_t offDuration;
  uint16_t endDuration;
};
struct LedIndicatorInfo {
  bool ledOn;
  LedIndicatorMode mode;
  bool interruptCadenceRunning;
  unsigned long eventStartTime;
  uint16_t eventDuration;
  CadenceElement cadence[5];
  uint8_t cadenceLength;
  uint8_t cadenceIndex;
  uint8_t cadencePulseCounter;
  int showDecimalNumber;
};
LedIndicatorInfo _ledIndicatorInfo[2] = {};

void LedIndicatorShowNewDecimal(LedIndicatorInfo& ledIndicatorInfo) {
  // If interrupt cadence is running do not set up a new cadence
  if (ledIndicatorInfo.interruptCadenceRunning) {
    return;
  }
  // Hundreds
  uint8_t numberOfPulses = ledIndicatorInfo.showDecimalNumber / 100;
  if (numberOfPulses > 0) {
    ledIndicatorInfo.cadence[0].numberOfPulses = numberOfPulses;
    ledIndicatorInfo.cadence[0].onDuration = 400;
    ledIndicatorInfo.cadence[0].offDuration = 200;
    ledIndicatorInfo.cadence[0].endDuration = 700;
    ledIndicatorInfo.cadenceLength = 1;
  }
  else {
    ledIndicatorInfo.cadenceLength = 0;
  }
  // Tens
  numberOfPulses = (ledIndicatorInfo.showDecimalNumber % 100) / 10;
  if (numberOfPulses > 0) {
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].numberOfPulses = numberOfPulses;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].onDuration = 200;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].offDuration = 250;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].endDuration = 700;
    ledIndicatorInfo.cadenceLength++;
  }
  // Ones
  numberOfPulses = ledIndicatorInfo.showDecimalNumber % 10;
  if (numberOfPulses > 0) {
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].numberOfPulses = numberOfPulses;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].onDuration = 100;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].offDuration = 300;
    ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceLength].endDuration = 1000;
    ledIndicatorInfo.cadenceLength++;
  }
}

void RunLedIndicator(LedIndicatorInfo& ledIndicatorInfo) {
  if (
    (ledIndicatorInfo.mode != LedIndicatorMode::none || ledIndicatorInfo.interruptCadenceRunning) &&
    (_currentTime - ledIndicatorInfo.eventStartTime >= (unsigned long)ledIndicatorInfo.eventDuration * 1000UL)
    ) {
    ledIndicatorInfo.eventStartTime = _currentTime;

    if (ledIndicatorInfo.ledOn) {
      // LED is on, turn it off for the off-duration or end-duration
      ledIndicatorInfo.ledOn = false;
      if (
        ledIndicatorInfo.cadencePulseCounter == ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].numberOfPulses &&
        ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].endDuration > 0
        ) {
        ledIndicatorInfo.eventDuration = ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].endDuration;
      }
      else {
        ledIndicatorInfo.eventDuration = ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].offDuration;
      }
    }
    else {
      // LED is off, a period has been completed
      // Check if we are at the end of this part of the cadence
      if (ledIndicatorInfo.cadencePulseCounter == ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].numberOfPulses) {
        // Go to the next part of the cadence
        ledIndicatorInfo.cadenceIndex = (ledIndicatorInfo.cadenceIndex + 1) % ledIndicatorInfo.cadenceLength;
        if (ledIndicatorInfo.cadenceIndex == 0) {
          // We're at the start of the cadence sequence again
          if (ledIndicatorInfo.interruptCadenceRunning) {
            // This was an interrupt cadence and has now ended
            ledIndicatorInfo.interruptCadenceRunning = false;
          }
          switch (ledIndicatorInfo.mode) {
          case LedIndicatorMode::none:
            // Nothing to do so stop
            return;
          case LedIndicatorMode::singleCadence:
            // This was a single cadence so stop
            ledIndicatorInfo.mode = LedIndicatorMode::none;
            return;
          case LedIndicatorMode::recurringCadence:
            // A recurring cadence is active so continue
            break;
          case LedIndicatorMode::showDecimal:
            if (ledIndicatorInfo.showDecimalNumber > 0) {
              // Update the decimal to be shown
              LedIndicatorShowNewDecimal(ledIndicatorInfo);
            }
            else {
              // Value is 0 so stop indication
              ledIndicatorInfo.mode = LedIndicatorMode::none;
              return;
            }
            break;
          }
        }
        ledIndicatorInfo.cadencePulseCounter = 0;
      }

      // Turn on the LED for the on-duration
      ledIndicatorInfo.ledOn = true;
      ledIndicatorInfo.eventDuration = ledIndicatorInfo.cadence[ledIndicatorInfo.cadenceIndex].onDuration;
      ledIndicatorInfo.cadencePulseCounter++;
    }
  }
}

void Blinker() {
  RunLedIndicator(_ledIndicatorInfo[0]);
  RunLedIndicator(_ledIndicatorInfo[1]);

  // Multiplex the LEDs on one output
  static bool startFirstMultiplexInterval = true;
  static unsigned long multiplexIntervalStartTime = 0;
  // static uint8_t multiplexSelector = 0;
  static LedIndicatorLedNumber multiplexSelector = LedIndicatorLedNumber::Led1;
  if (RunAtFrequency(startFirstMultiplexInterval, multiplexIntervalStartTime, _blinkerFrequency)) {
    if (_ledIndicatorInfo[(int)multiplexSelector].ledOn) {
      digitalWrite(_statusLedsPin, multiplexSelector == LedIndicatorLedNumber::Led1 ? 1 : 0);
      pinMode(_statusLedsPin, OUTPUT);
    }
    else {
      pinMode(_statusLedsPin, INPUT);
    }
    if (multiplexSelector == LedIndicatorLedNumber::Led1) {
      multiplexSelector = LedIndicatorLedNumber::Led2;
    }
    else {
      multiplexSelector = LedIndicatorLedNumber::Led1;
    }
  }
}

void LedIndicatorStartCadence(LedIndicatorInfo& ledIndicatorInfo, bool isInterruptCadence = false) {
  if (isInterruptCadence) {
    // This cadence will interrupt any running cadence and prohibit new cadences from being started
    ledIndicatorInfo.interruptCadenceRunning = true;
  }
  else {
    // Normal cadence
    // If interrupt cadence is running do not start a new cadence
    if (ledIndicatorInfo.interruptCadenceRunning) {
      return;
    }
  }
  ledIndicatorInfo.ledOn = true;
  ledIndicatorInfo.eventStartTime = _currentTime;
  ledIndicatorInfo.eventDuration = ledIndicatorInfo.cadence[0].onDuration;
  ledIndicatorInfo.cadenceIndex = 0;
  ledIndicatorInfo.cadencePulseCounter = 1;
}

void LedIndicatorAlive() {
  _ledIndicatorInfo[1].mode = LedIndicatorMode::recurringCadence;
  _ledIndicatorInfo[1].cadence[0].numberOfPulses = 1;
  _ledIndicatorInfo[1].cadence[0].onDuration = 200;
  _ledIndicatorInfo[1].cadence[0].offDuration = 600;
  _ledIndicatorInfo[1].cadence[0].endDuration = 0;
  _ledIndicatorInfo[1].cadenceLength = 1;
  LedIndicatorStartCadence(_ledIndicatorInfo[1]);
}

void LedIndicatorRegulationAction() {
  _ledIndicatorInfo[0].cadence[0].numberOfPulses = 1;
  _ledIndicatorInfo[0].cadence[0].onDuration = 11;
  _ledIndicatorInfo[0].cadence[0].offDuration = 2000;
  _ledIndicatorInfo[0].cadence[0].endDuration = 0;
  _ledIndicatorInfo[0].cadenceLength = 1;
  // We interrupt the current indication on LED 1 to show regulation action
  // It will be restarted after the interrupt
  LedIndicatorStartCadence(_ledIndicatorInfo[0], true);
}

void LedIndicatorShowDecimal(LedIndicatorLedNumber ledIndicatorLedNumber, int decimal) {
  LedIndicatorInfo& ledIndicatorInfo = _ledIndicatorInfo[(int)ledIndicatorLedNumber];
  // Always update the decimal
  // If a decimal is being shown, the new value will be shown afterwards
  // If the new value is 0 the indicator will stop
  ledIndicatorInfo.showDecimalNumber = decimal;
  // Immediately show the decimal if mode was not yet showDecimal
  // Of course we only show non-zero values
  if (decimal > 0 && ledIndicatorInfo.mode != LedIndicatorMode::showDecimal) {
    ledIndicatorInfo.mode = LedIndicatorMode::showDecimal;
    LedIndicatorShowNewDecimal(ledIndicatorInfo);
    LedIndicatorStartCadence(ledIndicatorInfo);
  }
}

void LedIndicatorBurst() {
  _ledIndicatorInfo[1].mode = LedIndicatorMode::singleCadence;
  _ledIndicatorInfo[1].cadence[0].numberOfPulses = 10;
  _ledIndicatorInfo[1].cadence[0].onDuration = 15;
  _ledIndicatorInfo[1].cadence[0].offDuration = 30;
  _ledIndicatorInfo[1].cadence[0].endDuration = 0;
  _ledIndicatorInfo[1].cadenceLength = 1;
  LedIndicatorStartCadence(_ledIndicatorInfo[1]);
}