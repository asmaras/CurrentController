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

namespace Constants {
  namespace RegulationParameters {
    namespace Initial {
      // Initial regulation parameters
      // These can be overriden by values configured in the settings menu
      constexpr int powerOnDelay = 15;
      constexpr int minimumCurrentInmA = 14000;//4000;
      constexpr int currentDeadBandInmA = 500;
      constexpr int currentHardLimitInmA = 15000;//8000;
      constexpr int currentIntegralRegulationGain = 7; // In Hz/A
    }
    namespace Runtime {
      constexpr int currentSensorOversamplingNumberOfBits = 13;
      constexpr int currentIntegralRegulationFrequencyMax = 30;
      constexpr int currentIntegralRegulationFrequencyMin = 1;
      constexpr int safeModeTime = 3;
    }
  }
  namespace HardwareProperties {
    // This is the mA range of the sensor covered by ADC values 0 to max
    // The maximum current rating of the sensor may be less
    // We use this value when no calibration has been done
    constexpr uint16_t currentSensorRangeInmA = 50000;
  }
  namespace Frequencies {
    constexpr int currentMeasureFrequency = 1000;
    constexpr int buttonReadFrequency = 20;
    constexpr int blinkerFrequency = 200;
  }
  namespace Pins {
    constexpr uint8_t currentSensorAndButton = A0;
    constexpr int currentSensorAndButtonThresholdForButton = 950;
    constexpr uint8_t enableCurrentRegulationNot = 3;
    constexpr uint8_t statusLeds = 4;
  }
  namespace EEPROMIndices {
    constexpr int version = 0;
    constexpr int currentSensorCalibrationValues = 1;
    constexpr int minimumCurrent = 9;
    constexpr int currentDeadBand = 11;
    constexpr int currentHardLimit = 13;
    constexpr int currentIntegralRegulationGain = 15;
    constexpr int powerOnDelay = 17;
  };
}

struct ApplicationVariables {
  struct CurrentSensorCalibrationValues {
    uint16_t referenceCurrentInmA;
    uint16_t adcReferencePoint;
    uint16_t slopeCurrentDeltaInmA;
    uint16_t slopeAdcDelta;
  };

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
    bool inverted;
    LedIndicatorMode mode;
    // bool interruptCadenceRunning;
    unsigned long eventStartTime;
    uint16_t eventDuration;
    CadenceElement cadence[5];
    uint8_t cadenceLength;
    uint8_t cadenceIndex;
    uint8_t cadencePulseCounter;
    int showDecimalNumber;
  };
  // Some constants for making LED indicator indexing easier
  struct LedIndicatorLedNumber {
    constexpr static uint8_t led1 = 0;
    constexpr static uint8_t led2 = 1;
  };

  CurrentSensorCalibrationValues currentSensorCalibrationValues;
  int minimumCurrentInmA;
  int currentDeadBandInmA;
  int currentHardLimitInmA;
  int currentIntegralRegulationGain;
  unsigned long currentTime;
  int powerOnDelay;
  bool powerOnDelayCompleted;
  bool debugMode;
  bool regulationInhibited;
  int currentSensorReadingCounter;
  uint32_t currentSensorReadingOversampledValueAccumulator;
  uint16_t currentSensorReadingLatestOversampledValue;
  bool currentSensorReadingSuspended;
  int currentSensorReadingSuspendedCountdown;
  bool safeMode;
  unsigned long safeModeStartTime;
  int aggregatePotentiometerValue;
  LedIndicatorInfo ledIndicatorInfo[2];
} applicationVariables = {};

void HandlePowerOnDelayCompleted();
void MeasureAndRegulate();
void HandleButtonInput();
void EnableRegulation();
void DisableRegulation();
void SetLowestCurrent();
void IncrementCurrent();
void DecrementCurrent();
void SetPotentiometers();
void Blinker();
void LedIndicatorStop();
void LedIndicatorAlive();
void LedIndicatorRegulationAction();
enum class LedIndicatorShowDecimalMode { waitUntilCadenceComplete, showImmediately };
void LedIndicatorShowDecimal(int ledIndicatorLedNumber, int decimal, LedIndicatorShowDecimalMode ledIndicatorShowDecimalMode);
void LedIndicatorBurst(int duration, int endDuration);
void LedIndicatorPause(int duration);
void LedIndicatorInvert(bool invert);
void SetInitialSettings();
void StoreSettings();

void setup() {
  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  // Set combined pin for current sensor and button as input
  pinMode(Constants::Pins::currentSensorAndButton, INPUT);
  // Set combined pin for the two status LEDs to tri-state
  pinMode(Constants::Pins::statusLeds, INPUT);
  // Initially set regulation to the lowest current
  // During the power-on delay regulation is also disabled by hardware
  // that uses an external pull-up resistor
  // Regulation can not take place before we set a pin low from software
  SetLowestCurrent();
  // Show alive activity
  LedIndicatorAlive();

  // Get configuration from EEPROM
  // The only version we know is 1
  if (EEPROM.read(Constants::EEPROMIndices::version) == 1) {
    EEPROM.get(Constants::EEPROMIndices::currentSensorCalibrationValues, applicationVariables.currentSensorCalibrationValues);
    EEPROM.get(Constants::EEPROMIndices::minimumCurrent, applicationVariables.minimumCurrentInmA);
    EEPROM.get(Constants::EEPROMIndices::currentDeadBand, applicationVariables.currentDeadBandInmA);
    EEPROM.get(Constants::EEPROMIndices::currentHardLimit, applicationVariables.currentHardLimitInmA);
    EEPROM.get(Constants::EEPROMIndices::currentIntegralRegulationGain, applicationVariables.currentIntegralRegulationGain);
    EEPROM.get(Constants::EEPROMIndices::powerOnDelay, applicationVariables.powerOnDelay);
  }
  else {
    SetInitialSettings();
  }
}

void loop() {
  applicationVariables.currentTime = micros();

  // Check power-on delay
  if (!applicationVariables.powerOnDelayCompleted && (applicationVariables.currentTime / 1000000 >= (unsigned)applicationVariables.powerOnDelay)) {
    HandlePowerOnDelayCompleted();
  }

  if (applicationVariables.powerOnDelayCompleted) {
    MeasureAndRegulate();
  }
  
  HandleButtonInput();

  Blinker();
}

void HandlePowerOnDelayCompleted() {
    applicationVariables.powerOnDelayCompleted = true;
  // Enable regulation
  // This pin is externaly pulled up so at power-up regulation is always disabled
  // From software we can set this pin low to enable regulation
  // First set value then set pin as output
  EnableRegulation();
  pinMode(Constants::Pins::enableCurrentRegulationNot, OUTPUT);
}

bool RunAtFrequency(bool& startFirstInterval, unsigned long& startTime, int frequency) {
  // This function allows software to run at a specified frequency
  // The startFirstInterval parameter is used to immediately run software (return true) the first
  // time this function is called
  // It must be maintained by the caller between function calls
  if (startFirstInterval) {
    startTime = applicationVariables.currentTime;
    startFirstInterval = false;
    return true;
  }
  else {
    unsigned long interval = 1000000 / frequency;
    if (applicationVariables.currentTime - startTime >= interval) {
      // If we are within the next interval, compensate for clock skew by adding exactly one interval time
      if (applicationVariables.currentTime - startTime < interval * 2) {
        startTime += interval;
      }
      else {
        startTime = applicationVariables.currentTime;
      }
      return true;
    }
    else {
      return false;
    }
  }
}

void CurrentSensorNewOversamplingCycle() {
  applicationVariables.currentSensorReadingCounter = 0;
  applicationVariables.currentSensorReadingOversampledValueAccumulator = 0;
}

void SuspendCurrentSensorReading() {
  applicationVariables.currentSensorReadingSuspended = true;
  applicationVariables.currentSensorReadingSuspendedCountdown = 2;
}

void ResumeCurrentSensorReadingIfSuspended() {
  if (applicationVariables.currentSensorReadingSuspended) {
    if (--applicationVariables.currentSensorReadingSuspendedCountdown == 0) {
      applicationVariables.currentSensorReadingSuspended = false;
      CurrentSensorNewOversamplingCycle();
    }
  }
}

bool GetCurrentSensorAnalogValue(int& value) {
  if (applicationVariables.currentSensorReadingSuspended) {
      return false;
  }
  else {
    value = analogRead(Constants::Pins::currentSensorAndButton);
    if (value < Constants::Pins::currentSensorAndButtonThresholdForButton) {
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
  constexpr int numberOfExtraBits = Constants::RegulationParameters::Runtime::currentSensorOversamplingNumberOfBits - 10;
  constexpr int numberOfReadings = 1 << (numberOfExtraBits * 2); // 4 ^ numberOfExtraBits

  // Add 10-bit reading
  int reading10Bit;
  if(GetCurrentSensorAnalogValue(reading10Bit)) {
    applicationVariables.currentSensorReadingOversampledValueAccumulator += reading10Bit;
    applicationVariables.currentSensorReadingCounter++;
    if (applicationVariables.currentSensorReadingCounter == numberOfReadings) {
      // Decimate accumulated readings
      applicationVariables.currentSensorReadingOversampledValueAccumulator >>= numberOfExtraBits;
      applicationVariables.currentSensorReadingLatestOversampledValue = applicationVariables.currentSensorReadingOversampledValueAccumulator;

      // Calculate current
      // Things could go negative so cast unsigned to signed
      // Things could also go big so cast to 32 bit
      // We use parenthesis around every part to be perfectly clear about computation order
      currentInmA =
        applicationVariables.currentSensorCalibrationValues.referenceCurrentInmA +
        (
          (
            ((int32_t)applicationVariables.currentSensorReadingOversampledValueAccumulator - (int32_t)applicationVariables.currentSensorCalibrationValues.adcReferencePoint) * // ADC deviation from reference point
            (int32_t)applicationVariables.currentSensorCalibrationValues.slopeCurrentDeltaInmA // Multiply with current delta per ADC delta
                                                                                               // Part 1: first do numerator part (current delta) of this ratio to keep precision
            ) /
          (int32_t)applicationVariables.currentSensorCalibrationValues.slopeAdcDelta // Multiply with current delta per ADC delta
                                                                                     // Part 2: the denominator part (ADC delta) of the ratio, therefore a division
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
  // Things could go big so cast to 32 bit
  // We use parenthesis around every part to be perfectly clear about computation order
  int changeFrequency =
    (int32_t)Constants::RegulationParameters::Runtime::currentIntegralRegulationFrequencyMin +
    (
      ((int32_t)deviationInmA * (int32_t)applicationVariables.currentIntegralRegulationGain) / 1000UL
      );
  if (changeFrequency > Constants::RegulationParameters::Runtime::currentIntegralRegulationFrequencyMax) {
    return Constants::RegulationParameters::Runtime::currentIntegralRegulationFrequencyMax;
  }
  else {
    return changeFrequency;
  }
}

void GoToSafeMode() {
  if (!applicationVariables.safeMode) {
    // Set _safeMode first so that any function called below can call GoToSafeMode again without causing a loop
    applicationVariables.safeMode = true;
    applicationVariables.safeModeStartTime = applicationVariables.currentTime;

    // Activate the regulation disable output
    // As an extra safety also set regulation to lowest possible current
    DisableRegulation();
    SetLowestCurrent();
  }
}

void MeasureAndRegulate() {
  static bool startFirstCurrentMeasureInterval = true;
  static unsigned long currentMeasureIntervalStartTime = 0;
  int currentInmA = 0;
  static enum class CurrentIntegralRegulationAction {
    none, up, down
  } currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
  bool changeCurrentImmediately = false;
  static int changeFrequency = 0;
  // Perform sensor measurements at a fixed frequency
  if (RunAtFrequency(startFirstCurrentMeasureInterval, currentMeasureIntervalStartTime, Constants::Frequencies::currentMeasureFrequency)) {
    if (GetCurrentSensorValueInmA(currentInmA)) {
      if (applicationVariables.safeMode) {
        // In safe mode
        // Go back to regulation mode when the current stays under the minimum for some time
        if (
          currentInmA < applicationVariables.minimumCurrentInmA &&
          (applicationVariables.currentTime - applicationVariables.safeModeStartTime >= Constants::RegulationParameters::Runtime::safeModeTime * 1000000UL)
          ) {
          applicationVariables.safeMode = false;
          EnableRegulation();
        }
      }
      else {
        if (!applicationVariables.regulationInhibited) {
          // In normal regulation mode
          if (currentInmA < applicationVariables.minimumCurrentInmA) {
            // Current too low, regulate up
            if (currentIntegralRegulationAction != CurrentIntegralRegulationAction::up) {
              changeCurrentImmediately = true;
            }
            currentIntegralRegulationAction = CurrentIntegralRegulationAction::up;
            changeFrequency = CalculateCurrentChangeFrequency(currentInmA, applicationVariables.minimumCurrentInmA);
          }
          else if (currentInmA > applicationVariables.currentHardLimitInmA) {
            // Current dangerously high, go to safe mode
            GoToSafeMode();
          }
          else if (currentInmA > applicationVariables.minimumCurrentInmA + applicationVariables.currentDeadBandInmA) {
            // Current is too high, regulate down
            if (currentIntegralRegulationAction != CurrentIntegralRegulationAction::down) {
              changeCurrentImmediately = true;
            }
            currentIntegralRegulationAction = CurrentIntegralRegulationAction::down;
            changeFrequency = CalculateCurrentChangeFrequency(currentInmA, applicationVariables.minimumCurrentInmA + applicationVariables.currentDeadBandInmA);
          }
          else {
            // Current is within dead band, do nothing
            currentIntegralRegulationAction = CurrentIntegralRegulationAction::none;
          }
        }
      }

      LedIndicatorShowDecimal(ApplicationVariables::LedIndicatorLedNumber::led1, currentInmA / 100, LedIndicatorShowDecimalMode::waitUntilCadenceComplete);
    }
  }

  // Integral regulation actions are done at their own frequency, independent from sensor measurements
  if (!applicationVariables.safeMode && currentIntegralRegulationAction != CurrentIntegralRegulationAction::none) {
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
  digitalWrite(Constants::Pins::enableCurrentRegulationNot, 0);
}

void DisableRegulation() {
  digitalWrite(Constants::Pins::enableCurrentRegulationNot, 1);
}

void SetLowestCurrent() {
  applicationVariables.aggregatePotentiometerValue = 0;
  SetPotentiometers();
}

void IncrementCurrent() {
  if (applicationVariables.aggregatePotentiometerValue < 512) {
    applicationVariables.aggregatePotentiometerValue++;
    SetPotentiometers();
  }
  LedIndicatorRegulationAction();
}

void DecrementCurrent() {
  if (applicationVariables.aggregatePotentiometerValue > 0) {
    applicationVariables.aggregatePotentiometerValue--;
    SetPotentiometers();
  }
}

void SetPotentiometers() {
  // MCP4251 is used, see https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22060b.pdf
  SPI.transfer16((applicationVariables.aggregatePotentiometerValue + 1) / 2);
  SPI.transfer16(0x1000 | (applicationVariables.aggregatePotentiometerValue / 2));

  if (!applicationVariables.debugMode) {
    // Check value of wiper 0
    // Expected are 7 bits set to "1" followed by the value
    if (SPI.transfer16(0x0C00) != (0xFE00 | ((applicationVariables.aggregatePotentiometerValue + 1) / 2))) {
      GoToSafeMode();
    }
  }
}

void HandleButtonInput() {
  // Read button input and test for short and long presses
  static bool buttonDepressed = false;
  static unsigned long buttonDepressedStartTime = 0;
  static bool buttonLongPressDetected = false;
  enum class ButtonEvent {none, shortPress, longPress} buttonEvent = ButtonEvent::none;
  static bool startFirstButtonReadInterval = true;
  static unsigned long buttonReadIntervalStartTime = 0;
  // Debouncing by periodic reading
  if (RunAtFrequency(startFirstButtonReadInterval, buttonReadIntervalStartTime, Constants::Frequencies::buttonReadFrequency)) {
    // The button is connected to an analogue input and shares the pin with the current sensor
    // Check for threshold value clearly indicating the button is pressed
    if (analogRead(Constants::Pins::currentSensorAndButton) >= Constants::Pins::currentSensorAndButtonThresholdForButton) {
      // Because the pin is shared suspend sensor reading during button presses
      SuspendCurrentSensorReading();

      if (buttonDepressed) {
        if (!buttonLongPressDetected && applicationVariables.currentTime - buttonDepressedStartTime >= 1000000UL) {
          buttonEvent = ButtonEvent::longPress;
          buttonLongPressDetected = true;
        }
      }
      else {
        buttonDepressed = true;
        buttonDepressedStartTime = applicationVariables.currentTime;
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

  // Long press during power-on delay switches debug mode on
  if (!applicationVariables.powerOnDelayCompleted && buttonEvent == ButtonEvent::longPress) {
    applicationVariables.debugMode = true;
    HandlePowerOnDelayCompleted();
  }

  // Define lambda function for handling up/down parameters
  enum class HandleUpDownParameterResult { working, parameterChanged, done };
  auto HandleUpDownParameter = [&buttonEvent](int& parameter, int delta = 1, int scaleDownForLedIndicator = 1) {
    static bool handlingParameter = false;
    static enum class Direction { up, down } direction = Direction::up;
    static int consecutiveLongPresses = 0;

    // If not yet handling a parameter, start showing the parameter value
    if (!handlingParameter) {
      // The indicator may be showing a burst on entry of the parameter setting
      // Show the decimal after that cadence has completed
      LedIndicatorShowDecimal(ApplicationVariables::LedIndicatorLedNumber::led2, parameter / scaleDownForLedIndicator, LedIndicatorShowDecimalMode::waitUntilCadenceComplete);
      handlingParameter = true;
    }
    if (buttonEvent == ButtonEvent::shortPress) {
      if (direction == Direction::up) {
        parameter += delta;
      }
      else {
        if (parameter - delta >= 0) {
          parameter -= delta;
        }
      }
      // Immediately show the updated parameter
      LedIndicatorShowDecimal(ApplicationVariables::LedIndicatorLedNumber::led2, parameter / scaleDownForLedIndicator, LedIndicatorShowDecimalMode::showImmediately);
      // Reset because of short press
      consecutiveLongPresses = 0;
      return HandleUpDownParameterResult::parameterChanged;
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      // Two consecutive long presses end this parameter setting
      if (++consecutiveLongPresses == 2) {
        LedIndicatorStop();

        // Reset for next parameter
        handlingParameter = false;
        direction = Direction::up;
        consecutiveLongPresses = 0;
        return HandleUpDownParameterResult::done;
      }
      else {
        // Change direction on long press
        if (direction == Direction::up) {
          direction = Direction::down;
        }
        else {
          direction = Direction::up;
        }
        LedIndicatorInvert(direction == Direction::down);
        // To improve "readability" of the indicator after inversion, pause for a while before
        // the parameter is shown again
        LedIndicatorPause(1000);
      }
    }

    return HandleUpDownParameterResult::working;
  };

  // Handle the menu
  static enum class MenuMode {
    off,
    minimumCurrent, minimumCurrentSetting,
    deadBand, deadBandSetting,
    hardLimitCurrent, hardLimitCurrentSetting,
    powerOnDelay, powerOnDelaySetting,
    twoPointCalibration, twoPointCalibrationPoint1SetValue, twoPointCalibrationPoint1ControlOutputCoarse, twoPointCalibrationPoint1ControlOutputFine, twoPointCalibrationPoint2SetValue, twoPointCalibrationPoint2ControlOutputCoarse, twoPointCalibrationPoint2ControlOutputFine,
    integralRegulationGain, integralRegulationGainSetting,
    testDisableRegulation, testDisableRegulationSetting,
    resetToInitialSettings, resetToInitialSettingsShowingBurst
  } menuMode = MenuMode::off;
  // Some lambda's for stepping through the menu
  auto NextMenuMode = [](MenuMode nextMenuMode, bool showBurst, int burstDuration, bool showDecimal, int decimal) {
    if (showBurst) {
      // Start burst and, in case a decimal needs to be shown, setup to show it after the burst
      LedIndicatorBurst(burstDuration, 1000);
      if (showDecimal) {
        LedIndicatorShowDecimal(ApplicationVariables::LedIndicatorLedNumber::led2, decimal, LedIndicatorShowDecimalMode::waitUntilCadenceComplete);
      }
    }
    else {
      if (showDecimal) {
        // No burst, immediately show the decimal even if a decimal was being shown
        LedIndicatorShowDecimal(ApplicationVariables::LedIndicatorLedNumber::led2, decimal, LedIndicatorShowDecimalMode::showImmediately);
      }
      else {
        LedIndicatorStop();
      }
    }
    menuMode = nextMenuMode;
  };
  auto NextMenuModeWithBurstAndDecimal = [NextMenuMode](MenuMode nextMenuMode, int burstDuration, int decimal) {
    NextMenuMode(nextMenuMode, true, burstDuration, true, decimal);
  };
  auto NextMenuModeWithBurst = [NextMenuMode](MenuMode nextMenuMode, int burstDuration) {
    NextMenuMode(nextMenuMode, true, burstDuration, false, 0);
  };
  auto NextMenuModeWithDecimal = [NextMenuMode](MenuMode nextMenuMode, int decimal) {
    NextMenuMode(nextMenuMode, false, 0, true, decimal);
  };
  static int integerSettingParameter = 0;
  static bool booleanSettingParameter = false;
  switch (menuMode) {
  case MenuMode::off:
    if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurstAndDecimal(MenuMode::minimumCurrent, 1000, 1);
    }
    break;
  case MenuMode::minimumCurrent:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::deadBand, 2);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurst(MenuMode::minimumCurrentSetting, 250);
    }
    break;
  case MenuMode::minimumCurrentSetting:
    if (HandleUpDownParameter(applicationVariables.minimumCurrentInmA, 100, 100) == HandleUpDownParameterResult::done) {
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::minimumCurrent, 3000, 1);
    }
    break;
  case MenuMode::deadBand:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::hardLimitCurrent, 3);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurst(MenuMode::deadBandSetting, 250);
    }
    break;
  case MenuMode::deadBandSetting:
    if (HandleUpDownParameter(applicationVariables.currentDeadBandInmA, 100, 100) == HandleUpDownParameterResult::done) {
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::deadBand, 3000, 2);
    }
    break;
  case MenuMode::hardLimitCurrent:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::powerOnDelay, 4);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurst(MenuMode::hardLimitCurrentSetting, 250);
    }
    break;
  case MenuMode::hardLimitCurrentSetting:
    if (HandleUpDownParameter(applicationVariables.currentHardLimitInmA, 100, 100) == HandleUpDownParameterResult::done) {
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::hardLimitCurrent, 3000, 3);
    }
    break;
  case MenuMode::powerOnDelay:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::twoPointCalibration, 5);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurst(MenuMode::powerOnDelaySetting, 250);
    }
    break;
  case MenuMode::powerOnDelaySetting:
    if (HandleUpDownParameter(applicationVariables.powerOnDelay) == HandleUpDownParameterResult::done) {
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::powerOnDelay, 3000, 4);
    }
    break;
  case MenuMode::twoPointCalibration:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::integralRegulationGain, 6);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      applicationVariables.regulationInhibited = true;
      applicationVariables.aggregatePotentiometerValue = 0;
      SetPotentiometers();
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint1SetValue, 250);
    }
    break;
  case MenuMode::twoPointCalibrationPoint1SetValue:
    if (HandleUpDownParameter((int&)applicationVariables.currentSensorCalibrationValues.referenceCurrentInmA, 1000, 100) == HandleUpDownParameterResult::done) {
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint1ControlOutputCoarse, 250);
    }
    break;
  case MenuMode::twoPointCalibrationPoint1ControlOutputCoarse:
    switch (HandleUpDownParameter(applicationVariables.aggregatePotentiometerValue, 10)) {
    case HandleUpDownParameterResult::parameterChanged:
      SetPotentiometers();
      break;
    case HandleUpDownParameterResult::done:
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint1ControlOutputFine, 250);
      break;
    case HandleUpDownParameterResult::working:
      break;
    }
    break;
  case MenuMode::twoPointCalibrationPoint1ControlOutputFine:
    switch (HandleUpDownParameter(applicationVariables.aggregatePotentiometerValue, 1)) {
    case HandleUpDownParameterResult::parameterChanged:
      SetPotentiometers();
      break;
    case HandleUpDownParameterResult::done:
      applicationVariables.currentSensorCalibrationValues.adcReferencePoint = applicationVariables.currentSensorReadingLatestOversampledValue;
      integerSettingParameter = applicationVariables.currentSensorCalibrationValues.referenceCurrentInmA + 1000;
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint2SetValue, 250);
      break;
    case HandleUpDownParameterResult::working:
      break;
    }
    break;
  case MenuMode::twoPointCalibrationPoint2SetValue:
    if (HandleUpDownParameter(integerSettingParameter, 1000, 100) == HandleUpDownParameterResult::done) {
      applicationVariables.currentSensorCalibrationValues.slopeCurrentDeltaInmA = integerSettingParameter - applicationVariables.currentSensorCalibrationValues.referenceCurrentInmA;
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint2ControlOutputCoarse, 250);
    }
    break;
  case MenuMode::twoPointCalibrationPoint2ControlOutputCoarse:
    switch (HandleUpDownParameter(applicationVariables.aggregatePotentiometerValue, 10)) {
    case HandleUpDownParameterResult::parameterChanged:
      SetPotentiometers();
      break;
    case HandleUpDownParameterResult::done:
      NextMenuModeWithBurst(MenuMode::twoPointCalibrationPoint2ControlOutputFine, 250);
      break;
    case HandleUpDownParameterResult::working:
      break;
    }
    break;
  case MenuMode::twoPointCalibrationPoint2ControlOutputFine:
    switch (HandleUpDownParameter(applicationVariables.aggregatePotentiometerValue, 1)) {
    case HandleUpDownParameterResult::parameterChanged:
      SetPotentiometers();
      break;
    case HandleUpDownParameterResult::done:
      applicationVariables.currentSensorCalibrationValues.slopeAdcDelta = applicationVariables.currentSensorReadingLatestOversampledValue - applicationVariables.currentSensorCalibrationValues.adcReferencePoint;
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::twoPointCalibration, 3000, 5);
      break;
    case HandleUpDownParameterResult::working:
      break;
    }
    break;
  case MenuMode::integralRegulationGain:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::testDisableRegulation, 7);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      NextMenuModeWithBurst(MenuMode::integralRegulationGainSetting, 250);
    }
    break;
  case MenuMode::integralRegulationGainSetting:
    if (HandleUpDownParameter(applicationVariables.currentIntegralRegulationGain) == HandleUpDownParameterResult::done) {
      StoreSettings();
      NextMenuModeWithBurstAndDecimal(MenuMode::integralRegulationGain, 3000, 6);
    }
    break;
  case MenuMode::testDisableRegulation:
    if (buttonEvent == ButtonEvent::shortPress) {
      NextMenuModeWithDecimal(MenuMode::resetToInitialSettings, 8);
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      // Prepare for testDisableRegulationSetting
      booleanSettingParameter = false;
      NextMenuModeWithBurst(MenuMode::testDisableRegulationSetting, 250);
    }
    break;
  case MenuMode::testDisableRegulationSetting:
    if (buttonEvent == ButtonEvent::shortPress) {
      booleanSettingParameter = !booleanSettingParameter;
      if (booleanSettingParameter) {
        DisableRegulation();
      }
      else {
        EnableRegulation();
      }
      applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].ledOn = booleanSettingParameter;
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      EnableRegulation();
      NextMenuModeWithBurstAndDecimal(MenuMode::testDisableRegulation, 3000, 7);
    }
    break;
  case MenuMode::resetToInitialSettings:
    if (buttonEvent == ButtonEvent::shortPress) {
      LedIndicatorAlive();
      menuMode = MenuMode::off;
    }
    else if (buttonEvent == ButtonEvent::longPress) {
      SetInitialSettings();
      StoreSettings();
      NextMenuModeWithBurst(MenuMode::resetToInitialSettingsShowingBurst, 5000);
    }
    break;
  case MenuMode::resetToInitialSettingsShowingBurst:
    // Wait for the burst to complete before exiting menu
    if (applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].mode == ApplicationVariables::LedIndicatorMode::none) {
      LedIndicatorAlive();
      menuMode = MenuMode::off;
    }
    break;
  }
}

void LedIndicatorShowNewDecimal(ApplicationVariables::LedIndicatorInfo& ledIndicatorInfo) {
  // // If interrupt cadence is running do not set up a new cadence
  // if (ledIndicatorInfo.interruptCadenceRunning) {
  //   return;
  // }
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

void RunLedIndicator(ApplicationVariables::LedIndicatorInfo& ledIndicatorInfo) {
  if (
    (ledIndicatorInfo.mode != ApplicationVariables::LedIndicatorMode::none /*|| ledIndicatorInfo.interruptCadenceRunning*/) &&
    (applicationVariables.currentTime - ledIndicatorInfo.eventStartTime >= (unsigned long)ledIndicatorInfo.eventDuration * 1000UL)
    ) {
    ledIndicatorInfo.eventStartTime = applicationVariables.currentTime;

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
          // if (ledIndicatorInfo.interruptCadenceRunning) {
          //   // This was an interrupt cadence and has now ended
          //   ledIndicatorInfo.interruptCadenceRunning = false;
          // }
          switch (ledIndicatorInfo.mode) {
          case ApplicationVariables::LedIndicatorMode::none:
            // Nothing to do so stop
            return;
          case ApplicationVariables::LedIndicatorMode::singleCadence:
            // This was a single cadence so stop
            ledIndicatorInfo.mode = ApplicationVariables::LedIndicatorMode::none;
            return;
          case ApplicationVariables::LedIndicatorMode::recurringCadence:
            // A recurring cadence is active so continue
            break;
          case ApplicationVariables::LedIndicatorMode::showDecimal:
            if (ledIndicatorInfo.showDecimalNumber > 0) {
              // Update the decimal to be shown
              LedIndicatorShowNewDecimal(ledIndicatorInfo);
            }
            else {
              // Value is 0 so stop indication
              ledIndicatorInfo.mode = ApplicationVariables::LedIndicatorMode::none;
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
  RunLedIndicator(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1]);
  RunLedIndicator(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2]);

  // Multiplex the LEDs on one output
  static bool startFirstMultiplexInterval = true;
  static unsigned long multiplexIntervalStartTime = 0;
  static uint8_t multiplexSelector = ApplicationVariables::LedIndicatorLedNumber::led1;
  if (RunAtFrequency(startFirstMultiplexInterval, multiplexIntervalStartTime, Constants::Frequencies::blinkerFrequency)) {
    if (applicationVariables.ledIndicatorInfo[multiplexSelector].ledOn != applicationVariables.ledIndicatorInfo[multiplexSelector].inverted) {
      digitalWrite(Constants::Pins::statusLeds, multiplexSelector == ApplicationVariables::LedIndicatorLedNumber::led1 ? 1 : 0);
      pinMode(Constants::Pins::statusLeds, OUTPUT);
    }
    else {
      pinMode(Constants::Pins::statusLeds, INPUT);
    }
    if (multiplexSelector == ApplicationVariables::LedIndicatorLedNumber::led1) {
      multiplexSelector = ApplicationVariables::LedIndicatorLedNumber::led2;
    }
    else {
      multiplexSelector = ApplicationVariables::LedIndicatorLedNumber::led1;
    }
  }
}

void LedIndicatorStartCadence(ApplicationVariables::LedIndicatorInfo& ledIndicatorInfo/*, bool isInterruptCadence = false*/) {
  // if (isInterruptCadence) {
  //   // This cadence will interrupt any running cadence and prohibit new cadences from being started
  //   ledIndicatorInfo.interruptCadenceRunning = true;
  // }
  // else {
  //   // Normal cadence
  //   // If interrupt cadence is running do not start a new cadence
  //   if (ledIndicatorInfo.interruptCadenceRunning) {
  //     return;
  //   }
  // }
  ledIndicatorInfo.ledOn = true;
  ledIndicatorInfo.eventStartTime = applicationVariables.currentTime;
  ledIndicatorInfo.eventDuration = ledIndicatorInfo.cadence[0].onDuration;
  ledIndicatorInfo.cadenceIndex = 0;
  ledIndicatorInfo.cadencePulseCounter = 1;
}

void LedIndicatorStop() {
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].ledOn = false;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].inverted = false;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].mode = ApplicationVariables::LedIndicatorMode::none;
}

void LedIndicatorAlive() {
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].mode = ApplicationVariables::LedIndicatorMode::recurringCadence;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].numberOfPulses = 1;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].onDuration = 200;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].offDuration = 600;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].endDuration = 0;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadenceLength = 1;
  LedIndicatorStartCadence(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2]);
}

void LedIndicatorRegulationAction() {
  // This will always be shown immediately and we want the active mode to resume afterwards
  // Therefore the mode is not changed. Only when the mode currently is LedIndicatorMode::none,
  // we set it to LedIndicatorMode::singleCadence so other functions know a cadance is running
  if (applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].mode == ApplicationVariables::LedIndicatorMode::none) {
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].mode = ApplicationVariables::LedIndicatorMode::singleCadence;
  }
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].cadence[0].numberOfPulses = 1;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].cadence[0].onDuration = 11;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].cadence[0].offDuration = 2000;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].cadence[0].endDuration = 0;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1].cadenceLength = 1;
  // // We interrupt the current indication on LED 1 to show regulation action
  // // It will be restarted after the interrupt
  LedIndicatorStartCadence(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led1]/*, false*/);
}

void LedIndicatorShowDecimal(int ledIndicatorLedNumber, int decimal, LedIndicatorShowDecimalMode ledIndicatorShowDecimalMode) {
  ApplicationVariables::LedIndicatorInfo& ledIndicatorInfo = applicationVariables.ledIndicatorInfo[ledIndicatorLedNumber];
  // If LedIndicatorShowDecimalMode::waitUntilCadenceComplete is requested the decimal should only be shown
  // after an active cadance has completed (which could be a decimal that is currently being shown)
  // If LedIndicatorShowDecimalMode::showImmediately is requested the decimal must be shown right away
  // Always update the decimal
  // If it is not immediately shown the indicator will pick it up when it's ready
  ledIndicatorInfo.showDecimalNumber = decimal;
  // For LedIndicatorShowDecimalMode::waitUntilCadenceComplete we should still show the decimal immediately if
  // the indicator is doing nothing at the moment
  if (
    (ledIndicatorShowDecimalMode == LedIndicatorShowDecimalMode::waitUntilCadenceComplete && ledIndicatorInfo.mode == ApplicationVariables::LedIndicatorMode::none) ||
    ledIndicatorShowDecimalMode == LedIndicatorShowDecimalMode::showImmediately
    ) {
    if (decimal > 0) {
      ledIndicatorInfo.mode = ApplicationVariables::LedIndicatorMode::showDecimal;
      LedIndicatorShowNewDecimal(ledIndicatorInfo);
      LedIndicatorStartCadence(ledIndicatorInfo);
    }
    else {
      // Value must be shown immediately, but it is 0
      // Stop indication
      LedIndicatorStop();
    }
  }
  else {
    // Don't show the decimal immediately
    // Just set the indicator mode to LedIndicatorMode::showDecimal so the indicator will pick up the decimal
    // when it has finished the current cadence
    ledIndicatorInfo.mode = ApplicationVariables::LedIndicatorMode::showDecimal;
  }
}

void LedIndicatorBurst(int duration, int endDuration) {
  constexpr int onDuration = 15;
  constexpr int offDuration = 30;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].mode = ApplicationVariables::LedIndicatorMode::singleCadence;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].numberOfPulses = duration / (onDuration + offDuration);
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].onDuration = onDuration;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].offDuration = offDuration;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].endDuration = endDuration;
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadenceLength = 1;
  LedIndicatorStartCadence(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2]);
}

void LedIndicatorPause(int duration) {
  // Pause the current cadence by starting a new cadence with only an offDuration
  // We want the active mode to resume afterwards so don't change the mode
  // Only when a cadence is running currently a pause is useful
  if (applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].mode != ApplicationVariables::LedIndicatorMode::none) {
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].numberOfPulses = 1;
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].onDuration = 0;
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].offDuration = duration;
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadence[0].endDuration = 0;
    applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].cadenceLength = 1;
    LedIndicatorStartCadence(applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2]);
  }
}

void LedIndicatorInvert(bool invert) {
  applicationVariables.ledIndicatorInfo[ApplicationVariables::LedIndicatorLedNumber::led2].inverted = invert;
}

void SetInitialSettings() {
  applicationVariables.currentSensorCalibrationValues.referenceCurrentInmA = 0;
  applicationVariables.currentSensorCalibrationValues.adcReferencePoint = 1 << (Constants::RegulationParameters::Runtime::currentSensorOversamplingNumberOfBits - 1);
  applicationVariables.currentSensorCalibrationValues.slopeCurrentDeltaInmA = Constants::HardwareProperties::currentSensorRangeInmA;
  applicationVariables.currentSensorCalibrationValues.slopeAdcDelta = 1 << Constants::RegulationParameters::Runtime::currentSensorOversamplingNumberOfBits;
  applicationVariables.minimumCurrentInmA = Constants::RegulationParameters::Initial::minimumCurrentInmA;
  applicationVariables.currentDeadBandInmA = Constants::RegulationParameters::Initial::currentDeadBandInmA;
  applicationVariables.currentHardLimitInmA = Constants::RegulationParameters::Initial::currentHardLimitInmA;
  applicationVariables.currentIntegralRegulationGain = Constants::RegulationParameters::Initial::currentIntegralRegulationGain;
  applicationVariables.powerOnDelay = Constants::RegulationParameters::Initial::powerOnDelay;
}

void StoreSettings() {
  EEPROM.write(Constants::EEPROMIndices::version, 1);
  EEPROM.put(Constants::EEPROMIndices::currentSensorCalibrationValues, applicationVariables.currentSensorCalibrationValues);
  EEPROM.put(Constants::EEPROMIndices::minimumCurrent, applicationVariables.minimumCurrentInmA);
  EEPROM.put(Constants::EEPROMIndices::currentDeadBand, applicationVariables.currentDeadBandInmA);
  EEPROM.put(Constants::EEPROMIndices::currentHardLimit, applicationVariables.currentHardLimitInmA);
  EEPROM.put(Constants::EEPROMIndices::currentIntegralRegulationGain, applicationVariables.currentIntegralRegulationGain);
  EEPROM.put(Constants::EEPROMIndices::powerOnDelay, applicationVariables.powerOnDelay);
}