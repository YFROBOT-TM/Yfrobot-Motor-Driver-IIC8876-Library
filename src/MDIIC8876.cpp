/******************************************************************************
  MDIIC8876.cpp
  MotorDriver IIC8876 I/O Expander Library Source File
  Creation Date: 06-11-2024
    @ YFROBOT
******************************************************************************/

#include <Arduino.h>
#include <MDIIC8876.h>

MDIIC8876::MDIIC8876()
{
  _clkX = 0;
}

MDIIC8876::MDIIC8876(uint8_t address, uint8_t resetPin, uint8_t interruptPin, uint8_t oscillatorPin)
{
    // Store the received parameters into member variables
    deviceAddress = address;
    pinInterrupt = interruptPin;
    pinOscillator = oscillatorPin;
    pinReset = resetPin;
}

// 在类的析构函数中清除中断服务例程
MDIIC8876::~MDIIC8876() {
}

uint8_t MDIIC8876::begin(uint8_t address, TwoWire &wirePort, uint8_t resetPin)
{
    // Store the received parameters into member variables
    _i2cPort = &wirePort;
    deviceAddress = address;
    pinReset = resetPin;

    return init();
}

uint8_t MDIIC8876::init(void)
{
    // Begin I2C should be done externally, before beginning MDIIC8876
    Wire.begin();

    // If the reset pin is connected
    if (pinReset != 255)
        reset(1);
    else
        reset(0);

    // 电机驱动引脚配置
    pinMode(M1EN, MDIIC8876_ANALOG_OUTPUT);
    pinMode(M1PH, MDIIC8876_DIGITAL_OUTPUT);
    pinMode(M2EN, MDIIC8876_ANALOG_OUTPUT);
    pinMode(M2PH, MDIIC8876_DIGITAL_OUTPUT);
    pinMode(M3EN, MDIIC8876_ANALOG_OUTPUT);
    pinMode(M3PH, MDIIC8876_DIGITAL_OUTPUT);
    pinMode(M4EN, MDIIC8876_ANALOG_OUTPUT);
    pinMode(M4PH, MDIIC8876_DIGITAL_OUTPUT);

    // Communication test. We'll read from two registers with different
    // default values to verify communication.
    uint8_t testRegisters = readByte(MDIIC8876_REG_INTERRUPT_MASK); // This should return 0xFF, Interrupt mask register address 0x09

    if (testRegisters == 0xFF)
    {
        // Set the clock to a default of 2MHz using internal
        clock(MDIIC8876_INTERNAL_CLOCK_2MHZ);
        return 1;
    }

    return 0;
}

void MDIIC8876::reset(bool hardware)
{
  // if hardware bool is set
  if (hardware) {
    // Check if bit 2 of MDIIC8876_REG_MISC is set
    // if so nReset will not issue a POR, we'll need to clear that bit first
    uint8_t regMisc = readByte(MDIIC8876_REG_MISC);
    if (regMisc & (1 << 2)) {
      regMisc &= ~(1 << 2);
      writeByte(MDIIC8876_REG_MISC, regMisc);
    }
    // Reset the MDIIC8876, the pin is active low
    pinMode(pinReset, OUTPUT);	  // set reset pin as output
    digitalWrite(pinReset, LOW);  // pull reset pin low
    delay(1);					  // Wait for the pin to settle
    digitalWrite(pinReset, HIGH); // pull reset pin back high
  } else {
    // Software reset command sequence:
    writeByte(MDIIC8876_REG_RESET, 0x12);
    writeByte(MDIIC8876_REG_RESET, 0x34);
  }
}

void MDIIC8876::pinDir(uint8_t pin, uint8_t inOut, uint8_t initialLevel)
{
  // The MDIIC8876 RegDir registers: MDIIC8876_REG_DIR, MDIIC8876_REG_DIR
  //	0: IO is configured as an output
  //	1: IO is configured as an input
  uint8_t modeBit;
  if ((inOut == OUTPUT) || (inOut == MDIIC8876_ANALOG_OUTPUT)) {
    uint8_t tempRegData = readByte(MDIIC8876_REG_DATA);
    if (initialLevel == LOW) {
      tempRegData &= ~(1 << pin);
      writeByte(MDIIC8876_REG_DATA, tempRegData);
    }
    modeBit = 0;
  } else {
    modeBit = 1;
  }

  uint8_t tempRegDir = readByte(MDIIC8876_REG_DIR);
  if (modeBit)
    tempRegDir |= (1 << pin);
  else
    tempRegDir &= ~(1 << pin);
  writeByte(MDIIC8876_REG_DIR, tempRegDir);

  // If INPUT_PULLUP was called, set up the pullup too:
  if (inOut == INPUT_PULLUP)
    writePin(pin, HIGH);

  if (inOut == MDIIC8876_ANALOG_OUTPUT) {
    ledDriverInit(pin);
  }
}

void MDIIC8876::pinMode(uint8_t pin, uint8_t inOut, uint8_t initialLevel)
{
  pinDir(pin, inOut, initialLevel);
}

bool MDIIC8876::writePin(uint8_t pin, uint8_t highLow)
{
  uint8_t tempRegDir = readByte(MDIIC8876_REG_DIR);
  if ((0xFF ^ tempRegDir) & (1 << pin)) { // If the pin is an output, write high/low
    uint8_t tempRegData = readByte(MDIIC8876_REG_DATA);
    if (highLow)
      tempRegData |= (1 << pin);
    else
      tempRegData &= ~(1 << pin);
    return writeByte(MDIIC8876_REG_DATA, tempRegData);
  } else { // Otherwise the pin is an input, pull-up/down
    uint8_t tempPullUp = readByte(MDIIC8876_REG_PULL_UP);
    uint8_t tempPullDown = readByte(MDIIC8876_REG_PULL_DOWN);

    if (highLow) { // if HIGH, do pull-up, disable pull-down
      tempPullUp |= (1 << pin);
      tempPullDown &= ~(1 << pin);
      return writeByte(MDIIC8876_REG_PULL_UP, tempPullUp) && writeByte(MDIIC8876_REG_PULL_DOWN, tempPullDown);
    } else { // If LOW do pull-down, disable pull-up
      tempPullDown |= (1 << pin);
      tempPullUp &= ~(1 << pin);
      return writeByte(MDIIC8876_REG_PULL_UP, tempPullUp) && writeByte(MDIIC8876_REG_PULL_DOWN, tempPullDown);
    }
  }
}

bool MDIIC8876::digitalWrite(uint8_t pin, uint8_t highLow)
{
  return writePin(pin, highLow);
}

uint8_t MDIIC8876::readPin(uint8_t pin)
{
  uint8_t tempRegDir = readByte(MDIIC8876_REG_DIR);

  if (tempRegDir & (1 << pin)) // If the pin is an input
    //   if (1 << pin) // If the pin is an input
  {
    uint8_t tempRegData = readByte(MDIIC8876_REG_DATA);
    if (tempRegData & (1 << pin))
      return 1;
  }
  else
  {
    // log_d("Pin %d not INPUT, MDIIC8876_REG_DIR: %d", pin, tempRegDir);
  }

  return 0;
}

bool MDIIC8876::readPin(const uint8_t pin, bool *value)
{
  uint8_t tempRegDir;
  if (readByte(MDIIC8876_REG_DIR, &tempRegDir)) {
    if (tempRegDir & (1 << pin)) { // If the pin is an input
      uint8_t tempRegData;
      if (readByte(MDIIC8876_REG_DATA, &tempRegData)) {
        *value = (tempRegData & (1 << pin)) != 0;
        return true;
      };
    }
    else
    {
      *value = false;
      return true;
    }
  }
  return false;
}

uint8_t MDIIC8876::digitalRead(uint8_t pin)
{
  return readPin(pin);
}

bool MDIIC8876::digitalRead(uint8_t pin, bool *value)
{
  return readPin(pin, value);
}

void MDIIC8876::ledDriverInit(uint8_t pin, uint8_t freq /*= 1*/, bool log /*= false*/)
{
  uint8_t tempByte;

  // Disable input buffer
  // Writing a 1 to the pin bit will disable that pins input buffer
  tempByte = readByte(MDIIC8876_REG_INPUT_DISABLE);
  tempByte |= (1 << pin);
  writeByte(MDIIC8876_REG_INPUT_DISABLE, tempByte);

  // Disable pull-up
  // Writing a 0 to the pin bit will disable that pull-up resistor
  tempByte = readByte(MDIIC8876_REG_PULL_UP);
  tempByte &= ~(1 << pin);
  writeByte(MDIIC8876_REG_PULL_UP, tempByte);

  // Set direction to output (MDIIC8876_REG_DIR)
  tempByte = readByte(MDIIC8876_REG_DIR);
  tempByte &= ~(1 << pin); // 0=output
  writeByte(MDIIC8876_REG_DIR, tempByte);

  // Enable oscillator (MDIIC8876_REG_CLOCK)
  tempByte = readByte(MDIIC8876_REG_CLOCK);
  tempByte |= (1 << 6);  // Internal 2MHz oscillator part 1 (set bit 6)
  tempByte &= ~(1 << 5); // Internal 2MHz oscillator part 2 (clear bit 5)
  writeByte(MDIIC8876_REG_CLOCK, tempByte);

  // Configure LED driver clock and mode (MDIIC8876_REG_MISC)
  tempByte = readByte(MDIIC8876_REG_MISC);
  if (log) {
    tempByte |= (1 << 7); // set logarithmic mode bank B
    tempByte |= (1 << 3); // set logarithmic mode bank A
  } else {
    tempByte &= ~(1 << 7); // set linear mode bank B
    tempByte &= ~(1 << 3); // set linear mode bank A
  }

  // Use configClock to setup the clock divder
  if (_clkX == 0) { // Make clckX non-zero
    _clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable

    uint8_t freq = (1 & 0x07) << 4; // freq should only be 3 bits from 6:4
    tempByte |= freq;
  }
  writeByte(MDIIC8876_REG_MISC, tempByte);

  // Enable LED driver operation (MDIIC8876_REG_LED_DRIVER_ENABLE)
  tempByte = readByte(MDIIC8876_REG_LED_DRIVER_ENABLE);
  tempByte |= (1 << pin);
  writeByte(MDIIC8876_REG_LED_DRIVER_ENABLE, tempByte);

  // Set MDIIC8876_REG_DATA bit low ~ LED driver started
  tempByte = readByte(MDIIC8876_REG_DATA);
  tempByte &= ~(1 << pin);
  writeByte(MDIIC8876_REG_DATA, tempByte);
}

void MDIIC8876::pwm(uint8_t pin, uint8_t iOn)
{
  // Write the on intensity of pin
  // Linear mode: Ion = iOn
  // Log mode: Ion = f(iOn)
  writeByte(MDIIC8876_REG_I_ON[pin], iOn);
}

void MDIIC8876::analogWrite(uint8_t pin, uint8_t iOn)
{
  pwm(pin, iOn);
}

void MDIIC8876::blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
  uint8_t onReg = calculateLEDTRegister(tOn);
  uint8_t offReg = calculateLEDTRegister(tOff);

  setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

void MDIIC8876::breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log)
{
  offInt = constrain(offInt, 0, 7);

  uint8_t onReg = calculateLEDTRegister(tOn);
  uint8_t offReg = calculateLEDTRegister(tOff);

  uint8_t riseTime = calculateSlopeRegister(rise, onInt, offInt);
  uint8_t fallTime = calculateSlopeRegister(fall, onInt, offInt);

  setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime, log);
}

void MDIIC8876::setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall, bool log)
{
  ledDriverInit(pin, log);

  // Keep parameters within their limits:
  tOn &= 0x1F;  // tOn should be a 5-bit value
  tOff &= 0x1F; // tOff should be a 5-bit value
  offIntensity &= 0x07;
  // Write the time on
  // 1-15:  TON = 64 * tOn * (255/ClkX)
  // 16-31: TON = 512 * tOn * (255/ClkX)
  writeByte(MDIIC8876_REG_T_ON[pin], tOn);

  // Write the time/intensity off register
  // 1-15:  TOFF = 64 * tOff * (255/ClkX)
  // 16-31: TOFF = 512 * tOff * (255/ClkX)
  // linear Mode - IOff = 4 * offIntensity
  // log mode - Ioff = f(4 * offIntensity)
  writeByte(MDIIC8876_REG_OFF[pin], (tOff << 3) | offIntensity);

  // Write the on intensity:
  writeByte(MDIIC8876_REG_I_ON[pin], onIntensity);

  // Prepare tRise and tFall
  tRise &= 0x1F; // tRise is a 5-bit value
  tFall &= 0x1F; // tFall is a 5-bit value

  // Write regTRise
  // 0: Off
  // 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
  // 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
  if (MDIIC8876_REG_T_RISE[pin] != 0xFF)
    writeByte(MDIIC8876_REG_T_RISE[pin], tRise);
  // Write regTFall
  // 0: off
  // 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
  // 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
  if (MDIIC8876_REG_T_FALL[pin] != 0xFF)
    writeByte(MDIIC8876_REG_T_FALL[pin], tFall);
}

void MDIIC8876::sync(void)
{
  // First check if nReset functionality is set
  uint8_t regMisc = readByte(MDIIC8876_REG_MISC);
  if (!(regMisc & 0x04))
  {
    regMisc |= (1 << 2);
    writeByte(MDIIC8876_REG_MISC, regMisc);
  }

  // Toggle nReset pin to sync LED timers
  pinMode(pinReset, OUTPUT);	  // set reset pin as output
  digitalWrite(pinReset, LOW);  // pull reset pin low
  delay(1);					  // Wait for the pin to settle
  digitalWrite(pinReset, HIGH); // pull reset pin back high

  // Return nReset to POR functionality
  writeByte(MDIIC8876_REG_MISC, (regMisc & ~(1 << 2)));
}

void MDIIC8876::debounceConfig(uint8_t configValue)
{
  // First make sure clock is configured
  uint8_t tempByte = readByte(MDIIC8876_REG_MISC);
  if ((tempByte & 0x70) == 0)
  {
    tempByte |= (1 << 4); // Just default to no divider if not set
    writeByte(MDIIC8876_REG_MISC, tempByte);
  }
  tempByte = readByte(MDIIC8876_REG_CLOCK);
  if ((tempByte & 0x60) == 0)
  {
    tempByte |= (1 << 6); // default to internal osc.
    writeByte(MDIIC8876_REG_CLOCK, tempByte);
  }

  configValue &= 0b111; // 3-bit value
  writeByte(MDIIC8876_REG_DEBOUNCE_CONFIG, configValue);
}

void MDIIC8876::debounceTime(uint8_t time)
{
  if (_clkX == 0)					   // If clock hasn't been set up.
    clock(MDIIC8876_INTERNAL_CLOCK_2MHZ, 1); // Set clock to 2MHz.

  // Debounce time-to-byte map: (assuming fOsc = 2MHz)
  // 0: 0.5ms		1: 1ms
  // 2: 2ms		3: 4ms
  // 4: 8ms		5: 16ms
  // 6: 32ms		7: 64ms
  // 2^(n-1)
  uint8_t configValue = 0;
  // We'll check for the highest set bit position,
  // and use that for debounceConfig
  for (int8_t i = 7; i >= 0; i--)
  {
    if (time & (1 << i))
    {
      configValue = i + 1;
      break;
    }
  }
  configValue = constrain(configValue, 0, 7);

  debounceConfig(configValue);
}

void MDIIC8876::debounceEnable(uint8_t pin)
{
  uint8_t debounceEnable = readByte(MDIIC8876_REG_DEBOUNCE_ENABLE);
  debounceEnable |= (1 << pin);
  writeByte(MDIIC8876_REG_DEBOUNCE_ENABLE, debounceEnable);
}

void MDIIC8876::debouncePin(uint8_t pin)
{
  debounceEnable(pin);
}

void MDIIC8876::debounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols)
{
  // Set up debounce time:
  debounceTime(time);

  // Set up debounce pins:
  for (uint8_t i = 0; i < numRows; i++)
    debouncePin(i);
  for (uint8_t i = 0; i < (8 + numCols); i++)
    debouncePin(i);
}

void MDIIC8876::enableInterrupt(uint8_t pin, uint8_t riseFall)
{
  // Set MDIIC8876_REG_INTERRUPT_MASK
  uint8_t tempByte = readByte(MDIIC8876_REG_INTERRUPT_MASK);
  tempByte &= ~(1 << pin); // 0 = event on IO will trigger interrupt
  writeByte(MDIIC8876_REG_INTERRUPT_MASK, tempByte);

  uint8_t sensitivity = 0;
  switch (riseFall)
  {
    case CHANGE:
      sensitivity = 0b11;
      break;
    case FALLING:
      sensitivity = 0b10;
      break;
    case RISING:
      sensitivity = 0b01;
      break;
  }

  // Set MDIIC8876_REG_SENSE_XXX
  // Sensitivity is set as follows:
  // 00: None
  // 01: Rising
  // 10: Falling
  // 11: Both
  uint8_t pinMask = (pin & 0x07) * 2;
  uint8_t senseRegister;

  // Need to select between two words. One for bank A, one for B.
  if (pin >= 8)
    senseRegister = MDIIC8876_REG_SENSE_HIGH;
  else
    senseRegister = MDIIC8876_REG_SENSE_HIGH;

  tempByte = readByte(senseRegister);
  tempByte &= ~(0b11 << pinMask);		  // Mask out the bits we want to write
  tempByte |= (sensitivity << pinMask); // Add our new bits
  writeByte(senseRegister, tempByte);
}

uint8_t MDIIC8876::interruptSource(bool clear /* =true*/)
{
  uint8_t intSource = readByte(MDIIC8876_REG_INTERRUPT_SOURCE);
  if (clear)
    writeByte(MDIIC8876_REG_INTERRUPT_SOURCE, 0xFFFF); // Clear interrupts
  return intSource;
}

bool MDIIC8876::checkInterrupt(uint8_t pin)
{
  if (interruptSource(false) & (1 << pin))
    return true;

  return false;
}

void MDIIC8876::clock(uint8_t oscSource, uint8_t oscDivider, uint8_t oscPinFunction, uint8_t oscFreqOut)
{
  configClock(oscSource, oscPinFunction, oscFreqOut, oscDivider);
}

void MDIIC8876::configClock(uint8_t oscSource /*= 2*/, uint8_t oscPinFunction /*= 0*/, uint8_t oscFreqOut /*= 0*/, uint8_t oscDivider /*= 1*/)
{
  // RegClock constructed as follows:
  //	6:5 - Oscillator frequency souce
  //		00: off, 01: external input, 10: internal 2MHz, 1: reserved
  //	4 - OSCIO pin function
  //		0: input, 1 ouptut
  //	3:0 - Frequency of oscout pin
  //		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
  oscSource = (oscSource & 0b11) << 5;		// 2-bit value, bits 6:5
  oscPinFunction = (oscPinFunction & 1) << 4; // 1-bit value bit 4
  oscFreqOut = (oscFreqOut & 0b1111);			// 4-bit value, bits 3:0
  uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
  writeByte(MDIIC8876_REG_CLOCK, regClock);

  // Config RegMisc[6:4] with oscDivider
  // 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
  oscDivider = constrain(oscDivider, 1, 7);
  _clkX = 2000000.0 / (1 << (oscDivider - 1)); // Update private clock variable
  oscDivider = (oscDivider & 0b111) << 4;		 // 3-bit value, bits 6:4

  uint8_t regMisc = readByte(MDIIC8876_REG_MISC);
  regMisc &= ~(0b111 << 4);
  regMisc |= oscDivider;
  writeByte(MDIIC8876_REG_MISC, regMisc);
}

uint8_t MDIIC8876::calculateLEDTRegister(uint8_t ms)
{
  uint8_t regOn1, regOn2;
  float timeOn1, timeOn2;

  if (_clkX == 0)
    return 0;

  regOn1 = (float)(ms / 1000.0) / (64.0 * 255.0 / (float)_clkX);
  regOn2 = regOn1 / 8;
  regOn1 = constrain(regOn1, 1, 15);
  regOn2 = constrain(regOn2, 16, 31);

  timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
  timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

  if (abs(timeOn1 - ms) < abs(timeOn2 - ms))
    return regOn1;
  else
    return regOn2;
}

uint8_t MDIIC8876::calculateSlopeRegister(uint8_t ms, uint8_t onIntensity, uint8_t offIntensity)
{
  uint16_t regSlope1, regSlope2;
  float regTime1, regTime2;

  if (_clkX == 0)
    return 0;

  float tFactor = ((float)onIntensity - (4.0 * (float)offIntensity)) * 255.0 / (float)_clkX;
  float timeS = float(ms) / 1000.0;

  regSlope1 = timeS / tFactor;
  regSlope2 = regSlope1 / 16;

  regSlope1 = constrain(regSlope1, 1, 15);
  regSlope2 = constrain(regSlope2, 16, 31);

  regTime1 = regSlope1 * tFactor * 1000.0;
  regTime2 = 16 * regTime1;

  if (abs(regTime1 - ms) < abs(regTime2 - ms))
    return regSlope1;
  else
    return regSlope2;
}

// readByte(uint8_t registerAddress)
//	This function reads a single byte located at the registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is the byte read from registerAddress
//		- Currently returns 0 if communication has timed out
uint8_t MDIIC8876::readByte(uint8_t registerAddress)
{
  uint8_t readValue;
  // Commented the line as variable seems unused;
  //uint16_t timeout = RECEIVE_TIMEOUT_VALUE;

  _i2cPort->beginTransmission(deviceAddress);
  _i2cPort->write(registerAddress);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(deviceAddress, (uint8_t)1);

  readValue = _i2cPort->read();

  return readValue;
}

bool MDIIC8876::readByte(uint8_t registerAddress, uint8_t *value)
{
  return readBytes(registerAddress, value, 1);
}

// readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
//	This function reads a series of bytes incrementing from a given address
//	- firstRegisterAddress is the first address to be read
//	- destination is an array of bytes where the read values will be stored into
//	- length is the number of bytes to be read
//	- Return boolean true if succesfull
bool MDIIC8876::readBytes(uint8_t firstRegisterAddress, uint8_t *destination, uint8_t length)
{
  _i2cPort->beginTransmission(deviceAddress);
  _i2cPort->write(firstRegisterAddress);
  uint8_t endResult = _i2cPort->endTransmission();
  bool result = (endResult == MDIIC8876_I2C_ERROR_OK) && (_i2cPort->requestFrom(deviceAddress, length) == length);

  if (result)
  {
    for (uint8_t i = 0; i < length; i++)
    {
      destination[i] = _i2cPort->read();
    }
  }
  return result;
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//	This function writes a single byte to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddres should already be set from the constructor
//	- Return value: true if succeeded, false if failed
bool MDIIC8876::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
  _i2cPort->beginTransmission(deviceAddress);
  bool result = _i2cPort->write(registerAddress) && _i2cPort->write(writeValue);
  uint8_t endResult = _i2cPort->endTransmission();
  return result && (endResult == MDIIC8876_I2C_ERROR_OK);
}

// writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
//	This function writes an array of bytes, beggining at a specific adddress
//	- firstRegisterAddress is the initial register to be written.
//		- All writes following will be at incremental register addresses.
//	- writeArray should be an array of byte values to be written.
//	- length should be the number of bytes to be written.
//	- Return value: true if succeeded, false if failed
bool MDIIC8876::writeBytes(uint8_t firstRegisterAddress, uint8_t *writeArray, uint8_t length)
{
  _i2cPort->beginTransmission(deviceAddress);
  bool result = _i2cPort->write(firstRegisterAddress);
  result = _i2cPort->write(writeArray, length);
  uint8_t endResult = _i2cPort->endTransmission();
  return result && (endResult == MDIIC8876_I2C_ERROR_OK);
}


/**
 * IIC8876电机驱动 SX1508-TMI8876：设置电机速度
 * 
 * 本函数用于通过模拟输出控制电机的速度。根据传入的速度值，决定电机的旋转方向和速度。
 * 通过调整两个引脚的PWM值，实现电机的正转、反转和停止。
 * 
 * @param pin_en 电机控制的第一个引脚，用于输出PWM信号，控制电机速度。
 * @param pin_ph 电机控制的第二个引脚，用于输出高低电平，控制电机方向。
 * @param speed 电机的速度设定值，可以为正、负或零。
 *               正值表示电机正转，值越大速度越快。
 *               负值表示电机反转，值越小速度越快。
 *               零表示电机停止。
 */
void MDIIC8876::setMotorSpeed(int pin_en, int pin_ph, int speed) {
    if (speed > 0) {    // 当速度大于0时，电机正转，pin_ph设为1，将pin_en设为速度值。
        digitalWrite(pin_ph, 1); // 正转
        analogWrite(pin_en, speed);
    } else if (speed < 0) { // 当速度小于0时，电机反转，pin_ph设为0，将pin_en设为速度的绝对值。
        digitalWrite(pin_ph, 0); // 反转
        analogWrite(pin_en, abs(speed));
    } else {    // 当速度为0时，电机停止，EN引脚设为0，刹车。
        analogWrite(pin_en, 0);
    }
}

/**
 * IIC8876电机驱动 SX1508-TMI8876：设置四个电机的速度
 * 
 * 该函数用于分别设置四个电机的速度，确保速度值在-255到255的范围内。
 * 超出范围的速度值会被限制在范围内，以保护电机和控制系统。
 * 
 * @param m1Speed 电机1的速度，范围为-255到255。
 * @param m2Speed 电机2的速度，范围为-255到255。
 * @param m3Speed 电机3的速度，范围为-255到255。
 * @param m4Speed 电机4的速度，范围为-255到255。
 */
void MDIIC8876::setMotor(int m1Speed, int m2Speed, int m3Speed, int m4Speed) {
    // 限制速度值在-255到255之间
    // 参数范围检查并修正，确保速度在 -255 到 255 之间
    m1Speed = constrain(m1Speed, -255, 255);
    m2Speed = constrain(m2Speed, -255, 255);
    m3Speed = constrain(m3Speed, -255, 255);
    m4Speed = constrain(m4Speed, -255, 255);

    // 调用函数设置每个电机的速度
    setMotorSpeed(M1EN, M1PH, m1Speed);
    setMotorSpeed(M2EN, M2PH, m2Speed);
    setMotorSpeed(M3EN, M3PH, m3Speed);
    setMotorSpeed(M4EN, M4PH, m4Speed);
}

/**
 * IIC8876电机驱动 SX1508-TMI8876：设置电机速度
 * 
 * @param motorId 电机编号，范围为1到4，超出范围将默认为5，即所有电机。
 * @param speed 电机速度，取值范围为-255到255，速度的正负代表电机的旋转方向。
 */
void MDIIC8876::setMotor(int motorId, int speed) {
    // 限制速度值在-255到255之间
    // 参数范围检查并修正，确保速度在 -255 到 255 之间
    speed = constrain(speed, -255, 255);

    // 确保电机ID在有效范围内，否则默认控制所有电机
    // 确保电机ID在有效范围内
    if (motorId < 1 || motorId > 4) {
        motorId = 5;
    }

    // 根据电机编号设置对应电机的速度
    if(motorId == 1){
        setMotorSpeed(M1EN, M1PH, speed);
    } else if(motorId == 2){
        setMotorSpeed(M2EN, M2PH, speed);
    } else if(motorId == 3){
        setMotorSpeed(M3EN, M3PH, speed);
    } else if(motorId == 4){
        setMotorSpeed(M4EN, M4PH, speed);
    } else {
        // 如果电机ID无效，则同时设置所有电机的速度
        setMotorSpeed(M1EN, M1PH, speed);
        setMotorSpeed(M2EN, M2PH, speed);
        setMotorSpeed(M3EN, M3PH, speed);
        setMotorSpeed(M4EN, M4PH, speed);
    }
}

/**
 * @brief IIC8876电机驱动 SX1508-TMI8876：停止指定电机的运行
 * 
 * @param motorId 电机标识符，用于指定要停止的电机(1, 2, 3, 4其他值则表示所有电机) 
 */
void MDIIC8876::stopMotor(int motorId) {
    // 设置电机控制参数为0，以停止电机运行
    setMotor(motorId, 0);
}
