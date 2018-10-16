// ----------------------------------------------------------------------------
// TMC2130.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "TMC2130.h"


void TMC2130::setup(size_t chip_select_pin)
{
  chip_select_pin_ = chip_select_pin;
  enable_pin_ = -1;

  pinMode(chip_select_pin_,OUTPUT);
  digitalWrite(chip_select_pin_,HIGH);

  SPI.begin();

  global_config_.uint32 = 0;

  driver_current_.uint32 = 0;

  chopper_config_.uint32 = 0;
  chopper_config_.fields.toff = TOFF_DEFAULT;
  chopper_config_.fields.hstrt = HSTRT_DEFAULT;
  chopper_config_.fields.hend = HEND_DEFAULT;
  chopper_config_.fields.chm = CHM_DEFAULT;
  chopper_config_.fields.tbl = TBL_DEFAULT;
  microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;

  pwm_config_.uint32 = 0;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.fields.pwm_grad = PWM_GRAD_DEFAULT;
  pwm_config_.fields.pwm_freq = PWM_FREQ_DEFAULT;
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_DEFAULT;
}

void TMC2130::setup(size_t chip_select_pin,
  size_t enable_pin)
{
  setup(chip_select_pin);
  setEnablePin(enable_pin);
}

bool TMC2130::communicating()
{
  return (getVersion() == VERSION);
}

uint8_t TMC2130::getVersion()
{
  uint32_t data = read(ADDRESS_IOIN);

  InputPinStatus input_pin_status;
  input_pin_status.uint32 = data;

  return input_pin_status.fields.version;
}

void TMC2130::initialize()
{
  setMicrostepsPerStep(256);
  enableStealthChop();
  setPwmThreshold(TPWMTHRS_DEFAULT);
  setPwmConfig();
}

void TMC2130::enable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_,LOW);
  }
}

void TMC2130::disable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_,HIGH);
  }
}

void TMC2130::setMicrostepsPerStep(size_t microsteps_per_step)
{
  size_t microsteps_per_step_shifted = constrain(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  size_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  setMicrostepsPerStepPowerOfTwo(exponent);
}

size_t TMC2130::getMicrostepsPerStep()
{
  return 1 << microsteps_per_step_exponent_;
}

void TMC2130::setRunCurrent(uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);

  driver_current_.fields.irun = run_current;
  setDriverCurrent();
}

void TMC2130::setHoldCurrent(uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);

  driver_current_.fields.ihold = hold_current;
  setDriverCurrent();
}

void TMC2130::setHoldDelay(uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);

  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}

void TMC2130::setAllCurrentValues(uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  driver_current_.fields.irun = run_current;
  driver_current_.fields.ihold = hold_current;
  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}

TMC2130::Status TMC2130::getStatus()
{
  DriveStatus drive_status;
  drive_status.uint32 = read(ADDRESS_DRV_STATUS);
  return drive_status.fields.status;
}

void TMC2130::enableAnalogInputCurrentScaling()
{
  global_config_.fields.i_scale_analog = 1;
  setGlobalConfig();
}

void TMC2130::disableAnalogInputCurrentScaling()
{
  global_config_.fields.i_scale_analog = 0;
  setGlobalConfig();
}

void TMC2130::enableInverseMotorDirection()
{
  global_config_.fields.shaft = 1;
  setGlobalConfig();
}

void TMC2130::disableInverseMotorDirection()
{
  global_config_.fields.shaft = 0;
  setGlobalConfig();
}

void TMC2130::enableStealthChop()
{
  global_config_.fields.en_pwm_mode = 1;
  setGlobalConfig();
}

void TMC2130::disableStealthChop()
{
  global_config_.fields.en_pwm_mode = 0;
  setGlobalConfig();
}

void TMC2130::enableAutomaticCurrentScaling()
{
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_ENABLED;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.fields.pwm_grad = PWM_GRAD_DEFAULT;
  setPwmConfig();
}

void TMC2130::disableAutomaticCurrentScaling()
{
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_DISABLED;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_MIN;
  pwm_config_.fields.pwm_grad = PWM_GRAD_MIN;
  setPwmConfig();
}

void TMC2130::setZeroHoldCurrentMode(TMC2130::ZeroHoldCurrentMode mode)
{
  pwm_config_.fields.freewheel = mode;
  setPwmConfig();
}

void TMC2130::setPwmOffset(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwmAmplitudeToPwmAmpl(pwm_amplitude);
  pwm_config_.fields.pwm_ampl = pwm_ampl;
  setPwmConfig();
}

void TMC2130::setPwmGradient(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwmAmplitudeToPwmGrad(pwm_amplitude);
  pwm_config_.fields.pwm_grad = pwm_grad;
  setPwmConfig();
}

uint8_t TMC2130::getPwmScale()
{
  return read(ADDRESS_PWM_SCALE);
}

TMC2130::Settings TMC2130::getSettings()
{
  Settings settings;
  settings.stealth_chop_enabled = global_config_.fields.en_pwm_mode;
  settings.automatic_current_scaling_enabled = pwm_config_.fields.pwm_autoscale;
  settings.zero_hold_current_mode = pwm_config_.fields.freewheel;
  settings.pwm_offset = pwm_config_.fields.pwm_ampl;
  settings.pwm_gradient = pwm_config_.fields.pwm_grad;
  settings.irun = driver_current_.fields.irun;
  settings.ihold = driver_current_.fields.ihold;
  settings.iholddelay = driver_current_.fields.iholddelay;

  return settings;
}

// private
void TMC2130::setEnablePin(size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_,OUTPUT);
  disable();
}

// void TMC2130::setStepDirInput()
// {
// }

// void TMC2130::setSpiInput()
// {
// }

void TMC2130::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  microsteps_per_step_exponent_ = exponent;

  switch (exponent)
  {
    case 0:
    {
      chopper_config_.fields.mres = MRES_001;
      break;
    }
    case 1:
    {
      chopper_config_.fields.mres = MRES_002;
      break;
    }
    case 2:
    {
      chopper_config_.fields.mres = MRES_004;
      break;
    }
    case 3:
    {
      chopper_config_.fields.mres = MRES_008;
      break;
    }
    case 4:
    {
      chopper_config_.fields.mres = MRES_016;
      break;
    }
    case 5:
    {
      chopper_config_.fields.mres = MRES_032;
      break;
    }
    case 6:
    {
      chopper_config_.fields.mres = MRES_064;
      break;
    }
    case 7:
    {
      chopper_config_.fields.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;
      chopper_config_.fields.mres = MRES_256;
      break;
    }
  }
  setChopperConfig();
}

uint32_t TMC2130::sendReceivePrevious(TMC2130::MosiDatagram & mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.uint64 = 0;

  spiBeginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.uint64 >> (8*i)) & 0xff;
    uint8_t byte_read = SPI.transfer(byte_write);
    miso_datagram.uint64 |= byte_read << (8*i);
  }
  spiEndTransaction();

  noInterrupts();
  spi_status_ = miso_datagram.fields.spi_status;
  interrupts();

  return miso_datagram.fields.data;
}

uint32_t TMC2130::write(uint8_t address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.address = address;
  mosi_datagram.fields.data = data;

  return sendReceivePrevious(mosi_datagram);
}

uint32_t TMC2130::read(uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.address = address;

  // must read twice to get value at address
  sendReceivePrevious(mosi_datagram);
  uint32_t data = sendReceivePrevious(mosi_datagram);
  return data;
}

uint8_t TMC2130::percentToCurrentSetting(uint8_t percent)
{
  uint8_t current_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(current_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t TMC2130::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t hold_delay_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay = map(hold_delay_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay;
}

uint8_t TMC2130::pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwm_amplitude;
  if (pwm_config_.fields.pwm_autoscale)
  {
    pwm_ampl = constrain(pwm_ampl,
      PWM_AMPL_AUTOSCALE_MIN,
      PWM_AMPL_AUTOSCALE_MAX);
  }
  return pwm_ampl;
}

uint8_t TMC2130::pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwm_amplitude;
  if (pwm_config_.fields.pwm_autoscale)
  {
    pwm_grad = constrain(pwm_grad,
      PWM_GRAD_AUTOSCALE_MIN,
      PWM_GRAD_AUTOSCALE_MAX);
  }
  return pwm_grad;
}

void TMC2130::setGlobalConfig()
{
  write(ADDRESS_GCONF,global_config_.uint32);
}

void TMC2130::setDriverCurrent()
{
  write(ADDRESS_IHOLD_IRUN,driver_current_.uint32);
}

void TMC2130::setChopperConfig()
{
  write(ADDRESS_CHOPCONF,chopper_config_.uint32);
}

void TMC2130::setPwmThreshold(uint32_t value)
{
  write(ADDRESS_TPWMTHRS,value);
}

void TMC2130::setPwmConfig()
{
  write(ADDRESS_PWMCONF,pwm_config_.uint32);
}

void TMC2130::enableClockSelect()
{
  digitalWrite(chip_select_pin_,LOW);
}

void TMC2130::disableClockSelect()
{
  digitalWrite(chip_select_pin_,HIGH);
}
void TMC2130::spiBeginTransaction()
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  enableClockSelect();
}

void TMC2130::spiEndTransaction()
{
  disableClockSelect();
  SPI.endTransaction();
}
