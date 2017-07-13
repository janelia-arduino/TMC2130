// ----------------------------------------------------------------------------
// TMC2130.cpp
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#include "TMC2130.h"


void TMC2130::setup(const size_t cs_pin)
{
  cs_pin_ = cs_pin;
  enable_pin_ = -1;

  global_config_.uint32 = 0;

  driver_current_.uint32 = 0;

  chopper_config_.uint32 = 0;
  chopper_config_.fields.tbl = TBL_DEFAULT;
  chopper_config_.fields.toff = TOFF_DEFAULT;
  microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;

  pinMode(cs_pin_,OUTPUT);
  digitalWrite(cs_pin_,HIGH);

  SPI.begin();
}

void TMC2130::setup(const size_t cs_pin,
                    const size_t enable_pin)
{
  setup(cs_pin);
  setEnablePin(enable_pin);
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

// void TMC2130::setStepDirInput()
// {
// }

// void TMC2130::setSpiInput()
// {
// }

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

void TMC2130::setMicrostepsPerStepPowerOfTwo(const uint8_t exponent)
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

size_t TMC2130::getMicrostepsPerStep()
{
  return 1 << microsteps_per_step_exponent_;
}

void TMC2130::setRunCurrent(const uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);
  Serial << "run_current_percent = " << percent << " run_current_setting = " << run_current << endl;

  driver_current_.fields.irun = run_current;
  setDriverCurrent();
}

void TMC2130::setHoldCurrent(const uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);
  Serial << "hold_current_percent = " << percent << " hold_current_setting = " << hold_current << endl;

  driver_current_.fields.ihold = hold_current;
  setDriverCurrent();
}

void TMC2130::setHoldDelay(const uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);
  Serial << "hold_delay_percent = " << percent << " hold_delay = " << hold_delay << endl;

  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}

void TMC2130::setAllCurrentValues(const uint8_t run_current_percent,
                                  const uint8_t hold_current_percent,
                                  const uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  driver_current_.fields.irun = run_current;
  driver_current_.fields.ihold = hold_current;
  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}


// TMC2130::Status TMC2130::getStatus()
// {
//   return status_;
// }

// private
void TMC2130::setEnablePin(const size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_,OUTPUT);
  disable();
}

TMC2130::MisoDatagram TMC2130::sendReceivePrevious(TMC2130::MosiDatagram & mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.uint64 = 0;
  // SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  // digitalWrite(cs_pin_,LOW);
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.uint64 >> (8*i)) & 0xff;
    Serial << "byte_write = "<< _HEX(byte_write) << endl;
    // uint8_t byte_read = SPI.transfer(byte_write);
    // miso_datagram.uint64 |= byte_read << (8*i);
  }
  // digitalWrite(cs_pin_,HIGH);
  // SPI.endTransaction();
  // noInterrupts();
  // status_ = miso_datagram.fields.status;
  // interrupts();
  return miso_datagram;
}

TMC2130::MisoDatagram TMC2130::write(const uint8_t address,
                                     const uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.address = address;

  mosi_datagram.fields.data = data;
  sendReceivePrevious(mosi_datagram);
}

uint8_t TMC2130::percentToCurrentSetting(uint8_t percent)
{
  uint8_t current_percent = constrain(percent,PERCENT_MIN,PERCENT_MAX);
  uint8_t current_setting = map(current_percent,PERCENT_MIN,PERCENT_MAX,CURRENT_SETTING_MIN,CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t TMC2130::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t hold_delay_percent = constrain(percent,PERCENT_MIN,PERCENT_MAX);
  uint8_t hold_delay = map(hold_delay_percent,PERCENT_MIN,PERCENT_MAX,HOLD_DELAY_MIN,HOLD_DELAY_MAX);
  return hold_delay;
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
