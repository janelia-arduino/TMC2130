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

  microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;
  chopper_config_.uint32 = 0;
  chopper_config_.fields.tbl = TBL_DEFAULT;
  chopper_config_.fields.toff = TOFF_DEFAULT;

  pinMode(cs_pin_,OUTPUT);
  digitalWrite(cs_pin_,HIGH);

  SPI.begin();
}

void TMC2130::setup(const size_t cs_pin, const size_t enable_pin)
{
  setup(cs_pin);
  setEnablePin(enable_pin);
}

void TMC2130::setEnablePin(const size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_,OUTPUT);
  disable();
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

// void TMC2130::disableCoolStep()
// {
//   setCoolStepRegister(SEMIN_DISABLED,
//                       SEUP_1,
//                       0b00,
//                       SEDN_32,
//                       SEIMIN_HALF);
// }

// void TMC2130::enableCoolStep()
// {
//   setCoolStepRegister(SEMIN_DISABLED,
//                              SEUP_1,
//                              0b00,
//                              SEDN_32,
//                              SEIMIN_HALF);
// }

// double TMC2130::setCurrentScalePercent(uint8_t cs)
// {
//   uint8_t cs_thresholded = cs;
//   if (cs_thresholded > CURRENT_SCALE_PERCENT_MAX)
//   {
//     cs_thresholded = CURRENT_SCALE_PERCENT_MAX;
//   }
//   if (cs_thresholded < CURRENT_SCALE_PERCENT_MIN)
//   {
//     cs_thresholded = CURRENT_SCALE_PERCENT_MIN;
//   }
//   uint8_t cs_mapped = betterMap(cs_thresholded,
//                                 CURRENT_SCALE_PERCENT_MIN,
//                                 CURRENT_SCALE_PERCENT_MAX,
//                                 CS_REGISTER_MIN,
//                                 CS_REGISTER_MAX);
//   setStallGuardRegister(cs_mapped,
//                         SGT_DEFAULT,
//                         SFILT_FILTERED_MODE);
//   return (cs_mapped + 1)*100.0/32;
// }

// TMC2130::Status TMC2130::getStatus()
// {
//   return status_;
// }

// private
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

void TMC2130::setChopperConfig()
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.address = ADDRESS_CHOPCONF;

  mosi_datagram.fields.data = chopper_config_.uint32;
  sendReceivePrevious(mosi_datagram);
}
