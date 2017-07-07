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

  pinMode(cs_pin_,OUTPUT);
  digitalWrite(cs_pin_,HIGH);

  SPI.begin();
}

void TMC2130::setStepDirInput()
{
  configDriver(RDSEL_MICROSTEP,
               VSENSE_LOW_SENSE_POWER,
               SDOFF_STEP_DIR,
               TS2G_3200NS,
               DISS2G_ENABLE_SHORT_PROTECTION,
               SLP_MAXIMUM,
               SLP_MAXIMUM);
}

// void TMC2130::setSpiInput()
// {
//   configDriver(RDSEL_MICROSTEP,
//                VSENSE_LOW_SENSE_POWER,
//                SDOFF_STEP_DIR,
//                TS2G_3200NS,
//                DISS2G_ENABLE_SHORT_PROTECTION,
//                SLP_MAXIMUM,
//                SLP_MAXIMUM);
// }

void TMC2130::setMicrostepsPerStepTo256()
{
  setDriverControlStepDir(MRES_256,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo128()
{
  setDriverControlStepDir(MRES_128,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo64()
{
  setDriverControlStepDir(MRES_064,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo32()
{
  setDriverControlStepDir(MRES_032,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo16()
{
  setDriverControlStepDir(MRES_016,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo8()
{
  setDriverControlStepDir(MRES_008,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo4()
{
  setDriverControlStepDir(MRES_004,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo2()
{
  setDriverControlStepDir(MRES_002,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setMicrostepsPerStepTo1()
{
  setDriverControlStepDir(MRES_001,
                          DEDGE_RISING,
                          INTPOL_DISABLE_INTERPOLATION);
}

void TMC2130::setDefaultChopperConfig()
{
  configChopper(1,
                0b011,
                0b0010,
                0b00,
                0,
                0,
                0b10);
}

void TMC2130::disableCoolStep()
{
  setCoolStepRegister(SEMIN_DISABLED,
                      SEUP_1,
                      0b00,
                      SEDN_32,
                      SEIMIN_HALF);
}

// void TMC2130::enableCoolStep()
// {
//   setCoolStepRegister(SEMIN_DISABLED,
//                              SEUP_1,
//                              0b00,
//                              SEDN_32,
//                              SEIMIN_HALF);
// }

double TMC2130::setCurrentScalePercent(uint8_t cs)
{
  uint8_t cs_thresholded = cs;
  if (cs_thresholded > CURRENT_SCALE_PERCENT_MAX)
  {
    cs_thresholded = CURRENT_SCALE_PERCENT_MAX;
  }
  if (cs_thresholded < CURRENT_SCALE_PERCENT_MIN)
  {
    cs_thresholded = CURRENT_SCALE_PERCENT_MIN;
  }
  uint8_t cs_mapped = betterMap(cs_thresholded,
                                CURRENT_SCALE_PERCENT_MIN,
                                CURRENT_SCALE_PERCENT_MAX,
                                CS_REGISTER_MIN,
                                CS_REGISTER_MAX);
  setStallGuardRegister(cs_mapped,
                        SGT_DEFAULT,
                        SFILT_FILTERED_MODE);
  return (cs_mapped + 1)*100.0/32;
}

// TMC2130::Status TMC2130::getStatus()
// {
//   return status_;
// }

// private
TMC2130::MisoDatagram TMC2130::writeRead(const uint32_t data)
{
  MosiDatagram datagram_write;
  datagram_write.uint32 = 0;
  datagram_write.fields.data = data;
  MisoDatagram datagram_read;
  datagram_read.uint32 = 0;
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  digitalWrite(cs_pin_,LOW);
  for (int i=(MOSI_DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (datagram_write.uint64 >> (8*i)) & 0xff;
    uint8_t byte_read = SPI.transfer(byte_write);
    datagram_read.uint64 |= byte_read << (8*i);
  }
  digitalWrite(cs_pin_,HIGH);
  SPI.endTransaction();
  noInterrupts();
  status_ = datagram_read.fields.status;
  interrupts();
  return datagram_read;
}

void TMC2130::configDriver(const uint8_t rdsel,
                          const uint8_t vsense,
                          const uint8_t sdoff,
                          const uint8_t ts2g,
                          const uint8_t diss2g,
                          const uint8_t slpl,
                          const uint8_t slph)
{
  DrvConf drv_conf;
  drv_conf.uint32 = 0;
  drv_conf.fields.rdsel = rdsel;
  drv_conf.fields.vsense = vsense;
  drv_conf.fields.sdoff = sdoff;
  drv_conf.fields.ts2g = ts2g;
  drv_conf.fields.diss2g = diss2g;
  drv_conf.fields.slpl = slpl;
  drv_conf.fields.slph = slph;
  drv_conf.fields.test = 0;
  drv_conf.fields.address = ADDRESS_DRVCONF;
  writeRead(drv_conf.uint32);
}

void TMC2130::setDriverControlStepDir(const uint8_t mres,
                                     const uint8_t dedge,
                                     const uint8_t intpol)
{
  DrvContStepDir drv_cont;
  drv_cont.uint32 = 0;
  drv_cont.fields.mres = mres;
  drv_cont.fields.dedge = dedge;
  drv_cont.fields.intpol = intpol;
  drv_cont.fields.address = ADDRESS_DRVCTRL;
  writeRead(drv_cont.uint32);
}

void TMC2130::configChopper(const uint8_t toff,
                           const uint8_t hstrt,
                           const uint8_t hend,
                           const uint8_t hdec,
                           const uint8_t rndtf,
                           const uint8_t chm,
                           const uint8_t tbl)
{
  ChopConf chop_conf;
  chop_conf.uint32 = 0;
  chop_conf.fields.toff = toff;
  chop_conf.fields.hstrt = hstrt;
  chop_conf.fields.hend = hend;
  chop_conf.fields.hdec = hdec;
  chop_conf.fields.rndtf = rndtf;
  chop_conf.fields.chm = chm;
  chop_conf.fields.tbl = tbl;
  chop_conf.fields.address = ADDRESS_CHOPCONF;
  writeRead(chop_conf.uint32);
}

void TMC2130::setCoolStepRegister(const uint8_t semin,
                                 const uint8_t seup,
                                 const uint8_t semax,
                                 const uint8_t sedn,
                                 const uint8_t seimin)
{
  SmartEn smart_en;
  smart_en.uint32 = 0;
  smart_en.fields.semin = semin;
  smart_en.fields.seup = seup;
  smart_en.fields.semax = semax;
  smart_en.fields.sedn = sedn;
  smart_en.fields.seimin = seimin;
  smart_en.fields.address = ADDRESS_SMARTEN;
  writeRead(smart_en.uint32);
}

void TMC2130::setStallGuardRegister(const uint8_t cs,
                                   const int8_t sgt,
                                   const uint8_t sfilt)
{
  SgcsConf sgcs_conf;
  sgcs_conf.uint32 = 0;
  sgcs_conf.fields.cs = cs;
  int8_t sgt_thresholded = sgt;
  if (sgt_thresholded > SGT_REGISTER_MAX)
  {
    sgt_thresholded = SGT_REGISTER_MAX;
  }
  if (sgt_thresholded < SGT_REGISTER_MIN)
  {
    sgt_thresholded = SGT_REGISTER_MIN;
  }
  sgcs_conf.fields.sgt = sgt_thresholded;
  sgcs_conf.fields.sfilt = sfilt;
  sgcs_conf.fields.address = ADDRESS_SGCSCONF;
  writeRead(sgcs_conf.uint32);
}

long TMC2130::betterMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  long mapped_x = map(x,in_min,(in_max+1),out_min,(out_max+1));
  return constrain(mapped_x,out_min,out_max);
}
