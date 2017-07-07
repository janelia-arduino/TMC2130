// ----------------------------------------------------------------------------
// TMC2130.h
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#ifndef TMC2130_H
#define TMC2130_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SPI.h"

#include "Streaming.h"


class TMC2130
{
public:
  void setup(const size_t cs_pin);

  void setStepDirInput();
  // void setSpiInput();

  void setMicrostepsPerStepTo256();
  void setMicrostepsPerStepTo128();
  void setMicrostepsPerStepTo64();
  void setMicrostepsPerStepTo32();
  void setMicrostepsPerStepTo16();
  void setMicrostepsPerStepTo8();
  void setMicrostepsPerStepTo4();
  void setMicrostepsPerStepTo2();
  void setMicrostepsPerStepTo1();

  void setDefaultChopperConfig();

  void disableCoolStep();
  // void enableCoolStep();

  double setCurrentScalePercent(uint8_t cs);
  const static uint8_t CURRENT_SCALE_PERCENT_MIN = 1;
  const static uint8_t CURRENT_SCALE_PERCENT_MAX = 100;

  struct Status
  {
    uint8_t stall : 1;
    uint8_t over_temperature_shutdown : 1;
    uint8_t over_temperature_warning : 1;
    uint8_t short_to_ground_a : 1;
    uint8_t short_to_ground_b : 1;
    uint8_t open_load_a : 1;
    uint8_t open_load_b : 1;
    uint8_t standstill : 1;
  };

  // Status getStatus();

private:
  // SPISettings
  const static uint32_t SPI_CLOCK = 1000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE3;

  // MOSI Datagrams
  union MosiDatagram
  {
    struct Fields
    {
      uint32_t data : 20;
      uint32_t space : 12;
    } fields;
    uint32_t uint32;
  };

  // Addresses
  const static uint8_t ADDRESS_DRVCTRL = 0b00;
  const static uint8_t ADDRESS_CHOPCONF = 0b100;
  const static uint8_t ADDRESS_SMARTEN = 0b101;
  const static uint8_t ADDRESS_SGCSCONF = 0b110;
  const static uint8_t ADDRESS_DRVCONF = 0b111;

  // MISO Datagrams
  union MisoDatagram
  {
    struct Fields
    {
      uint32_t space0 : 12;
      Status status;
      uint32_t space1 : 2;
      uint32_t data : 10;
    } fields;
    uint32_t uint32;
  };

  // Union Structs
  union DrvConf
  {
    struct Fields
    {
      uint32_t reserved0 : 4;
      uint32_t rdsel : 2;
      uint32_t vsense : 1;
      uint32_t sdoff : 1;
      uint32_t ts2g : 2;
      uint32_t diss2g : 1;
      uint32_t reserved1 : 1;
      uint32_t slpl : 2;
      uint32_t slph : 2;
      uint32_t test : 1;
      uint32_t address : 3;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  const static uint8_t RDSEL_MICROSTEP = 0b00;
  const static uint8_t RDSEL_SG = 0b01;
  const static uint8_t RDSEL_SGCS = 0b11;
  const static uint8_t VSENSE_BEST_CURRENT_REG = 0b0;
  const static uint8_t VSENSE_LOW_SENSE_POWER = 0b1;
  const static uint8_t SDOFF_STEP_DIR = 0b0;
  const static uint8_t SDOFF_SPI = 0b1;
  const static uint8_t TS2G_3200NS = 0b00;
  const static uint8_t TS2G_1600NS = 0b01;
  const static uint8_t TS2G_1200NS = 0b10;
  const static uint8_t TS2G_0800NS = 0b11;
  const static uint8_t DISS2G_ENABLE_SHORT_PROTECTION = 0b0;
  const static uint8_t DISS2G_DISABLE_SHORT_PROTECTION = 0b1;
  const static uint8_t SLP_MINIMUM = 0b00;
  const static uint8_t SLP_MINIMUM_TEMP = 0b01;
  const static uint8_t SLP_MEDIUM = 0b10;
  const static uint8_t SLP_MAXIMUM = 0b11;

  union DrvContStepDir
  {
    struct Fields
    {
      uint32_t mres : 4;
      uint32_t reserved0 : 4;
      uint32_t dedge : 1;
      uint32_t intpol : 1;
      uint32_t reserved1 : 8;
      uint32_t address : 2;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  const static uint8_t MRES_256 = 0b0000;
  const static uint8_t MRES_128 = 0b0001;
  const static uint8_t MRES_064 = 0b0010;
  const static uint8_t MRES_032 = 0b0011;
  const static uint8_t MRES_016 = 0b0100;
  const static uint8_t MRES_008 = 0b0101;
  const static uint8_t MRES_004 = 0b0110;
  const static uint8_t MRES_002 = 0b0111;
  const static uint8_t MRES_001 = 0b1000;
  const static uint8_t DEDGE_RISING = 0b0;
  const static uint8_t DEDGE_BOTH = 0b1;
  const static uint8_t INTPOL_DISABLE_INTERPOLATION = 0b0;
  const static uint8_t INTPOL_ENABLE_INTERPOLATION = 0b1;

  union DrvContSpi
  {
    struct Fields
    {
      uint32_t cb : 8;
      uint32_t phb : 1;
      uint32_t ca : 8;
      uint32_t pha : 1;
      uint32_t address : 2;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  union ChopConf
  {
    struct Fields
    {
      uint32_t toff : 4;
      uint32_t hstrt : 3;
      uint32_t hend : 4;
      uint32_t hdec : 2;
      uint32_t rndtf : 1;
      uint32_t chm : 1;
      uint32_t tbl : 2;
      uint32_t address : 3;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  union SmartEn
  {
    struct Fields
    {
      uint32_t semin : 4;
      uint32_t reserved0 : 1;
      uint32_t seup : 2;
      uint32_t reserved1 : 1;
      uint32_t semax : 4;
      uint32_t reserved2 : 1;
      uint32_t sedn : 2;
      uint32_t seimin : 1;
      uint32_t reserved3 : 1;
      uint32_t address : 3;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  const static uint8_t SEMIN_DISABLED = 0b0000;
  const static uint8_t SEUP_1 = 0b00;
  const static uint8_t SEUP_2 = 0b01;
  const static uint8_t SEUP_4 = 0b10;
  const static uint8_t SEUP_8 = 0b11;
  const static uint8_t SEDN_32 = 0b00;
  const static uint8_t SEDN_08 = 0b01;
  const static uint8_t SEDN_02 = 0b10;
  const static uint8_t SEDN_01 = 0b11;
  const static uint8_t SEIMIN_HALF = 0b0;
  const static uint8_t SEIMIN_QUARTER = 0b1;

  union SgcsConf
  {
    struct Fields
    {
      uint32_t cs : 5;
      uint32_t reserved0 : 3;
      uint32_t sgt : 7;
      uint32_t reserved1 : 1;
      uint32_t sfilt : 1;
      uint32_t address : 3;
      uint32_t space0 : 12;
    } fields;
    uint32_t uint32;
  };

  const static uint8_t CS_REGISTER_MIN = 0b00000;
  const static uint8_t CS_REGISTER_MAX = 0b11111;
  const static int8_t SGT_REGISTER_MIN = -10;
  const static int8_t SGT_REGISTER_MAX = 63;
  const static int8_t SGT_DEFAULT = 0b0000101;
  const static uint8_t SFILT_STANDARD_MODE = 0b0;
  const static uint8_t SFILT_FILTERED_MODE = 0b1;

  size_t cs_pin_;
  Status status_;

  MisoDatagram writeRead(const uint32_t data);

  void configDriver(const uint8_t rdsel,
                    const uint8_t vsense,
                    const uint8_t sdoff,
                    const uint8_t ts2g,
                    const uint8_t diss2g,
                    const uint8_t slpl,
                    const uint8_t slph);
  void setDriverControlStepDir(const uint8_t mres,
                               const uint8_t dedge,
                               const uint8_t intpol);
  void configChopper(const uint8_t toff,
                     const uint8_t hstrt,
                     const uint8_t hend,
                     const uint8_t hdec,
                     const uint8_t rndtf,
                     const uint8_t chm,
                     const uint8_t tbl);
  void setCoolStepRegister(const uint8_t semin,
                           const uint8_t seup,
                           const uint8_t semax,
                           const uint8_t sedn,
                           const uint8_t seimin);
  void setStallGuardRegister(const uint8_t cs,
                             const int8_t sgt,
                             const uint8_t sfilt);
  long betterMap(long x,
                 long in_min,
                 long in_max,
                 long out_min,
                 long out_max);

};

#endif
