#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC2130.h"

const long BAUDRATE = 115200;
const int LOOP_DELAY = 2000;
const int CS_PIN = 14;

// Instantiate TMC2130
TMC2130 stepper_driver;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  stepper_driver.setup(CS_PIN);

}

void loop()
{
  // stepper_driver.enableAnalogInputCurrentScaling();
  // stepper_driver.disableAnalogInputCurrentScaling();

  // for (size_t i=0; i<=8; ++i)
  // {
  //   stepper_driver.setMicrostepsPerStepPowerOfTwo(i);
  //   Serial << "microsteps_per_step = " << stepper_driver.getMicrostepsPerStep() << endl;
  // }

  stepper_driver.setRunCurrent(100);
  Serial << "\n";
  stepper_driver.setHoldCurrent(50);
  Serial << "\n";
  stepper_driver.setHoldDelay(50);
  Serial << "\n";

  stepper_driver.setAllCurrentValues(100,50,50);
  Serial << "\n";

  Serial << "\n";
  delay(LOOP_DELAY);
}
