#include <Arduino.h>
#include <SPI.h>
#include <TMC2130.h>

const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 14;

// Instantiate TMC2130
TMC2130 stepper_driver;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  stepper_driver.setup(CHIP_SELECT_PIN);

}

void loop()
{
  if (stepper_driver.communicating())
  {
    Serial.println("SPI communicating with stepper driver!");
  }
  else
  {
    Serial.println("SPI not communicating with stepper driver!");
  }

  stepper_driver.initialize();

  TMC2130::Status status = stepper_driver.getStatus();
  Serial.print("status.load = ");
  Serial.println(status.load);
  Serial.print("status.full_step_active = ");
  Serial.println(status.full_step_active);
  Serial.print("status.current_scaling = ");
  Serial.println(status.current_scaling);
  Serial.print("status.stall = ");
  Serial.println(status.stall);
  Serial.print("status.over_temperature_shutdown = ");
  Serial.println(status.over_temperature_shutdown);
  Serial.print("status.over_temperature_warning = ");
  Serial.println(status.over_temperature_warning);
  Serial.print("status.short_to_ground_a = ");
  Serial.println(status.short_to_ground_a);
  Serial.print("status.short_to_ground_b = ");
  Serial.println(status.short_to_ground_b);
  Serial.print("status.open_load_a = ");
  Serial.println(status.open_load_a);
  Serial.print("status.open_load_b = ");
  Serial.println(status.open_load_b);
  Serial.print("status.standstill = ");
  Serial.println(status.standstill);

  // stepper_driver.setRunCurrent(100);
  // Serial.println();
  // stepper_driver.setHoldCurrent(50);
  // Serial.println();
  // stepper_driver.setHoldDelay(50);
  // Serial.println();

  // stepper_driver.setAllCurrentValues(100,50,50);
  // Serial.println();

  Serial.println();
  delay(LOOP_DELAY);
}
