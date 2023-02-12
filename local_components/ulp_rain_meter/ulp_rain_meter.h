#pragma once

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"

namespace esphome {
namespace ulp_rain_meter {

class UlpRainMeter : public sensor::Sensor, public PollingComponent {
public:
  UlpRainMeter() : PollingComponent(15000) {}

  void setup();
  void update();

private:
  void ulpBlink(uint32_t halfPeriodUs);
};

} // namespace ulp_rain_meter
} // namespace esphome
