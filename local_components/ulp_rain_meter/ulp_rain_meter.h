#pragma once

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ulp_rain_meter {

class UlpRainMeter : public sensor::Sensor, public PollingComponent {
public:
  UlpRainMeter() : PollingComponent(5000) {}

  void setup();
  void update();

private:
  void load_ulp();
  void start_ulp();
};

} // namespace ulp_rain_meter
} // namespace esphome
