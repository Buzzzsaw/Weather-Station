#pragma once

#ifndef HEADER_H
#define HEADER_H

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ulp_rain_meter {

class UlpRainMeter : public sensor::Sensor, public PollingComponent {
public:
  UlpRainMeter() : PollingComponent(15000) {}

  void setup();
  void update();

private:
  float pulse_count = 0;

  void setupUlpProgram();
};

} // namespace ulp_rain_meter
} // namespace esphome

#endif // HEADER_H
