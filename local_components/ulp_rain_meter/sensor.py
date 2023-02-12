import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@buzzzsaw"]
DEPENDENCIES = ["esp32"]

ulp_rain_meter_ns = cg.esphome_ns.namespace("ulp_rain_meter")
UlpRainMeter = ulp_rain_meter_ns.class_("UlpRainMeter", sensor.Sensor, cg.PollingComponent)

CONFIG_SCHEMA = sensor.sensor_schema(UlpRainMeter)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
