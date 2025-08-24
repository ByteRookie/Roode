"""VL53L1X distance sensor platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    ICON_RULER,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["vl53l1x"]

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="mm",
    icon=ICON_RULER,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    await sensor.new_sensor(config)
