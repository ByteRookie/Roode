from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import Roode, roode_ns, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["switch"]

InvertDirectionSwitch = roode_ns.class_("InvertDirectionSwitch", switch.Switch)
CalibrationPersistenceSwitch = roode_ns.class_("CalibrationPersistenceSwitch", switch.Switch)
PerformanceModeSwitch = roode_ns.class_("PerformanceModeSwitch", switch.Switch)

CONF_INVERT_DIRECTION = "invert_direction"
CONF_CALIBRATION_PERSISTENCE = "calibration_persistence"
CONF_PERFORMANCE_MODE = "performance_mode"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_INVERT_DIRECTION): switch.switch_schema(
            InvertDirectionSwitch,
            icon="mdi:swap-horizontal",
        ),
        cv.Optional(CONF_CALIBRATION_PERSISTENCE): switch.switch_schema(
            CalibrationPersistenceSwitch,
            icon="mdi:content-save",
        ),
        cv.Optional(CONF_PERFORMANCE_MODE): switch.switch_schema(
            PerformanceModeSwitch,
            icon="mdi:speedometer",
        ),
    }
)


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])

    if CONF_INVERT_DIRECTION in config:
        conf = config[CONF_INVERT_DIRECTION]
        sw = await switch.new_switch(conf)
        cg.add(sw.set_roode_hub(hub))
        cg.add(hub.set_invert_direction_switch(sw))

    if CONF_CALIBRATION_PERSISTENCE in config:
        conf = config[CONF_CALIBRATION_PERSISTENCE]
        sw = await switch.new_switch(conf)
        cg.add(sw.set_roode_hub(hub))
        cg.add(hub.set_calibration_persistence_switch(sw))

    if CONF_PERFORMANCE_MODE in config:
        conf = config[CONF_PERFORMANCE_MODE]
        sw = await switch.new_switch(conf)
        cg.add(sw.set_roode_hub(hub))
        cg.add(hub.set_performance_mode_switch(sw))
