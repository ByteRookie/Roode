from typing import OrderedDict

from esphome.core import ID

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ICON,
    CONF_MAX_VALUE,
    CONF_ID,
    CONF_NAME,
    CONF_DISABLED_BY_DEFAULT,
    CONF_MODE,
    CONF_RESTORE_VALUE,
)
from esphome.components.number import NumberMode
from esphome.cpp_generator import MockObj

from ..persisted_number import (
    PERSISTED_NUMBER_SCHEMA,
    new_persisted_number,
    PersistedNumber,
)
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

SETTINGS = {
    "invalid_distance_limit": {"min": 1, "max": 100, "step": 1, "default": 10},
    "restart_timeout": {"min": 1, "max": 120, "step": 1, "default": 30},
    "sampling": {"min": 1, "max": 6, "step": 1, "default": 2},
    "filter_window": {"min": 3, "max": 9, "step": 2, "default": 5},
    "calibration_offset": {"min": -50, "max": 50, "step": 1, "default": 0},
    "calibration_crosstalk": {"min": 0, "max": 100000, "step": 1000, "default": 0},
    "detection_min_threshold": {"min": 0, "max": 100, "step": 1, "default": 15},
    "detection_max_threshold": {"min": 0, "max": 100, "step": 1, "default": 80},
    "roi_height": {"min": 4, "max": 16, "step": 1, "default": 16},
    "roi_width": {"min": 4, "max": 16, "step": 1, "default": 6},
}

CONF_PEOPLE_COUNTER = "people_counter"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_PEOPLE_COUNTER): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:counter"): cv.icon,  # new default
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(-128, 128),
            }
        ),
    }
)


async def setup_people_counter(config: OrderedDict, hub: MockObj):
    counter = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE], default_value=0
    )
    cg.add(hub.set_people_counter(counter))


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_PEOPLE_COUNTER in config:
        await setup_people_counter(config[CONF_PEOPLE_COUNTER], hub)
    for key, opts in SETTINGS.items():
        conf = OrderedDict()
        conf[CONF_ID] = ID(None, is_declaration=True, type=PersistedNumber)
        conf[CONF_NAME] = f"Roode {key}"
        conf[CONF_DISABLED_BY_DEFAULT] = False
        conf[CONF_MODE] = NumberMode.NUMBER_MODE_BOX
        conf[CONF_RESTORE_VALUE] = True
        num = await new_persisted_number(
            conf,
            min_value=opts["min"],
            max_value=opts["max"],
            step=opts["step"],
            default_value=opts.get("default"),
        )
        cg.add(getattr(hub, f"set_{key}_number")(num))
