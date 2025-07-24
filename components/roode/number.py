from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_MAX_VALUE
from esphome.cpp_generator import MockObj

from ..persisted_number import PERSISTED_NUMBER_SCHEMA, new_persisted_number
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

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
    max_val = config.get(CONF_MAX_VALUE, 10)
    clean = OrderedDict({k: v for k, v in config.items() if k != CONF_MAX_VALUE and k != CONF_NAME})
    # Use the ID as the default name so each device gets a unique entity
    clean[CONF_NAME] = str(config[CONF_ID])
    counter = await new_persisted_number(
        clean, min_value=0, step=1, max_value=max_val
    )
    cg.add(hub.set_people_counter(counter))


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_PEOPLE_COUNTER in config:
        await setup_people_counter(config[CONF_PEOPLE_COUNTER], hub)
