from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_MAX_VALUE, CONF_NAME
from esphome.cpp_generator import MockObj

from ..persisted_number import PERSISTED_NUMBER_SCHEMA, new_persisted_number
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

CONF_PEOPLE_COUNTER = "people_counter"
CONF_SAMPLING_SIZE = "sampling_size"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(
            CONF_PEOPLE_COUNTER,
            default={CONF_NAME: "People Count"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:counter"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(-128, 128),
            }
        ),
        cv.Optional(
            CONF_SAMPLING_SIZE,
            default={CONF_NAME: "Sampling Size"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:chart-histogram"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(1, 10),
            }
        ),
    }
)


async def setup_people_counter(config: OrderedDict, hub: MockObj):
    counter = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_people_counter(counter))


async def setup_sampling_size(config: OrderedDict, hub: MockObj):
    sampler = await new_persisted_number(
        config, min_value=1, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_sampling_size_number(sampler))


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    await setup_people_counter(config[CONF_PEOPLE_COUNTER], hub)
    await setup_sampling_size(config[CONF_SAMPLING_SIZE], hub)
