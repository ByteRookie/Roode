from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_MAX_VALUE, CONF_NAME
from esphome.cpp_generator import MockObj

from ..persisted_number import PERSISTED_NUMBER_SCHEMA, new_persisted_number
from . import (
    Roode,
    CONF_ROODE_ID,
    CONF_FILTER_WINDOW,
)

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

CONF_PEOPLE_COUNTER = "people_counter"
CONF_SAMPLING_SIZE = "sampling_size"
CONF_ENTRY_MIN_THRESHOLD = "entry_min_threshold"
CONF_ENTRY_MAX_THRESHOLD = "entry_max_threshold"
CONF_EXIT_MIN_THRESHOLD = "exit_min_threshold"
CONF_EXIT_MAX_THRESHOLD = "exit_max_threshold"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(
            CONF_PEOPLE_COUNTER,
            default={CONF_NAME: "People Count"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:counter"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(min=-128, max=128),
            }
        ),
        cv.Optional(
            CONF_SAMPLING_SIZE,
            default={CONF_NAME: "Sampling Size"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:chart-histogram"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(min=1, max=10),
            }
        ),
        cv.Optional(
            CONF_FILTER_WINDOW,
            default={CONF_NAME: "Filter Window"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:filter"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(min=1, max=10),
            }
        ),
        cv.Optional(
            CONF_ENTRY_MIN_THRESHOLD,
            default={CONF_NAME: "Entry Min %"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:arrow-collapse-down"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 100): cv.int_range(min=0, max=100),
            }
        ),
        cv.Optional(
            CONF_ENTRY_MAX_THRESHOLD,
            default={CONF_NAME: "Entry Max %"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:arrow-collapse-up"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 100): cv.int_range(min=0, max=100),
            }
        ),
        cv.Optional(
            CONF_EXIT_MIN_THRESHOLD,
            default={CONF_NAME: "Exit Min %"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:arrow-collapse-down"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 100): cv.int_range(min=0, max=100),
            }
        ),
        cv.Optional(
            CONF_EXIT_MAX_THRESHOLD,
            default={CONF_NAME: "Exit Max %"},
        ): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:arrow-collapse-up"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 100): cv.int_range(min=0, max=100),
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


async def setup_filter_window(config: OrderedDict, hub: MockObj):
    win = await new_persisted_number(
        config, min_value=1, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_filter_window_number(win))


async def setup_entry_threshold_min(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_entry_min_threshold_number(num))


async def setup_entry_threshold_max(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_entry_max_threshold_number(num))


async def setup_exit_threshold_min(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_exit_min_threshold_number(num))


async def setup_exit_threshold_max(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE]
    )
    cg.add(hub.set_exit_max_threshold_number(num))


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    await setup_people_counter(config[CONF_PEOPLE_COUNTER], hub)
    await setup_sampling_size(config[CONF_SAMPLING_SIZE], hub)
    await setup_filter_window(config[CONF_FILTER_WINDOW], hub)
    await setup_entry_threshold_min(config[CONF_ENTRY_MIN_THRESHOLD], hub)
    await setup_entry_threshold_max(config[CONF_ENTRY_MAX_THRESHOLD], hub)
    await setup_exit_threshold_min(config[CONF_EXIT_MIN_THRESHOLD], hub)
    await setup_exit_threshold_max(config[CONF_EXIT_MAX_THRESHOLD], hub)
