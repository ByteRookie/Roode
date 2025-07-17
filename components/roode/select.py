from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_NAME

from ..persisted_select import PERSISTED_SELECT_SCHEMA, new_persisted_select
from . import Roode, CONF_ROODE_ID, CONF_FILTER_MODE, FILTER_MODES

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select", "persisted_select"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(
            CONF_FILTER_MODE,
            default={CONF_NAME: "Filter Mode", "options": list(FILTER_MODES.keys())},
        ): PERSISTED_SELECT_SCHEMA.extend(
            {cv.Optional(CONF_ICON, default="mdi:filter"): cv.icon}
        ),
    }
)

async def setup_filter_mode(config: OrderedDict, hub: cg.Pvariable):
    sel = await new_persisted_select(config)
    cg.add(hub.set_filter_mode_select(sel))

async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    await setup_filter_mode(config[CONF_FILTER_MODE], hub)
