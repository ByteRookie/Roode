from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_NAME

from ..persisted_switch import PERSISTED_SWITCH_SCHEMA, new_persisted_switch
from . import Roode, CONF_ROODE_ID, CONF_LOG_FALLBACK

CONF_INVERT_DIRECTION = "invert_direction"

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["switch", "persisted_switch"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(
            CONF_LOG_FALLBACK,
            default={CONF_NAME: "Log Fallback Events"},
        ): PERSISTED_SWITCH_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:file-alert-outline"): cv.icon,
            }
        ),
        cv.Optional(
            CONF_INVERT_DIRECTION,
            default={CONF_NAME: "Invert Direction"},
        ): PERSISTED_SWITCH_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:swap-horizontal"): cv.icon,
            }
        ),
    }
)

async def setup_log_switch(config: OrderedDict, hub: cg.Pvariable):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_log_fallback_switch(sw))


async def setup_invert_switch(config: OrderedDict, hub: cg.Pvariable):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_invert_direction_switch(sw))

async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    await setup_log_switch(config[CONF_LOG_FALLBACK], hub)
    await setup_invert_switch(config[CONF_INVERT_DIRECTION], hub)
