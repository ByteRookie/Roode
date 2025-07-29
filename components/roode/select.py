import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from ..persisted_select import PERSISTED_SELECT_SCHEMA, new_persisted_select
from . import Roode, CONF_ROODE_ID, CONF_FILTER_MODE, FILTER_MODES

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select", "persisted_select"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_FILTER_MODE): PERSISTED_SELECT_SCHEMA.extend(
            {
                cv.Required("options", default=list(FILTER_MODES.keys())):
                    cv.ensure_list(cv.one_of(*FILTER_MODES.keys(), lower=True))
            }
        ),
    }
)

async def setup_filter_mode(config, hub):
    sel = await new_persisted_select(config)
    cg.add(hub.set_filter_mode_select(sel))

async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_FILTER_MODE in config:
        await setup_filter_mode(config[CONF_FILTER_MODE], hub)
