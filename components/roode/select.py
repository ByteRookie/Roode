import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from ..persisted_select import PERSISTED_SELECT_SCHEMA, new_persisted_select
from . import (
    Roode,
    CONF_ROODE_ID,
    CONF_FILTER_MODE,
    FILTER_MODES,
    CONF_REFRESH_MODE,
    CONF_CALIBRATION_RANGING,
)

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select", "persisted_select"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_FILTER_MODE): PERSISTED_SELECT_SCHEMA.extend(
            {
                cv.Optional(
                    "options", default=list(FILTER_MODES.keys())
                ): cv.ensure_list(cv.one_of(*FILTER_MODES.keys(), lower=True))
            }
        ),
        cv.Optional(CONF_CALIBRATION_RANGING): PERSISTED_SELECT_SCHEMA.extend(
            {
                cv.Optional(
                    "options", default=["auto", "short", "medium", "long"]
                ): cv.ensure_list(
                    cv.one_of("auto", "short", "medium", "long", lower=True)
                )
            }
        ),
        cv.Optional(CONF_REFRESH_MODE): PERSISTED_SELECT_SCHEMA.extend(
            {
                cv.Optional(
                    "options", default=["interrupt", "polling"]
                ): cv.ensure_list(cv.one_of("interrupt", "polling", lower=True))
            }
        ),
    }
)


async def setup_filter_mode(config, hub):
    sel = await new_persisted_select(config)
    cg.add(hub.set_filter_mode_select(sel))


async def setup_ranging(config, hub):
    sel = await new_persisted_select(config)
    cg.add(hub.set_ranging_select(sel))


async def setup_refresh(config, hub):
    sel = await new_persisted_select(config)
    cg.add(hub.set_refresh_select(sel))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_FILTER_MODE in config:
        await setup_filter_mode(config[CONF_FILTER_MODE], hub)
    if CONF_CALIBRATION_RANGING in config:
        await setup_ranging(config[CONF_CALIBRATION_RANGING], hub)
    if CONF_REFRESH_MODE in config:
        await setup_refresh(config[CONF_REFRESH_MODE], hub)
