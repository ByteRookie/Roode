import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from ..persisted_switch import PERSISTED_SWITCH_SCHEMA, new_persisted_switch
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["switch", "persisted_switch"]

CONF_LOG_FALLBACK = "log_fallback_events"
CONF_FORCE_SINGLE_CORE = "force_single_core"
CONF_CALIBRATION_PERSISTENCE = "calibration_persistence"
CONF_ZONES_INVERT = "zones_invert"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
    cv.Optional(CONF_LOG_FALLBACK): PERSISTED_SWITCH_SCHEMA,
    cv.Optional(CONF_FORCE_SINGLE_CORE): PERSISTED_SWITCH_SCHEMA,
    cv.Optional(CONF_CALIBRATION_PERSISTENCE): PERSISTED_SWITCH_SCHEMA,
    cv.Optional(CONF_ZONES_INVERT): PERSISTED_SWITCH_SCHEMA,
})

async def setup_log_fallback(config, hub):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_log_fallback_switch(sw))

async def setup_force_single_core(config, hub):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_force_single_core_switch(sw))

async def setup_calibration_persistence(config, hub):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_calibration_persistence_switch(sw))

async def setup_zones_invert(config, hub):
    sw = await new_persisted_switch(config)
    cg.add(hub.set_invert_direction_switch(sw))

async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_LOG_FALLBACK in config:
        await setup_log_fallback(config[CONF_LOG_FALLBACK], hub)
    if CONF_FORCE_SINGLE_CORE in config:
        await setup_force_single_core(config[CONF_FORCE_SINGLE_CORE], hub)
    if CONF_CALIBRATION_PERSISTENCE in config:
        await setup_calibration_persistence(config[CONF_CALIBRATION_PERSISTENCE], hub)
    if CONF_ZONES_INVERT in config:
        await setup_zones_invert(config[CONF_ZONES_INVERT], hub)
