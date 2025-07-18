import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import (
    CONF_ID,
    CONF_ICON,
    CONF_INITIAL_OPTION,
    CONF_NAME,
    CONF_OPTIONS,
    CONF_RESTORE_VALUE,
)

PersistedSelect = select.select_ns.class_(
    "PersistedSelect", select.Select, cg.Component
)

PERSISTED_SELECT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PersistedSelect),
        cv.Optional(CONF_NAME): cv.string,
        cv.Required(CONF_OPTIONS): cv.ensure_list(cv.string),
        cv.Optional(CONF_ICON): cv.icon,
        cv.Optional(CONF_INITIAL_OPTION): cv.string,
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def new_persisted_select(config):
    var = await select.new_select(config)
    await cg.register_component(var, config)
    if CONF_RESTORE_VALUE in config:
        cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    return var
