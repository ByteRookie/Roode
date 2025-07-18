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

PERSISTED_SELECT_SCHEMA = select.select_schema(PersistedSelect).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedSelect),
        cv.Required(CONF_OPTIONS): cv.ensure_list(cv.string),
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    }
)


async def new_persisted_select(config):
    var = cg.new_Pvariable(config[CONF_ID], PersistedSelect)
    await select.register_select(var, config, options=config[CONF_OPTIONS])
    await cg.register_component(var, config)
    if CONF_RESTORE_VALUE in config:
        cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    return var
