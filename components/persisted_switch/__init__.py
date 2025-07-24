import esphome.codegen as cg
from esphome.components import switch
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_RESTORE_MODE,
    CONF_DISABLED_BY_DEFAULT,
)
from esphome.core import CORE

async def new_persisted_switch(config):
    config = {
        **config,
        CONF_NAME: config.get(CONF_NAME, str(config[CONF_ID])),
        CONF_RESTORE_MODE: config.get(CONF_RESTORE_MODE, "RESTORE_DEFAULT_OFF"),
        CONF_DISABLED_BY_DEFAULT: config.get(CONF_DISABLED_BY_DEFAULT, False),
    }
    var = cg.new_Pvariable(config[CONF_ID], switch.Switch)
    CORE.component_ids.add(config[CONF_ID].id)
    await cg.register_component(var, config)
    await switch.register_switch(var, config)
    return var
