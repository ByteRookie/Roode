import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_RESTORE_VALUE

PersistedSwitch = switch.switch_ns.class_("PersistedSwitch", switch.Switch, cg.Component)

PERSISTED_SWITCH_SCHEMA = switch.switch_schema(PersistedSwitch).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedSwitch),
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    }
)

async def new_persisted_switch(config):
    var = cg.new_Pvariable(config[CONF_ID], PersistedSwitch)
    await switch.register_switch(var, config)
    await cg.register_component(var, config)
    if CONF_RESTORE_VALUE in config:
        cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    return var
