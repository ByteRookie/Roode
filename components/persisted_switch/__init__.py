import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

PersistedSwitch = switch.switch_ns.class_(
    "PersistedSwitch", switch.Switch, cg.Component
)

PERSISTED_SWITCH_SCHEMA = switch.switch_schema(PersistedSwitch).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedSwitch),
    }
)


async def new_persisted_switch(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    return var
