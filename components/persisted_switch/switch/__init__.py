from esphome.components import switch

from .. import new_persisted_switch

AUTO_LOAD = ["persisted_switch"]

CONFIG_SCHEMA = switch.switch_schema(switch.Switch)


async def to_code(config):
    return await new_persisted_switch(config)
