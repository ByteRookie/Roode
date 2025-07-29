from esphome.components import switch as esphome_switch

from .. import new_persisted_switch

AUTO_LOAD = ["persisted_switch"]

CONFIG_SCHEMA = esphome_switch.switch_schema(esphome_switch.Switch)


async def to_code(config):
    return await new_persisted_switch(config)
