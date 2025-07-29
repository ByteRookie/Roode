import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_OPTIONS

from .. import new_persisted_select

AUTO_LOAD = ["persisted_select"]

CONFIG_SCHEMA = select.select_schema(select.Select).extend(
    {cv.Required(CONF_OPTIONS): cv.ensure_list(cv.string)}
)


async def to_code(config):
    return await new_persisted_select(config, options=config[CONF_OPTIONS])
