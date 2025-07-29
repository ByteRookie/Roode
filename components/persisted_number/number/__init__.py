import esphome.config_validation as cv
from esphome.const import CONF_MIN_VALUE, CONF_MAX_VALUE, CONF_STEP

from .. import new_persisted_number, PERSISTED_NUMBER_SCHEMA

AUTO_LOAD = ["persisted_number"]

CONFIG_SCHEMA = PERSISTED_NUMBER_SCHEMA.extend(
    {
        cv.Required(CONF_MIN_VALUE): cv.float_,
        cv.Required(CONF_MAX_VALUE): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
    }
)


async def to_code(config):
    return await new_persisted_number(
        {k: v for k, v in config.items() if k not in (CONF_MIN_VALUE, CONF_MAX_VALUE, CONF_STEP)},
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
