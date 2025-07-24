from typing import Optional, OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_RESTORE_VALUE,
    CONF_DISABLED_BY_DEFAULT,
)

PersistedNumber = number.number_ns.class_(
    "PersistedNumber", number.Number, cg.Component
)

PERSISTED_NUMBER_SCHEMA = number.number_schema(PersistedNumber).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedNumber),
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=False): cv.boolean,
    }
)


async def new_persisted_number(
    config: OrderedDict,
    min_value: float,
    max_value: float,
    step: Optional[float] = None,
):
    config = {
        **config,
        CONF_NAME: config.get(CONF_NAME, str(config[CONF_ID])),
        CONF_RESTORE_VALUE: config.get(CONF_RESTORE_VALUE, True),
        CONF_DISABLED_BY_DEFAULT: config.get(CONF_DISABLED_BY_DEFAULT, False),
    }
    config = cv.Schema(PERSISTED_NUMBER_SCHEMA, extra=cv.ALLOW_EXTRA)(config)
    var = await number.new_number(
        config, min_value=min_value, max_value=max_value, step=step
    )
    from esphome.core import CORE
    CORE.component_ids.add(config[CONF_ID].id)
    await cg.register_component(var, config)
    cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    return var
