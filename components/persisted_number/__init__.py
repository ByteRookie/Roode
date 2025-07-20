from typing import Optional, OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_RESTORE_VALUE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

try:
    from esphome.const import ENTITY_CATEGORY_CONFIG
    # Older ESPHome versions may define the constant but still reject the value
    # during validation. Check that it is actually accepted by the schema and
    # fall back to the diagnostic category otherwise.
    try:
        cv.entity_category("config")
    except Exception:
        ENTITY_CATEGORY_CONFIG = ENTITY_CATEGORY_DIAGNOSTIC
except ImportError:  # Pre-config ESPHome
    ENTITY_CATEGORY_CONFIG = ENTITY_CATEGORY_DIAGNOSTIC

PersistedNumber = number.number_ns.class_(
    "PersistedNumber", number.Number, cg.Component
)

PERSISTED_NUMBER_SCHEMA = number.number_schema(
    PersistedNumber, entity_category=ENTITY_CATEGORY_CONFIG
).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedNumber),
        cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
    }
)


async def new_persisted_number(
    config: OrderedDict,
    min_value: float,
    max_value: float,
    step: Optional[float] = None,
):
    var = await number.new_number(
        config, min_value=min_value, max_value=max_value, step=step
    )
    await cg.register_component(var, config)
    if CONF_RESTORE_VALUE in config:
        cg.add(var.set_restore_value(config[CONF_RESTORE_VALUE]))
    return var
