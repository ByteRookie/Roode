import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID

AUTO_LOAD = ["select"]

PersistedSelect = select.select_ns.class_(
    "PersistedSelect", select.Select, cg.Component
)

PERSISTED_SELECT_SCHEMA = select.select_schema(PersistedSelect).extend(
    {
        cv.GenerateID(): cv.declare_id(PersistedSelect),
        cv.Required("options"): cv.ensure_list(cv.string),
    }
)


async def new_persisted_select(config):
    var = await select.new_select(config, options=config["options"])
    await cg.register_component(var, config)
    return var
