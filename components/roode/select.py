import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import Roode, CONF_ROODE_ID, roode_ns

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select"]

SensorModeSelect = roode_ns.class_("SensorModeSelect", select.Select, cg.Component)

OPTIONS = ["no_sensors", "all_sensors"]

CONFIG_SCHEMA = select.select_schema(
    SensorModeSelect,
    icon="mdi:menu-down",
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend({cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode)})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ROODE_ID])
    var = await select.new_select(config, options=OPTIONS)
    await cg.register_component(var, config)
    cg.add(var.set_parent(parent))
    cg.add(parent.set_sensor_mode_select(var))
