import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, CONF_ICON
from . import Roode, roode_ns

PortalSwitch = roode_ns.class_("PortalSwitch", switch.Switch, cg.Component)

CONF_PORTAL = "portal"

CONFIG_SCHEMA = switch.SWITCH_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(PortalSwitch),
        cv.GenerateID("roode_id"): cv.use_id(Roode),
        cv.Optional(CONF_ICON, default="mdi:web"): cv.icon,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    parent = await cg.get_variable(config["roode_id"])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await switch.register_switch(var, config)
    cg.add(parent.set_portal_switch(var))
    cg.add(var.set_parent(parent))
