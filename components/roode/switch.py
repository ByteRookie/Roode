import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID
from . import Roode, CONF_ROODE_ID, roode_ns

DEPENDENCIES = ["roode"]

CONF_WEB_PORTAL = "web_portal"

PortalSwitch = roode_ns.class_("PortalSwitch", switch.Switch)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_WEB_PORTAL): switch.switch_schema(
            PortalSwitch, icon="mdi:web"
        ),
    }
)


async def _setup_switch(config, key, hub):
    if key in config:
        conf = config[key]
        sw = cg.new_Pvariable(conf[CONF_ID])
        await switch.register_switch(sw, conf)
        cg.add(getattr(hub, f"set_{key}_switch")(sw))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    await _setup_switch(config, CONF_WEB_PORTAL, hub)
