import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT, CONF_USERNAME, CONF_PASSWORD

web_portal_ns = cg.esphome_ns.namespace("web_portal")
WebPortal = web_portal_ns.class_("WebPortal", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WebPortal),
        cv.Optional(CONF_PORT, default=80): cv.port,
        cv.Optional(CONF_USERNAME): cv.string,
        cv.Optional(CONF_PASSWORD): cv.string,
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_PORT])
    if CONF_USERNAME in config and CONF_PASSWORD in config:
        cg.add(var.set_auth(config[CONF_USERNAME], config[CONF_PASSWORD]))
    await cg.register_component(var, config)
