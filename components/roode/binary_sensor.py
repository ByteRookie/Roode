import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
)
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]

CONF_PRESENCE = "presence_sensor"
CONF_XSHUT_STATE = "sensor_xshut_state"
TYPES = [CONF_PRESENCE, CONF_XSHUT_STATE]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
            binary_sensor.BinarySensor
        ).extend(
            {
                cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
            }
        ),
        cv.Optional(CONF_XSHUT_STATE): binary_sensor.binary_sensor_schema(
            binary_sensor.BinarySensor
        ).extend(
            {
                cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
            }
        ),
    }

)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        sens = cg.new_Pvariable(conf[CONF_ID])
        await binary_sensor.register_binary_sensor(sens, conf)
        cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
