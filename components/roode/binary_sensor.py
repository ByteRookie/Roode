from typing import Dict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.binary_sensor import (
    BINARY_SENSOR_SCHEMA,
    validate_device_class,
    new_binary_sensor,
)
from esphome.const import CONF_DEVICE_CLASS, DEVICE_CLASS_OCCUPANCY
from . import (
    Roode,
    CONF_ROODE_ID,
    CONF_ZONES,
    CONF_ENTRY_ZONE,
    CONF_EXIT_ZONE,
    NullableSchema,
)

DEPENDENCIES = ["roode"]

CONF_PRESENCE = "presence"
CONF_XSHUT_STATE = "sensor_xshut_state"
TYPES = [CONF_PRESENCE, CONF_XSHUT_STATE]

OCCUPANCY_SCHEMA = BINARY_SENSOR_SCHEMA.extend(
    {cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_OCCUPANCY): validate_device_class}
)

ZONE_SCHEMA = NullableSchema({cv.Optional(CONF_PRESENCE): OCCUPANCY_SCHEMA})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_PRESENCE): OCCUPANCY_SCHEMA,
        cv.Optional(CONF_XSHUT_STATE): BINARY_SENSOR_SCHEMA,
        cv.Optional(CONF_ZONES, default={}): NullableSchema(
            {
                cv.Optional(CONF_ENTRY_ZONE, default={}): ZONE_SCHEMA,
                cv.Optional(CONF_EXIT_ZONE, default={}): ZONE_SCHEMA,
            }
        ),
    }
)


async def setup_conf(config, key, hub):
    if key in config:
        sens = await new_binary_sensor(config[key])
        cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))


async def to_code(config: Dict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
    zones = config.get(CONF_ZONES, {})
    entry = zones.get(CONF_ENTRY_ZONE, {})
    if CONF_PRESENCE in entry:
        sens = await new_binary_sensor(entry[CONF_PRESENCE])
        cg.add(hub.set_entry_presence_binary_sensor(sens))
    exit_ = zones.get(CONF_EXIT_ZONE, {})
    if CONF_PRESENCE in exit_:
        sens = await new_binary_sensor(exit_[CONF_PRESENCE])
        cg.add(hub.set_exit_presence_binary_sensor(sens))

