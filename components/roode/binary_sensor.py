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

CONF_OCCUPANCY = "presence"
CONF_XSHUT_STATE = "sensor_xshut_state"

OCCUPANCY_SCHEMA = BINARY_SENSOR_SCHEMA.extend(
    {
        cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_OCCUPANCY): validate_device_class,
    }
)

ZONE_SCHEMA = NullableSchema({cv.Optional(CONF_OCCUPANCY): OCCUPANCY_SCHEMA})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_OCCUPANCY): OCCUPANCY_SCHEMA,
        cv.Optional(CONF_XSHUT_STATE): BINARY_SENSOR_SCHEMA,
        cv.Optional(CONF_ZONES, default={}): NullableSchema(
            {
                cv.Optional(CONF_ENTRY_ZONE, default={}): ZONE_SCHEMA,
                cv.Optional(CONF_EXIT_ZONE, default={}): ZONE_SCHEMA,
            }
        ),
    }
)


async def to_code(config: Dict):
    roode = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_OCCUPANCY in config:
        cg.add(
            roode.set_presence_sensor_binary_sensor(
                await new_binary_sensor(config[CONF_OCCUPANCY])
            )
        )
    if CONF_XSHUT_STATE in config:
        cg.add(
            roode.set_xshut_state_binary_sensor(
                await new_binary_sensor(config[CONF_XSHUT_STATE])
            )
        )
    await to_code_zone(CONF_ENTRY_ZONE, config, roode)
    await to_code_zone(CONF_EXIT_ZONE, config, roode)


async def to_code_zone(name: str, config: Dict, roode: cg.MockObj):
    zone_config = config.get(CONF_ZONES, {}).get(name, {})
    zone_var = cg.MockObj(f"{roode}->{name}", "->")
    if CONF_OCCUPANCY in zone_config:
        cg.add(
            zone_var.set_occupancy_sensor(
                await new_binary_sensor(zone_config[CONF_OCCUPANCY])
            )
        )
