import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    CONF_ICON,
    CONF_ENTITY_CATEGORY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_CONFIG,
)
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]

VERSION = "version"
ENTRY_EXIT_EVENT = "entry_exit_event"
STATUS = "sensor_status"
FEATURES = "enabled_features"
SENSOR_NAME = "sensor_name"
OPTIONAL_SENSORS = "optional_sensors"

TYPES = [VERSION, ENTRY_EXIT_EVENT, STATUS, FEATURES, SENSOR_NAME, OPTIONAL_SENSORS]

_defined_roode_ids = set()

def _validate_single_block(config):
    rid = str(config[CONF_ROODE_ID].id)
    if rid in _defined_roode_ids:
        raise cv.Invalid(
            "Only one 'platform: roode' text_sensor block is allowed per Roode id"
        )
    _defined_roode_ids.add(rid)
    return config

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(VERSION): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:git"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_DIAGNOSTIC
                ): cv.entity_category,
            }
        ),
        cv.Optional(ENTRY_EXIT_EVENT): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:sign-direction"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_DIAGNOSTIC
                ): cv.entity_category,
            }
        ),
        cv.Optional(STATUS): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:check-circle"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_DIAGNOSTIC
                ): cv.entity_category,
            }
        ),
        cv.Optional(
            FEATURES,
            default={"name": "Roode Enabled Features"},
        ): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:cog"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_CONFIG
                ): cv.entity_category,
            }
        ),
        cv.Optional(
            SENSOR_NAME,
            default={"name": "Roode Sensor Name"},
        ): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:identifier"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_CONFIG
                ): cv.entity_category,
            }
        ),
        cv.Optional(
            OPTIONAL_SENSORS,
            default={"name": "Roode Optional Sensors"},
        ): text_sensor.text_sensor_schema().extend(
            {
                cv.Optional(CONF_ICON, default="mdi:format-list-checks"): cv.icon,
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(
                    CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_CONFIG
                ): cv.entity_category,
            }
        ),
    }
    ),
    _validate_single_block,
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        sens = cg.new_Pvariable(conf[CONF_ID])
        await text_sensor.register_text_sensor(sens, conf)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
