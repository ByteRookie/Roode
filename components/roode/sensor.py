import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    ICON_ARROW_EXPAND_VERTICAL,
    ICON_NEW_BOX,
    ICON_RULER,
    STATE_CLASS_MEASUREMENT,
    UNIT_EMPTY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]

CONF_DISTANCE_entry = "distance_entry"
CONF_DISTANCE_exit = "distance_exit"
CONF_MAX_THRESHOLD_entry = "max_threshold_entry"
CONF_MAX_THRESHOLD_exit = "max_threshold_exit"
CONF_MIN_THRESHOLD_entry = "min_threshold_entry"
CONF_MIN_THRESHOLD_exit = "min_threshold_exit"
CONF_ROI_HEIGHT_entry = "roi_height_entry"
CONF_ROI_WIDTH_entry = "roi_width_entry"
CONF_ROI_HEIGHT_exit = "roi_height_exit"
CONF_ROI_WIDTH_exit = "roi_width_exit"
SENSOR_STATUS = "sensor_status"
CONF_LOOP_TIME = "loop_time"
CONF_CPU_USAGE = "cpu_usage"
CONF_RAM_FREE = "ram_free"
CONF_FLASH_FREE = "flash_free"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.Optional(CONF_DISTANCE_entry): sensor.sensor_schema(
            icon=ICON_RULER,
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_DISTANCE_exit): sensor.sensor_schema(
            icon=ICON_RULER,
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MAX_THRESHOLD_entry): sensor.sensor_schema(
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MAX_THRESHOLD_exit): sensor.sensor_schema(
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIN_THRESHOLD_entry): sensor.sensor_schema(
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_MIN_THRESHOLD_exit): sensor.sensor_schema(
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ROI_HEIGHT_entry): sensor.sensor_schema(
            icon="mdi:table-row-height",
            unit_of_measurement="px",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ROI_WIDTH_entry): sensor.sensor_schema(
            icon="mdi:table-column-width",
            unit_of_measurement="px",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ROI_HEIGHT_exit): sensor.sensor_schema(
            icon="mdi:table-row-height",
            unit_of_measurement="px",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_ROI_WIDTH_exit): sensor.sensor_schema(
            icon="mdi:table-column-width",
            unit_of_measurement="px",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(SENSOR_STATUS): sensor.sensor_schema(
            icon="mdi:check-circle",
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_LOOP_TIME): sensor.sensor_schema(
            icon="mdi:progress-clock",
            unit_of_measurement="ms",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_CPU_USAGE): sensor.sensor_schema(
            icon="mdi:cpu-64-bit",
            unit_of_measurement="%",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_RAM_FREE): sensor.sensor_schema(
            icon="mdi:memory",
            unit_of_measurement="B",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_FLASH_FREE): sensor.sensor_schema(
            icon=ICON_NEW_BOX,
            unit_of_measurement="B",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
    }
)


async def to_code(config):
    var = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_DISTANCE_entry in config:
        distance = await sensor.new_sensor(config[CONF_DISTANCE_entry])
        cg.add(var.set_distance_entry(distance))
    if CONF_DISTANCE_exit in config:
        distance = await sensor.new_sensor(config[CONF_DISTANCE_exit])
        cg.add(var.set_distance_exit(distance))
    if CONF_MAX_THRESHOLD_entry in config:
        count = await sensor.new_sensor(config[CONF_MAX_THRESHOLD_entry])
        cg.add(var.set_max_threshold_entry_sensor(count))
    if CONF_MAX_THRESHOLD_exit in config:
        count = await sensor.new_sensor(config[CONF_MAX_THRESHOLD_exit])
        cg.add(var.set_max_threshold_exit_sensor(count))
    if CONF_MIN_THRESHOLD_entry in config:
        count = await sensor.new_sensor(config[CONF_MIN_THRESHOLD_entry])
        cg.add(var.set_min_threshold_entry_sensor(count))
    if CONF_MIN_THRESHOLD_exit in config:
        count = await sensor.new_sensor(config[CONF_MIN_THRESHOLD_exit])
        cg.add(var.set_min_threshold_exit_sensor(count))
    if CONF_ROI_HEIGHT_entry in config:
        count = await sensor.new_sensor(config[CONF_ROI_HEIGHT_entry])
        cg.add(var.set_entry_roi_height_sensor(count))
    if CONF_ROI_WIDTH_entry in config:
        count = await sensor.new_sensor(config[CONF_ROI_WIDTH_entry])
        cg.add(var.set_entry_roi_width_sensor(count))
    if CONF_ROI_HEIGHT_exit in config:
        count = await sensor.new_sensor(config[CONF_ROI_HEIGHT_exit])
        cg.add(var.set_exit_roi_height_sensor(count))
    if CONF_ROI_WIDTH_exit in config:
        count = await sensor.new_sensor(config[CONF_ROI_WIDTH_exit])
        cg.add(var.set_exit_roi_width_sensor(count))
    if SENSOR_STATUS in config:
        count = await sensor.new_sensor(config[SENSOR_STATUS])
        cg.add(var.set_sensor_status_sensor(count))
    if CONF_LOOP_TIME in config:
        count = await sensor.new_sensor(config[CONF_LOOP_TIME])
        cg.add(var.set_loop_time_sensor(count))
    if CONF_CPU_USAGE in config:
        count = await sensor.new_sensor(config[CONF_CPU_USAGE])
        cg.add(var.set_cpu_usage_sensor(count))
    if CONF_RAM_FREE in config:
        count = await sensor.new_sensor(config[CONF_RAM_FREE])
        cg.add(var.set_ram_free_sensor(count))
    if CONF_FLASH_FREE in config:
        count = await sensor.new_sensor(config[CONF_FLASH_FREE])
        cg.add(var.set_flash_free_sensor(count))
