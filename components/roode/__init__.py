from typing import Dict, Union
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_HEIGHT,
    CONF_ID,
    CONF_INVERT,
    CONF_SENSOR,
    CONF_WIDTH,
)
from ..vl53l1x import distance_as_mm, NullableSchema, VL53L1X

DEPENDENCIES = ["vl53l1x"]
AUTO_LOAD = ["vl53l1x", "sensor", "binary_sensor", "text_sensor", "number"]
MULTI_CONF = True

CONF_ROODE_ID = "roode_id"

roode_ns = cg.esphome_ns.namespace("roode")
Roode = roode_ns.class_("Roode", cg.PollingComponent)

CONF_AUTO = "auto"
CONF_ORIENTATION = "orientation"
CONF_DETECTION_THRESHOLDS = "detection_thresholds"
CONF_ENTRY_ZONE = "entry"
CONF_EXIT_ZONE = "exit"
CONF_CENTER = "center"
CONF_MAX = "max"
CONF_MIN = "min"
CONF_ROI = "roi"
CONF_SAMPLING = "sampling"
CONF_ZONES = "zones"
CONF_CALIBRATION_PERSISTENCE = "calibration_persistence"
CONF_FILTER_MODE = "filter_mode"
CONF_FILTER_WINDOW = "filter_window"
CONF_LOG_FALLBACK = "log_fallback_events"
CONF_FORCE_SINGLE_CORE = "force_single_core"

FilterMode = roode_ns.enum("FilterMode")
FILTER_MODES = {
    "min": FilterMode.FILTER_MIN,
    "median": FilterMode.FILTER_MEDIAN,
    "percentile10": FilterMode.FILTER_PERCENTILE10,
}

Orientation = roode_ns.enum("Orientation")
ORIENTATION_VALUES = {
    "parallel": Orientation.Parallel,
    "perpendicular": Orientation.Perpendicular,
}

# New configuration keys for scheduled recalibration
CONF_AUTO_RECALIBRATE_INTERVAL = "auto_recalibrate_interval"
CONF_RECALIBRATE_ON_TEMP_CHANGE = "recalibrate_on_temp_change"
CONF_MAX_TEMP_DELTA_FOR_RECALIB = "max_temp_delta_for_recalib"
CONF_RECALIBRATE_COOLDOWN = "recalibrate_cooldown"
CONF_USE_LIGHT_SENSOR = "use_light_sensor"
CONF_LUX_LEARNING_WINDOW = "lux_learning_window"
CONF_LUX_SAMPLE_INTERVAL = "lux_sample_interval"
CONF_USE_SUNRISE_PREDICTION = "use_sunrise_prediction"
CONF_LATITUDE = "latitude"
CONF_LONGITUDE = "longitude"
CONF_SUN_ENTITY_ID = "sun_entity_id"
CONF_ALPHA = "alpha"
CONF_BASE_MULTIPLIER = "base_multiplier"
CONF_MAX_MULTIPLIER = "max_multiplier"
CONF_TIME_MULTIPLIER = "time_multiplier"
CONF_COMBINED_MULTIPLIER = "combined_multiplier"
CONF_SUPPRESSION_WINDOW = "suppression_window"
CONF_IDLE_RECALIB_INTERVAL = "idle_recalibrate_interval"
CONF_LUX_SENSOR = "lux_sensor"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"

roi_range = cv.int_range(min=4, max=16)

ROI_SCHEMA = cv.Any(
    NullableSchema(
        {
            cv.Optional(CONF_HEIGHT): roi_range,
            cv.Optional(CONF_WIDTH): roi_range,
            cv.Optional(CONF_CENTER): cv.uint8_t,
        },
    ),
    cv.one_of(CONF_AUTO),
)

threshold = cv.Any(cv.percentage, cv.All(distance_as_mm, cv.uint16_t))

THRESHOLDS_SCHEMA = NullableSchema(
    {
        cv.Optional(CONF_MIN): threshold,
        cv.Optional(CONF_MAX): threshold,
    }
)

ZONE_SCHEMA = NullableSchema(
    {
        cv.Optional(CONF_ROI, default={}): ROI_SCHEMA,
        cv.Optional(CONF_DETECTION_THRESHOLDS, default={}): THRESHOLDS_SCHEMA,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Roode),
        cv.GenerateID(CONF_SENSOR): cv.use_id(VL53L1X),
        cv.Optional(CONF_ORIENTATION, default="parallel"): cv.enum(ORIENTATION_VALUES),
        cv.Optional(CONF_SAMPLING, default=2): cv.All(cv.uint8_t, cv.Range(min=1)),
        cv.Optional(CONF_ROI, default={}): ROI_SCHEMA,
        cv.Optional(CONF_DETECTION_THRESHOLDS, default={}): THRESHOLDS_SCHEMA,
        cv.Optional(CONF_CALIBRATION_PERSISTENCE, default=False): cv.boolean,
        cv.Optional(CONF_FILTER_MODE, default="min"): cv.enum(
            FILTER_MODES, upper=False
        ),
        cv.Optional(CONF_FILTER_WINDOW, default=5): cv.All(cv.uint8_t, cv.Range(min=1)),
        cv.Optional(CONF_LOG_FALLBACK, default=False): cv.boolean,
        cv.Optional(CONF_FORCE_SINGLE_CORE, default=False): cv.boolean,
        # Scheduled recalibration options
        cv.Optional(
            CONF_AUTO_RECALIBRATE_INTERVAL, default="6h"
        ): cv.positive_time_period,
        cv.Optional(CONF_RECALIBRATE_ON_TEMP_CHANGE, default=False): cv.boolean,
        cv.Optional(CONF_MAX_TEMP_DELTA_FOR_RECALIB, default=8): cv.int_,
        cv.Optional(
            CONF_RECALIBRATE_COOLDOWN, default="30min"
        ): cv.positive_time_period,
        cv.Optional(CONF_USE_LIGHT_SENSOR, default=False): cv.boolean,
        cv.Optional(CONF_LUX_LEARNING_WINDOW, default="24h"): cv.positive_time_period,
        cv.Optional(CONF_LUX_SAMPLE_INTERVAL, default="1min"): cv.positive_time_period,
        cv.Optional(CONF_USE_SUNRISE_PREDICTION, default=False): cv.boolean,
        cv.Optional(CONF_LATITUDE): cv.float_,
        cv.Optional(CONF_LONGITUDE): cv.float_,
        cv.Optional(CONF_SUN_ENTITY_ID, default="sun.sun"): cv.string,
        cv.Optional(CONF_ALPHA, default=0.5): cv.float_,
        cv.Optional(CONF_BASE_MULTIPLIER, default=1.0): cv.float_,
        cv.Optional(CONF_MAX_MULTIPLIER, default=4.0): cv.float_,
        cv.Optional(CONF_TIME_MULTIPLIER, default=1.5): cv.float_,
        cv.Optional(CONF_COMBINED_MULTIPLIER, default=3.0): cv.float_,
        cv.Optional(CONF_SUPPRESSION_WINDOW, default="30min"): cv.positive_time_period,
        cv.Optional(CONF_LUX_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_IDLE_RECALIB_INTERVAL): cv.positive_time_period,
        cv.Optional(CONF_ZONES, default={}): NullableSchema(
            {
                cv.Optional(CONF_INVERT, default=False): cv.boolean,
                cv.Optional(CONF_ENTRY_ZONE, default={}): ZONE_SCHEMA,
                cv.Optional(CONF_EXIT_ZONE, default={}): ZONE_SCHEMA,
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: Dict):
    roode = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(roode, config)

    sens = await cg.get_variable(config[CONF_SENSOR])
    cg.add(roode.set_tof_sensor(sens))

    cg.add(roode.set_orientation(config[CONF_ORIENTATION]))
    cg.add(roode.set_sampling_size(config[CONF_SAMPLING]))
    cg.add(roode.set_calibration_persistence(config[CONF_CALIBRATION_PERSISTENCE]))
    cg.add(roode.set_filter_mode(config[CONF_FILTER_MODE]))
    cg.add(roode.set_filter_window(config[CONF_FILTER_WINDOW]))
    cg.add(roode.set_log_fallback_events(config[CONF_LOG_FALLBACK]))
    cg.add(roode.set_force_single_core(config[CONF_FORCE_SINGLE_CORE]))
    cg.add(
        roode.set_auto_recalibrate_interval(
            config[CONF_AUTO_RECALIBRATE_INTERVAL].total_seconds
        )
    )
    cg.add(
        roode.set_recalibrate_on_temp_change(config[CONF_RECALIBRATE_ON_TEMP_CHANGE])
    )
    cg.add(
        roode.set_max_temp_delta_for_recalib(config[CONF_MAX_TEMP_DELTA_FOR_RECALIB])
    )
    cg.add(
        roode.set_recalibrate_cooldown(config[CONF_RECALIBRATE_COOLDOWN].total_seconds)
    )
    cg.add(roode.set_use_light_sensor(config[CONF_USE_LIGHT_SENSOR]))
    cg.add(
        roode.set_lux_learning_window(config[CONF_LUX_LEARNING_WINDOW].total_seconds)
    )
    cg.add(
        roode.set_lux_sample_interval(config[CONF_LUX_SAMPLE_INTERVAL].total_seconds)
    )
    cg.add(roode.set_use_sunrise_prediction(config[CONF_USE_SUNRISE_PREDICTION]))
    if CONF_LATITUDE in config:
        cg.add(roode.set_latitude(config[CONF_LATITUDE]))
    if CONF_LONGITUDE in config:
        cg.add(roode.set_longitude(config[CONF_LONGITUDE]))
    if CONF_SUN_ENTITY_ID in config:
        cg.add(roode.set_sun_entity_id(config[CONF_SUN_ENTITY_ID]))
    cg.add(roode.set_alpha(config[CONF_ALPHA]))
    cg.add(roode.set_base_multiplier(config[CONF_BASE_MULTIPLIER]))
    cg.add(roode.set_max_multiplier(config[CONF_MAX_MULTIPLIER]))
    cg.add(roode.set_time_multiplier(config[CONF_TIME_MULTIPLIER]))
    cg.add(roode.set_combined_multiplier(config[CONF_COMBINED_MULTIPLIER]))
    cg.add(roode.set_suppression_window(config[CONF_SUPPRESSION_WINDOW].total_seconds))
    if CONF_IDLE_RECALIB_INTERVAL in config:
        cg.add(
            roode.set_idle_recalib_interval(
                config[CONF_IDLE_RECALIB_INTERVAL].total_seconds
            )
        )
    if CONF_LUX_SENSOR in config:
        sens = await cg.get_variable(config[CONF_LUX_SENSOR])
        cg.add(roode.set_lux_sensor(sens))
    if CONF_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(roode.set_temperature_sensor(sens))
    cg.add(roode.set_invert_direction(config[CONF_ZONES][CONF_INVERT]))
    setup_zone(CONF_ENTRY_ZONE, config, roode)
    setup_zone(CONF_EXIT_ZONE, config, roode)


def setup_zone(name: str, config: Dict, roode: cg.Pvariable):
    zone_config = config[CONF_ZONES][name]
    zone_var = cg.MockObj(f"{roode}->{name}", "->")

    roi_var = cg.MockObj(f"{zone_var}->roi_override", "->")
    setup_roi(roi_var, zone_config.get(CONF_ROI, {}), config.get(CONF_ROI, {}))

    threshold_var = cg.MockObj(f"{zone_var}->threshold", "->")
    setup_thresholds(
        threshold_var,
        zone_config.get(CONF_DETECTION_THRESHOLDS, {}),
        config.get(CONF_DETECTION_THRESHOLDS, {}),
    )


def setup_roi(var: cg.MockObj, config: Union[Dict, str], fallback: Union[Dict, str]):
    config: Dict = (
        config
        if config != CONF_AUTO
        else {CONF_HEIGHT: CONF_AUTO, CONF_WIDTH: CONF_AUTO}
    )
    fallback: Dict = (
        fallback
        if fallback != CONF_AUTO
        else {CONF_HEIGHT: CONF_AUTO, CONF_WIDTH: CONF_AUTO}
    )
    height = config.get(CONF_HEIGHT, fallback.get(CONF_HEIGHT, 16))
    width = config.get(CONF_WIDTH, fallback.get(CONF_WIDTH, 6))
    if height != CONF_AUTO:
        cg.add(var.set_height(height))
    if width != CONF_AUTO:
        cg.add(var.set_width(width))
    if CONF_CENTER in config:
        cg.add(var.set_center(config[CONF_CENTER]))


def setup_thresholds(var: cg.MockObj, config: Dict, fallback: Dict):
    min = config.get(CONF_MIN, fallback.get(CONF_MIN, 0.0))
    max = config.get(CONF_MAX, fallback.get(CONF_MAX, 0.85))
    if isinstance(min, float):
        cg.add(var.set_min_percentage(int(min * 100)))
    else:
        cg.add(var.set_min(min))
    if isinstance(max, float):
        cg.add(var.set_max_percentage(int(max * 100)))
    else:
        cg.add(var.set_max(max))
