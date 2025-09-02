from typing import Dict, Union
import json
from pathlib import Path
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.components.template import button as template_button
from esphome.const import (
    CONF_HEIGHT,
    CONF_ICON,
    CONF_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_INVERT,
    CONF_NAME,
    CONF_SENSOR,
    CONF_WIDTH,
)
from esphome.core import ID
from ..vl53l1x import distance_as_mm, NullableSchema, VL53L1X

DEPENDENCIES = ["vl53l1x"]
AUTO_LOAD = [
    "vl53l1x",
    "sensor",
    "binary_sensor",
    "text_sensor",
    "number",
    "button",
    "template",
]
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
CONF_AUTO_CALIBRATION = "auto_calibration"
CONF_INTERVAL = "interval"
CONF_PERSIST = "persist"
CONF_FILTER_MODE = "filter_mode"
CONF_FILTER_WINDOW = "filter_window"
CONF_LOG_FALLBACK = "log_fallback_events"
CONF_FORCE_SINGLE_CORE = "force_single_core"
CONF_INVALID_DISTANCE_LIMIT = "invalid_distance_limit"
CONF_RESTART_TIMEOUT = "restart_timeout"
CONF_CPU_OPTIMIZATION = "cpu_optimization"
CONF_ACTIVATE = "activate"
CONF_DEACTIVATE = "deactivate"
CONF_TRIAL_BUMP_CV = "trial_bump_cv"
CONF_SCAN_TIME_CAP_SECONDS = "scan_time_cap_seconds"
CONF_ROI_RESULT = "roi_result"
CONF_PORTAL_PASSWORD = "portal_password"
CONF_WEB_USERNAME = "web_username"
CONF_WEB_PASSWORD = "web_password"

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
        cv.Optional(CONF_AUTO_CALIBRATION, default={}): cv.Schema(
            {
                cv.Optional(CONF_INTERVAL, default=4 * 60 * 60): cv.uint32_t,
                cv.Optional(CONF_PERSIST, default=False): cv.boolean,
            }
        ),
        cv.Optional(CONF_FILTER_MODE, default="min"): cv.enum(
            FILTER_MODES, upper=False
        ),
        cv.Optional(CONF_FILTER_WINDOW, default=5): cv.All(cv.uint8_t, cv.Range(min=1)),
        cv.Optional(CONF_LOG_FALLBACK, default=False): cv.boolean,
        cv.Optional(CONF_FORCE_SINGLE_CORE, default=False): cv.boolean,
        cv.Optional(CONF_INVALID_DISTANCE_LIMIT, default=10): cv.All(
            cv.uint8_t, cv.Range(min=1)
        ),
        cv.Optional(
            CONF_RESTART_TIMEOUT, default="30s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_CPU_OPTIMIZATION, default={}): cv.Schema(
            {
                cv.Optional(CONF_ACTIVATE, default=0.90): cv.percentage,
                cv.Optional(CONF_DEACTIVATE, default=0.50): cv.percentage,
            }
        ),
        cv.Optional(CONF_TRIAL_BUMP_CV, default=0.20): cv.percentage,

        cv.Optional(CONF_SCAN_TIME_CAP_SECONDS, default=90.0): cv.All(
            cv.positive_float, cv.Range(min=60.0, max=180.0)
        ),
        cv.Optional(CONF_PORTAL_PASSWORD): cv.string,
        cv.Optional(CONF_WEB_USERNAME): cv.string,
        cv.Optional(CONF_WEB_PASSWORD): cv.string,
        cv.Optional(CONF_ROI_RESULT): cv.file_,
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
    cg.add_library("bblanchon", "6.21.3", "ArduinoJson")

    roode = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(roode, config)

    button_id = ID(f"{config[CONF_ID].id}_start_passive_scan")
    button_id.type = template_button.TemplateButton
    btn_conf = {
        CONF_ID: button_id,
        CONF_NAME: "Start Passive Scan",
        CONF_ICON: "mdi:radar",
        CONF_DISABLED_BY_DEFAULT: False,
    }
    start_scan_button = await button.new_button(btn_conf)
    cg.add(
        start_scan_button.set_entity_category(
            cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC
        )
    )
    cg.add(
        start_scan_button.add_on_press_callback(
            cg.RawExpression(f"std::bind(&{Roode}::start_passive_scan, {roode})")
        )
    )

    button_id = ID(f"{config[CONF_ID].id}_complete_calibration")
    button_id.type = template_button.TemplateButton
    btn_conf = {
        CONF_ID: button_id,
        CONF_NAME: "Complete Calibration",
        CONF_ICON: "mdi:auto-fix",
        CONF_DISABLED_BY_DEFAULT: False,
    }
    complete_button = await button.new_button(btn_conf)
    cg.add(
        complete_button.set_entity_category(
            cg.EntityCategory.ENTITY_CATEGORY_CONFIG
        )
    )
    cg.add(
        complete_button.add_on_press_callback(
            cg.RawExpression(f"std::bind(&{Roode}::complete_calibration, {roode})")
        )
    )

    button_id = ID(f"{config[CONF_ID].id}_recalibrate")
    button_id.type = template_button.TemplateButton
    btn_conf = {
        CONF_ID: button_id,
        CONF_NAME: "Recalibrate",
        CONF_ICON: "mdi:restart",
        CONF_DISABLED_BY_DEFAULT: False,
    }
    recalibration_button = await button.new_button(btn_conf)
    cg.add(
        recalibration_button.set_entity_category(
            cg.EntityCategory.ENTITY_CATEGORY_CONFIG
        )
    )
    cg.add(
        recalibration_button.add_on_press_callback(
            cg.RawExpression(f"std::bind(&{Roode}::recalibration, {roode})")
        )
    )

    sens = await cg.get_variable(config[CONF_SENSOR])
    cg.add(roode.set_tof_sensor(sens))

    cg.add(roode.set_orientation(config[CONF_ORIENTATION]))
    cg.add(roode.set_sampling_size(config[CONF_SAMPLING]))
    if CONF_PORTAL_PASSWORD in config:
        cg.add(roode.set_portal_password(config[CONF_PORTAL_PASSWORD]))
    if CONF_WEB_USERNAME in config:
        cg.add(roode.set_web_username(config[CONF_WEB_USERNAME]))
    if CONF_WEB_PASSWORD in config:
        cg.add(roode.set_web_password(config[CONF_WEB_PASSWORD]))
    auto_conf = config.get(CONF_AUTO_CALIBRATION, {})
    cg.add(roode.set_calibration_persistence(auto_conf.get(CONF_PERSIST, False)))
    cg.add(
        roode.set_auto_calibration_interval_sec(
            auto_conf.get(CONF_INTERVAL, 4 * 60 * 60)
        )
    )
    cg.add(roode.set_filter_mode(config[CONF_FILTER_MODE]))
    cg.add(roode.set_filter_window(config[CONF_FILTER_WINDOW]))
    cg.add(roode.set_log_fallback_events(config[CONF_LOG_FALLBACK]))
    cg.add(roode.set_force_single_core(config[CONF_FORCE_SINGLE_CORE]))
    cg.add(roode.set_invalid_distance_limit(config[CONF_INVALID_DISTANCE_LIMIT]))
    cg.add(roode.set_restart_timeout(config[CONF_RESTART_TIMEOUT]))
    cg.add(roode.set_trial_bump_cv(config[CONF_TRIAL_BUMP_CV] * 100.0))
    cg.add(roode.set_scan_time_cap_seconds(config[CONF_SCAN_TIME_CAP_SECONDS]))
    cpu_conf = config.get(CONF_CPU_OPTIMIZATION, {})
    cg.add(
        roode.set_cpu_optimization_thresholds(
            cpu_conf.get(CONF_ACTIVATE, 0.90) * 100.0,
            cpu_conf.get(CONF_DEACTIVATE, 0.50) * 100.0,
        )
    )
    roi_file = config.get(CONF_ROI_RESULT)
    if roi_file:
        data = json.load(Path(roi_file).open())
        entry_z = config[CONF_ZONES][CONF_ENTRY_ZONE]
        exit_z = config[CONF_ZONES][CONF_EXIT_ZONE]
        config[CONF_ROI].update(data.get("roi", {}))
        entry_z[CONF_ROI][CONF_CENTER] = data["zones"]["entry"]["center"]
        exit_z[CONF_ROI][CONF_CENTER] = data["zones"]["exit"]["center"]
        config[CONF_DETECTION_THRESHOLDS].update(data.get("thresholds", {}))

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
