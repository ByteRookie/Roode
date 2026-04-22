from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ICON, CONF_MAX_VALUE

# NUMBER_MODE_BOX was added to esphome.const in a later ESPHome release.
# Fall back gracefully so the component still loads on older installs;
# numbers will just render as sliders instead of input boxes.
try:
    from esphome.const import NUMBER_MODE_BOX as _NUMBER_MODE_BOX
except ImportError:
    try:
        from esphome.components.number import NumberMode as _NM
        _NUMBER_MODE_BOX = _NM.NUMBER_MODE_BOX
    except Exception:
        _NUMBER_MODE_BOX = None

from ..persisted_number import PERSISTED_NUMBER_SCHEMA, new_persisted_number
from . import Roode, roode_ns, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

RoodeSettingNumber = roode_ns.class_("RoodeSettingNumber", number.Number)

CONF_PEOPLE_COUNTER = "people_counter"
CONF_FILTER_WINDOW = "filter_window"
CONF_SAMPLING = "sampling"
CONF_ENTRY_MAX_THRESHOLD = "entry_max_threshold"
CONF_ENTRY_MIN_THRESHOLD = "entry_min_threshold"
CONF_EXIT_MAX_THRESHOLD = "exit_max_threshold"
CONF_EXIT_MIN_THRESHOLD = "exit_min_threshold"
CONF_AUTO_CALIBRATION_INTERVAL = "auto_calibration_interval"
CONF_ENTRY_ROI_HEIGHT = "entry_roi_height"
CONF_ENTRY_ROI_WIDTH = "entry_roi_width"
CONF_EXIT_ROI_HEIGHT = "exit_roi_height"
CONF_EXIT_ROI_WIDTH = "exit_roi_width"

# (conf_key, setting_enum_name, min, max, step, icon, hub_setter)
SETTING_NUMBER_DEFS = [
    (CONF_FILTER_WINDOW,          "FILTER_WINDOW",          1,   10,  1,   "mdi:filter",                "set_filter_window_number"),
    (CONF_SAMPLING,               "SAMPLING",               1,   10,  1,   "mdi:format-list-numbered",  "set_sampling_number"),
    (CONF_ENTRY_MAX_THRESHOLD,    "ENTRY_MAX_PCT",         51,   95,  1,   "mdi:arrow-collapse-up",     "set_entry_max_threshold_number"),
    (CONF_ENTRY_MIN_THRESHOLD,    "ENTRY_MIN_PCT",          2,   49,  1,   "mdi:arrow-collapse-down",   "set_entry_min_threshold_number"),
    (CONF_EXIT_MAX_THRESHOLD,     "EXIT_MAX_PCT",          51,   95,  1,   "mdi:arrow-collapse-up",     "set_exit_max_threshold_number"),
    (CONF_EXIT_MIN_THRESHOLD,     "EXIT_MIN_PCT",           2,   49,  1,   "mdi:arrow-collapse-down",   "set_exit_min_threshold_number"),
    (CONF_AUTO_CALIBRATION_INTERVAL, "AUTO_CAL_INTERVAL_HOURS", 0, 24, 0.5, "mdi:clock-outline",       "set_auto_cal_interval_number"),
    (CONF_ENTRY_ROI_HEIGHT,       "ENTRY_ROI_HEIGHT",       4,   16,  1,   "mdi:arrow-expand-vertical", "set_entry_roi_height_number"),
    (CONF_ENTRY_ROI_WIDTH,        "ENTRY_ROI_WIDTH",        4,   16,  1,   "mdi:arrow-expand-horizontal","set_entry_roi_width_number"),
    (CONF_EXIT_ROI_HEIGHT,        "EXIT_ROI_HEIGHT",        4,   16,  1,   "mdi:arrow-expand-vertical", "set_exit_roi_height_number"),
    (CONF_EXIT_ROI_WIDTH,         "EXIT_ROI_WIDTH",         4,   16,  1,   "mdi:arrow-expand-horizontal","set_exit_roi_width_number"),
]

# Build schema entry for each setting number using the newer number_schema() API
def _setting_schema(icon: str):
    kwargs = {} if _NUMBER_MODE_BOX is None else {"mode": _NUMBER_MODE_BOX}
    return number.number_schema(RoodeSettingNumber, **kwargs).extend(
        {cv.Optional(CONF_ICON, default=icon): cv.icon}
    )

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_PEOPLE_COUNTER): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:counter"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(-128, 128),
            }
        ),
        **{
            cv.Optional(conf_key): _setting_schema(icon)
            for conf_key, _, _, _, _, icon, _ in SETTING_NUMBER_DEFS
        },
    }
)


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])

    if CONF_PEOPLE_COUNTER in config:
        counter = await new_persisted_number(
            config[CONF_PEOPLE_COUNTER],
            min_value=0,
            step=1,
            max_value=config[CONF_PEOPLE_COUNTER][CONF_MAX_VALUE],
        )
        cg.add(hub.set_people_counter(counter))

    for conf_key, setting_enum, min_val, max_val, step, icon, setter in SETTING_NUMBER_DEFS:
        if conf_key not in config:
            continue
        conf = config[conf_key]
        num = await number.new_number(
            conf,
            min_value=min_val,
            max_value=max_val,
            step=step,
        )
        cg.add(num.set_roode_hub(hub))
        cg.add(num.set_setting(
            cg.RawExpression(f"esphome::roode::RoodeSettingNumber::{setting_enum}")
        ))
        cg.add(getattr(hub, setter)(num))
