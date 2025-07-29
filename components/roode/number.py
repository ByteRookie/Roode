from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_MAX_VALUE
from esphome.cpp_generator import MockObj

from ..persisted_number import PERSISTED_NUMBER_SCHEMA, new_persisted_number
from . import Roode, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["number", "persisted_number"]

CONF_PEOPLE_COUNTER = "people_counter"
CONF_INVALID_DISTANCE_LIMIT = "invalid_distance_limit"
CONF_RESTART_TIMEOUT = "restart_timeout"
CONF_SAMPLING = "sampling"
CONF_FILTER_WINDOW = "filter_window"
CONF_DETECTION_MIN = "detection_threshold_min"
CONF_DETECTION_MAX = "detection_threshold_max"
CONF_ENTRY_THRESHOLD_MIN = "entry_threshold_min"
CONF_ENTRY_THRESHOLD_MAX = "entry_threshold_max"
CONF_EXIT_THRESHOLD_MIN = "exit_threshold_min"
CONF_EXIT_THRESHOLD_MAX = "exit_threshold_max"
CONF_ENTRY_ROI_HEIGHT = "entry_roi_height"
CONF_EXIT_ROI_HEIGHT = "exit_roi_height"
CONF_ROI_WIDTH = "roi_width"
CONF_ROI_HEIGHT = "roi_height"
CONF_ENTRY_ROI_CENTER = "entry_roi_center"
CONF_EXIT_ROI_CENTER = "exit_roi_center"
CONF_CALIBRATION_OFFSET = "calibration_offset"
CONF_CALIBRATION_CROSSTALK = "calibration_crosstalk"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_PEOPLE_COUNTER): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:counter"): cv.icon,  # new default
                cv.Optional(CONF_MAX_VALUE, 10): cv.int_range(-128, 128),
            }
        ),
        cv.Optional(CONF_INVALID_DISTANCE_LIMIT): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:ruler"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 100): cv.int_range(1, 100),
            }
        ),
        cv.Optional(CONF_RESTART_TIMEOUT): PERSISTED_NUMBER_SCHEMA.extend(
            {
                cv.Optional(CONF_ICON, default="mdi:timer"): cv.icon,
                cv.Optional(CONF_MAX_VALUE, 120): cv.int_range(1, 120),
            }
        ),
        cv.Optional(CONF_SAMPLING): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 6): cv.int_range(1, 6)}
        ),
        cv.Optional(CONF_FILTER_WINDOW): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 9): cv.int_range(3, 9)}
        ),
        cv.Optional(CONF_DETECTION_MIN): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_DETECTION_MAX): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_ENTRY_THRESHOLD_MIN): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_ENTRY_THRESHOLD_MAX): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_EXIT_THRESHOLD_MIN): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_EXIT_THRESHOLD_MAX): PERSISTED_NUMBER_SCHEMA,
        cv.Optional(CONF_ENTRY_ROI_HEIGHT): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 16): cv.int_range(4, 16)}
        ),
        cv.Optional(CONF_EXIT_ROI_HEIGHT): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 16): cv.int_range(4, 16)}
        ),
        cv.Optional(CONF_ROI_WIDTH): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 16): cv.int_range(4, 16)}
        ),
        cv.Optional(CONF_ROI_HEIGHT): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 16): cv.int_range(4, 16)}
        ),
        cv.Optional(CONF_ENTRY_ROI_CENTER): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 255): cv.int_range(0, 255)}
        ),
        cv.Optional(CONF_EXIT_ROI_CENTER): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 255): cv.int_range(0, 255)}
        ),
        cv.Optional(CONF_CALIBRATION_OFFSET): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 50): cv.int_range(-50, 50)}
        ),
        cv.Optional(CONF_CALIBRATION_CROSSTALK): PERSISTED_NUMBER_SCHEMA.extend(
            {cv.Optional(CONF_MAX_VALUE, 100000): cv.int_range(0, 100000)}
        ),
    }
)


async def setup_people_counter(config: OrderedDict, hub: MockObj):
    counter = await new_persisted_number(
        config, min_value=0, step=1, max_value=config[CONF_MAX_VALUE], default_value=0
    )
    cg.add(hub.set_people_counter(counter))


async def setup_invalid_distance_limit(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=1, max_value=config[CONF_MAX_VALUE], step=1, default_value=10
    )
    cg.add(hub.set_invalid_distance_limit_number(num))


async def setup_restart_timeout(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=1, max_value=config[CONF_MAX_VALUE], step=1, default_value=30
    )
    cg.add(hub.set_restart_timeout_number(num))


async def setup_sampling(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=1, max_value=config[CONF_MAX_VALUE], step=1, default_value=2
    )
    cg.add(hub.set_sampling_number(num))


async def setup_filter_window(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=3, max_value=config[CONF_MAX_VALUE], step=1, default_value=5
    )
    cg.add(hub.set_filter_window_number(num))


async def setup_detection_min(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=0
    )
    cg.add(hub.set_detection_min_number(num))


async def setup_detection_max(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=85
    )
    cg.add(hub.set_detection_max_number(num))


async def setup_entry_threshold_min(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=0
    )
    cg.add(hub.set_entry_threshold_min_number(num))


async def setup_entry_threshold_max(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=85
    )
    cg.add(hub.set_entry_threshold_max_number(num))


async def setup_exit_threshold_min(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=0
    )
    cg.add(hub.set_exit_threshold_min_number(num))


async def setup_exit_threshold_max(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100, step=1, default_value=85
    )
    cg.add(hub.set_exit_threshold_max_number(num))


async def setup_entry_roi_height(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=4, max_value=16, step=1, default_value=16
    )
    cg.add(hub.set_entry_roi_height_number(num))


async def setup_exit_roi_height(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=4, max_value=16, step=1, default_value=16
    )
    cg.add(hub.set_exit_roi_height_number(num))


async def setup_roi_width(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=4, max_value=16, step=1, default_value=6
    )
    cg.add(hub.set_roi_width_number(num))


async def setup_roi_height(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=4, max_value=16, step=1, default_value=16
    )
    cg.add(hub.set_roi_height_number(num))


async def setup_entry_roi_center(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=255, step=1, default_value=0
    )
    cg.add(hub.set_entry_roi_center_number(num))


async def setup_exit_roi_center(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=255, step=1, default_value=0
    )
    cg.add(hub.set_exit_roi_center_number(num))


async def setup_calibration_offset(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=-50, max_value=50, step=1, default_value=0
    )
    cg.add(hub.set_calibration_offset_number(num))


async def setup_calibration_crosstalk(config: OrderedDict, hub: MockObj):
    num = await new_persisted_number(
        config, min_value=0, max_value=100000, step=1000, default_value=0
    )
    cg.add(hub.set_calibration_crosstalk_number(num))


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_PEOPLE_COUNTER in config:
        await setup_people_counter(config[CONF_PEOPLE_COUNTER], hub)
    if CONF_INVALID_DISTANCE_LIMIT in config:
        await setup_invalid_distance_limit(config[CONF_INVALID_DISTANCE_LIMIT], hub)
    if CONF_RESTART_TIMEOUT in config:
        await setup_restart_timeout(config[CONF_RESTART_TIMEOUT], hub)
    if CONF_SAMPLING in config:
        await setup_sampling(config[CONF_SAMPLING], hub)
    if CONF_FILTER_WINDOW in config:
        await setup_filter_window(config[CONF_FILTER_WINDOW], hub)
    if CONF_DETECTION_MIN in config:
        await setup_detection_min(config[CONF_DETECTION_MIN], hub)
    if CONF_DETECTION_MAX in config:
        await setup_detection_max(config[CONF_DETECTION_MAX], hub)
    if CONF_ENTRY_THRESHOLD_MIN in config:
        await setup_entry_threshold_min(config[CONF_ENTRY_THRESHOLD_MIN], hub)
    if CONF_ENTRY_THRESHOLD_MAX in config:
        await setup_entry_threshold_max(config[CONF_ENTRY_THRESHOLD_MAX], hub)
    if CONF_EXIT_THRESHOLD_MIN in config:
        await setup_exit_threshold_min(config[CONF_EXIT_THRESHOLD_MIN], hub)
    if CONF_EXIT_THRESHOLD_MAX in config:
        await setup_exit_threshold_max(config[CONF_EXIT_THRESHOLD_MAX], hub)
    if CONF_ENTRY_ROI_HEIGHT in config:
        await setup_entry_roi_height(config[CONF_ENTRY_ROI_HEIGHT], hub)
    if CONF_EXIT_ROI_HEIGHT in config:
        await setup_exit_roi_height(config[CONF_EXIT_ROI_HEIGHT], hub)
    if CONF_ROI_WIDTH in config:
        await setup_roi_width(config[CONF_ROI_WIDTH], hub)
    if CONF_ROI_HEIGHT in config:
        await setup_roi_height(config[CONF_ROI_HEIGHT], hub)
    if CONF_ENTRY_ROI_CENTER in config:
        await setup_entry_roi_center(config[CONF_ENTRY_ROI_CENTER], hub)
    if CONF_EXIT_ROI_CENTER in config:
        await setup_exit_roi_center(config[CONF_EXIT_ROI_CENTER], hub)
    if CONF_CALIBRATION_OFFSET in config:
        await setup_calibration_offset(config[CONF_CALIBRATION_OFFSET], hub)
    if CONF_CALIBRATION_CROSSTALK in config:
        await setup_calibration_crosstalk(config[CONF_CALIBRATION_CROSSTALK], hub)
