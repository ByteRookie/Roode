from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import Roode, roode_ns, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select"]

FilterModeSelect = roode_ns.class_("FilterModeSelect", select.Select)
RangingModeSelect = roode_ns.class_("RangingModeSelect", select.Select)
OrientationSelect = roode_ns.class_("OrientationSelect", select.Select)

CONF_FILTER_MODE = "filter_mode"
CONF_RANGING_MODE = "ranging_mode"
CONF_ORIENTATION = "orientation"

FILTER_MODE_OPTIONS = ["min", "median", "percentile10"]
RANGING_MODE_OPTIONS = ["auto", "short", "medium", "long", "longer", "longest"]
ORIENTATION_OPTIONS = ["Above", "Side"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_FILTER_MODE): select.select_schema(FilterModeSelect),
        cv.Optional(CONF_RANGING_MODE): select.select_schema(RangingModeSelect),
        cv.Optional(CONF_ORIENTATION): select.select_schema(OrientationSelect),
    }
)


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])

    if CONF_FILTER_MODE in config:
        conf = config[CONF_FILTER_MODE]
        sel = await select.new_select(conf, options=FILTER_MODE_OPTIONS)
        cg.add(sel.set_roode_hub(hub))
        cg.add(hub.set_filter_mode_select(sel))

    if CONF_RANGING_MODE in config:
        conf = config[CONF_RANGING_MODE]
        sel = await select.new_select(conf, options=RANGING_MODE_OPTIONS)
        cg.add(sel.set_roode_hub(hub))
        cg.add(hub.set_ranging_mode_select(sel))

    if CONF_ORIENTATION in config:
        conf = config[CONF_ORIENTATION]
        sel = await select.new_select(conf, options=ORIENTATION_OPTIONS)
        cg.add(sel.set_roode_hub(hub))
        cg.add(hub.set_orientation_select(sel))
