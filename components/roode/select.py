from typing import OrderedDict

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID

from . import Roode, roode_ns, CONF_ROODE_ID

DEPENDENCIES = ["roode"]
AUTO_LOAD = ["select"]

FilterModeSelect = roode_ns.class_("FilterModeSelect", select.Select)

CONF_FILTER_MODE = "filter_mode"

FILTER_MODE_OPTIONS = ["min", "median", "percentile10"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ROODE_ID): cv.use_id(Roode),
        cv.Optional(CONF_FILTER_MODE): select.SELECT_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(FilterModeSelect),
            }
        ),
    }
)


async def to_code(config: OrderedDict):
    hub = await cg.get_variable(config[CONF_ROODE_ID])
    if CONF_FILTER_MODE in config:
        conf = config[CONF_FILTER_MODE]
        sel = cg.new_Pvariable(conf[CONF_ID])
        await select.register_select(sel, conf, options=FILTER_MODE_OPTIONS)
        cg.add(sel.set_roode_hub(hub))
        cg.add(hub.set_filter_mode_select(sel))
