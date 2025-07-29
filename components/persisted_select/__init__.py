from typing import OrderedDict, List

import esphome.codegen as cg
from esphome.components import select as esphome_select
from esphome.components.template import select as tselect
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_RESTORE_VALUE,
    CONF_DISABLED_BY_DEFAULT,
)

async def new_persisted_select(config: OrderedDict, *, options: List[str]):
    config = {
        **config,
        CONF_NAME: config.get(CONF_NAME, str(config[CONF_ID])),
        CONF_RESTORE_VALUE: True,
        CONF_DISABLED_BY_DEFAULT: config.get(CONF_DISABLED_BY_DEFAULT, False),
    }
    var = cg.new_Pvariable(config[CONF_ID], tselect.TemplateSelect)
    from esphome.core import CORE
    CORE.component_ids.add(config[CONF_ID].id)
    await cg.register_component(var, config)
    await esphome_select.register_select(var, config, options=options)
    cg.add(var.set_optimistic(True))
    cg.add(var.set_initial_option(options[0]))
    cg.add(var.set_restore_value(True))
    return var
