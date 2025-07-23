from typing import OrderedDict, List

import esphome.codegen as cg
from esphome.components import select
from esphome.components.template import select as tselect
from esphome.const import CONF_ID

async def new_persisted_select(config: OrderedDict, *, options: List[str]):
    var = cg.new_Pvariable(config[CONF_ID], tselect.TemplateSelect)
    await cg.register_component(var, config)
    await select.register_select(var, config, options=options)
    cg.add(var.set_optimistic(True))
    cg.add(var.set_initial_option(options[0]))
    cg.add(var.set_restore_value(True))
    return var
