import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_WATER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    CONF_UNIT_OF_MEASUREMENT,
    UNIT_CUBIC_METER,
)

from . import muino_3phase_ns, Muino3PhaseSensor

# Remove this line as it's causing the dependency error
# DEPENDENCIES = ['muino_3phase']

CONF_FLOW = "flow"
CONF_TOTAL = "total"
CONF_SENSOR_A = "sensor_a"
CONF_SENSOR_B = "sensor_b"
CONF_SENSOR_C = "sensor_c"

UNIT_LITER_PER_MINUTE = "L/min"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Muino3PhaseSensor),
    cv.Optional(CONF_FLOW): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_MEASUREMENT,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITER_PER_MINUTE): cv.string,
    }),
    cv.Optional(CONF_TOTAL): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CUBIC_METER): cv.string,
    }),
    cv.Optional(CONF_SENSOR_A): sensor.sensor_schema(),
    cv.Optional(CONF_SENSOR_B): sensor.sensor_schema(),
    cv.Optional(CONF_SENSOR_C): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_FLOW in config:
        sens = await sensor.new_sensor(config[CONF_FLOW])
        cg.add(var.set_flow_sensor(sens))

    if CONF_TOTAL in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL])
        cg.add(var.set_total_sensor(sens))

    if CONF_SENSOR_A in config:
        sens = await sensor.new_sensor(config[CONF_SENSOR_A])
        cg.add(var.set_sensor_a(sens))

    if CONF_SENSOR_B in config:
        sens = await sensor.new_sensor(config[CONF_SENSOR_B])
        cg.add(var.set_sensor_b(sens))

    if CONF_SENSOR_C in config:
        sens = await sensor.new_sensor(config[CONF_SENSOR_C])
        cg.add(var.set_sensor_c(sens))
