import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_ACCURACY_DECIMALS,
    DEVICE_CLASS_WATER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    CONF_UNIT_OF_MEASUREMENT,
    UNIT_LITRE,
)

from . import muino_3phase_i2c_ns, Muino3PhaseI2CSensor

CONF_TOTAL = "total"
CONF_ML = "ml"
CONF_MAIN_CONSUMPTION = "main_consumption"
CONF_SECONDARY_CONSUMPTION = "secondary_consumption"
CONF_TERTIARY_CONSUMPTION = "tertiary_consumption"
CONF_SENSOR_A = "sensor_a"
CONF_SENSOR_B = "sensor_b"
CONF_SENSOR_C = "sensor_c"
CONF_PHASE = "phase"
CONF_TIME_SINCE_LAST_FLOW = "time_since_last_flow"
CONF_LAST_CONSUMPTION = "last_consumption"

UNIT_LITER_PER_MINUTE = "L/min"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Muino3PhaseI2CSensor),
    cv.Optional(CONF_TOTAL): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
    }),
    cv.Optional(CONF_ML): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default='mL'): cv.string,
    }),
    cv.Optional(CONF_MAIN_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
        cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.positive_int,
    }),
    cv.Optional(CONF_SECONDARY_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
        cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.positive_int,
    }),
    cv.Optional(CONF_TERTIARY_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
        cv.Optional(CONF_ACCURACY_DECIMALS, default=1): cv.positive_int,
    }),
    cv.Optional(CONF_TIME_SINCE_LAST_FLOW): sensor.sensor_schema(
        unit_of_measurement="s",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_LAST_CONSUMPTION): sensor.sensor_schema(
        unit_of_measurement=UNIT_LITRE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SENSOR_A): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_SENSOR_B): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_SENSOR_C): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_PHASE): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_TOTAL in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL])
        cg.add(var.set_total_sensor(sens))

    if CONF_ML in config:
        sens = await sensor.new_sensor(config[CONF_ML])
        cg.add(var.set_ml_sensor(sens))

    if CONF_MAIN_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_MAIN_CONSUMPTION])
        cg.add(var.set_main_consumption_sensor(sens))

    if CONF_SECONDARY_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_SECONDARY_CONSUMPTION])
        cg.add(var.set_secondary_consumption_sensor(sens))

    if CONF_TERTIARY_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_TERTIARY_CONSUMPTION])
        cg.add(var.set_tertiary_consumption_sensor(sens))

    if CONF_TIME_SINCE_LAST_FLOW in config:
        sens = await sensor.new_sensor(config[CONF_TIME_SINCE_LAST_FLOW])
        cg.add(var.set_time_since_last_flow(sens))

    if CONF_LAST_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_LAST_CONSUMPTION])
        cg.add(var.set_last_consumption(sens))

    if CONF_SENSOR_A in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_A])
        cg.add(var.set_sensor_a(sens))

    if CONF_SENSOR_B in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_B])
        cg.add(var.set_sensor_b(sens))

    if CONF_SENSOR_C in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_C])
        cg.add(var.set_sensor_c(sens))

    if CONF_PHASE in config:
        sens = await sensor.new_sensor(config[CONF_PHASE])
        cg.add(var.set_sensor_phase(sens))
