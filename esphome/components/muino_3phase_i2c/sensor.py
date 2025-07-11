import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_WATER,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    CONF_UNIT_OF_MEASUREMENT,
    UNIT_LITRE,
)

from . import Muino3PhaseI2CSensor

CONF_INDEX = "index"
CONF_MAIN_CONSUMPTION = "main_consumption"
CONF_SECONDARY_CONSUMPTION = "secondary_consumption"
CONF_TERTIARY_CONSUMPTION = "tertiary_consumption"
CONF_SENSOR_A = "sensor_a"
CONF_SENSOR_B = "sensor_b"
CONF_SENSOR_C = "sensor_c"
CONF_PHASE = "phase"
CONF_TIME_SINCE_LAST_FLOW = "time_since_last_flow"
CONF_PREVIOUS_CONSUMPTION = "previous_consumption"
CONF_CURRENT_CONSUMPTION = "current_consumption"
CONF_MEASUREMENTS_CONSISTENCY= "measurements_consistency"
CONF_FLOW_RATE = "flow_rate"
CONF_FLOW_RATE_RESET_TIME = "reset_time"

UNIT_LITER_PER_MINUTE = "L/min"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Muino3PhaseI2CSensor),
    cv.Optional(CONF_INDEX): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
    }),
    cv.Optional(CONF_MAIN_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
    }),
    cv.Optional(CONF_SECONDARY_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
    }),
    cv.Optional(CONF_TERTIARY_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        accuracy_decimals=1,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITRE): cv.string,
    }),
    cv.Optional(CONF_TIME_SINCE_LAST_FLOW): sensor.sensor_schema(
        unit_of_measurement="s",
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_PREVIOUS_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_LITRE,
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_CURRENT_CONSUMPTION): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_LITRE,
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_SENSOR_A): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_SENSOR_B): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_SENSOR_C): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_PHASE): sensor.sensor_schema(),
    cv.Optional(CONF_MEASUREMENTS_CONSISTENCY): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    cv.Optional(CONF_FLOW_RATE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_WATER,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=2,
    ).extend({
        cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_LITER_PER_MINUTE): cv.string,
        cv.Optional(CONF_FLOW_RATE_RESET_TIME, default=15): cv.positive_int,
    }),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_INDEX in config:
        sens = await sensor.new_sensor(config[CONF_INDEX])
        cg.add(var.set_index_sensor(sens))

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
        cg.add(var.set_time_since_last_flow_sensor(sens))

    if CONF_PREVIOUS_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_PREVIOUS_CONSUMPTION])
        cg.add(var.set_previous_consumption_sensor(sens))

    if CONF_CURRENT_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT_CONSUMPTION])
        cg.add(var.set_current_consumption_sensor(sens))

    if CONF_SENSOR_A in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_A])
        cg.add(var.set_a_sensor(sens))

    if CONF_SENSOR_B in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_B])
        cg.add(var.set_b_sensor(sens))

    if CONF_SENSOR_C in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SENSOR_C])
        cg.add(var.set_c_sensor(sens))

    if CONF_PHASE in config:
        sens = await sensor.new_sensor(config[CONF_PHASE])
        cg.add(var.set_phase_sensor(sens))

    if CONF_MEASUREMENTS_CONSISTENCY in config:
        bin_sens = await binary_sensor.new_binary_sensor(config[CONF_MEASUREMENTS_CONSISTENCY])
        cg.add(var.set_measurements_consistency_sensor(bin_sens))

    if CONF_FLOW_RATE in config:
        sens = await sensor.new_sensor(config[CONF_FLOW_RATE])
        cg.add(var.set_flow_rate_sensor(sens))

        if CONF_FLOW_RATE_RESET_TIME in config[CONF_FLOW_RATE]:
            cg.add(var.set_flow_rate_reset_time(config[CONF_FLOW_RATE][CONF_FLOW_RATE_RESET_TIME]))
