import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

gatepro_ns = cg.esphome_ns.namespace("gatepro")
GateProParamReader = gatepro_ns.class_("GateProParamReader", cg.Component, uart.UARTDevice)

# Configuration keys
CONF_AUTO_CLOSE = "auto_close"
CONF_OP_SPEED = "op_speed"
CONF_DECEL_START = "decel_start"
CONF_DECEL_SPEED = "decel_speed"
CONF_TORQUE_SENSE = "torque_sense"
CONF_PEDESTRIAN = "pedestrian"
CONF_LAMP_FLASH = "lamp_flash"
CONF_TORQUE_REACTION = "torque_reaction"
CONF_FULL_OPEN_BUTTON = "full_open_button"
CONF_PED_BUTTON = "ped_button"
CONF_INFRA_BEAM = "infra_beam"
CONF_OPERATION_MODE = "operation_mode"
CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GateProParamReader),
            cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_AUTO_CLOSE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_OP_SPEED): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DECEL_START): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DECEL_SPEED): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_TORQUE_SENSE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PEDESTRIAN): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAMP_FLASH): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_TORQUE_REACTION): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_FULL_OPEN_BUTTON): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PED_BUTTON): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_INFRA_BEAM): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_OPERATION_MODE): text_sensor.text_sensor_schema(),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_UART_ID in config:
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart(uart_component))
    else:
        await uart.register_uart_device(var, config)

    if CONF_AUTO_CLOSE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_AUTO_CLOSE])
        cg.add(var.set_auto_close(sens))

    if CONF_OP_SPEED in config:
        sens = await text_sensor.new_text_sensor(config[CONF_OP_SPEED])
        cg.add(var.set_op_speed(sens))

    if CONF_DECEL_START in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DECEL_START])
        cg.add(var.set_decel_start(sens))

    if CONF_DECEL_SPEED in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DECEL_SPEED])
        cg.add(var.set_decel_speed(sens))

    if CONF_TORQUE_SENSE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_TORQUE_SENSE])
        cg.add(var.set_torque_sense(sens))

    if CONF_PEDESTRIAN in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PEDESTRIAN])
        cg.add(var.set_pedestrian(sens))

    if CONF_LAMP_FLASH in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAMP_FLASH])
        cg.add(var.set_lamp_flash(sens))

    if CONF_TORQUE_REACTION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_TORQUE_REACTION])
        cg.add(var.set_torque_reaction(sens))

    if CONF_FULL_OPEN_BUTTON in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FULL_OPEN_BUTTON])
        cg.add(var.set_full_open_button(sens))

    if CONF_PED_BUTTON in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PED_BUTTON])
        cg.add(var.set_ped_button(sens))

    if CONF_INFRA_BEAM in config:
        sens = await text_sensor.new_text_sensor(config[CONF_INFRA_BEAM])
        cg.add(var.set_infra_beam(sens))

    if CONF_OPERATION_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_OPERATION_MODE])
        cg.add(var.set_operation_mode(sens))