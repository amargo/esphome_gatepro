import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, cover
from esphome.const import CONF_ID, ICON_EMPTY, UNIT_EMPTY

DEPENDENCIES = ["uart", "cover"]

gatepro_ns = cg.esphome_ns.namespace("gatepro")
GatePro = gatepro_ns.class_(
    "GatePro", cover.Cover, cg.PollingComponent, uart.UARTDevice
)

CONF_OPERATIONAL_SPEED = "operational_speed"
CONF_SOURCE = "source"
CONF_OPEN_DURATION_WARNING = "open_duration_warning"

cover.COVER_OPERATIONS.update({
    "READ_STATUS": cover.CoverOperation.COVER_OPERATION_READ_STATUS,
})
validate_cover_operation = cv.enum(cover.COVER_OPERATIONS, upper=True)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(GatePro),
        cv.Optional(CONF_SOURCE, default="P00287D7"): cv.string,
        cv.Optional(CONF_OPEN_DURATION_WARNING, default="5min"): cv.positive_time_period_milliseconds,
    }).extend(cv.COMPONENT_SCHEMA).extend(cv.polling_component_schema("60s")).extend(uart.UART_DEVICE_SCHEMA)



async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    #var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_SOURCE in config:
        cg.add(var.set_source(config[CONF_SOURCE]))
        
    if CONF_OPEN_DURATION_WARNING in config:
        cg.add(var.set_open_duration_warning_threshold(config[CONF_OPEN_DURATION_WARNING]))