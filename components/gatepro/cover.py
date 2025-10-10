import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, cover, button, number, text_sensor, switch
from esphome.const import CONF_ID, ICON_EMPTY, UNIT_EMPTY

DEPENDENCIES = ["uart", "cover", "button"]

gatepro_ns = cg.esphome_ns.namespace("gatepro")
GatePro = gatepro_ns.class_(
    "GatePro", cover.Cover, cg.PollingComponent, uart.UARTDevice
)

CONF_OPERATIONAL_SPEED = "operational_speed"
CONF_SOURCE = "source"

# Basic operation button configurations
CONF_OPEN_BTN = "open"                      # Manual open button
CONF_CLOSE_BTN = "close"                    # Manual close button
CONF_STOP_BTN = "stop"                      # Manual stop button

# Advanced button configurations
CONF_LEARN = "set_learn"                    # Auto learn function
CONF_PARAMS_OD = "get_params"               # Read parameters on demand
CONF_REMOTE_LEARN = "remote_learn"          # Remote control learn

# Number slider configurations (parameter groups)
CONF_SPEED_SLIDER = "set_speed"             # group 3 - Operation speed
CONF_DECEL_DIST_SLIDER = "set_decel_dist"   # group 4 - Deceleration distance
CONF_DECEL_SPEED_SLIDER = "set_decel_speed" # group 5 - Deceleration speed
CONF_MAX_AMP = "set_max_amp"                # group 6 - Maximum current/torque
CONF_AUTO_CLOSE = "set_auto_close"          # group 1 - Auto close timer
CONF_SMALL_GATE_TIMER = "set_small_gate_timer" # group 8 - Small gate timer
CONF_FORCE_DETECTION = "set_force_detection" # group 10 - Force detection reaction

# Text sensor configurations
CONF_DEVINFO = "txt_devinfo"                # Device information
CONF_LEARN_STATUS = "txt_learn_status"      # Learn status information

# Switch configurations (parameter groups)
CONF_PERMALOCK = "sw_permalock"             # group 15 - Permanent lock
CONF_INFRA1 = "sw_infra1"                   # group 13 - Infrared sensor 1
CONF_INFRA2 = "sw_infra2"                   # group 14 - Infrared sensor 2

cover.COVER_OPERATIONS.update({
    "READ_STATUS": cover.CoverOperation.COVER_OPERATION_READ_STATUS,
})
validate_cover_operation = cv.enum(cover.COVER_OPERATIONS, upper=True)

CONFIG_SCHEMA = cover.cover_schema(GatePro).extend(
    {
        cv.GenerateID(): cv.declare_id(GatePro),
        cv.Optional(CONF_SOURCE, default="P00287D7"): cv.string,
        
        # Basic operation button components
        cv.Optional(CONF_OPEN_BTN): cv.use_id(button.Button),                # Manual open button
        cv.Optional(CONF_CLOSE_BTN): cv.use_id(button.Button),               # Manual close button
        cv.Optional(CONF_STOP_BTN): cv.use_id(button.Button),                # Manual stop button
        
        # Advanced button components
        cv.Optional(CONF_LEARN): cv.use_id(button.Button),                    # Auto learn function
        cv.Optional(CONF_PARAMS_OD): cv.use_id(button.Button),               # Read parameters on demand
        cv.Optional(CONF_REMOTE_LEARN): cv.use_id(button.Button),            # Remote control learn
        
        # Number slider components (parameter groups)
        cv.Optional(CONF_SPEED_SLIDER): cv.use_id(number.Number),            # group 3 - Operation speed
        cv.Optional(CONF_DECEL_DIST_SLIDER): cv.use_id(number.Number),       # group 4 - Deceleration distance
        cv.Optional(CONF_DECEL_SPEED_SLIDER): cv.use_id(number.Number),      # group 5 - Deceleration speed
        cv.Optional(CONF_MAX_AMP): cv.use_id(number.Number),                 # group 6 - Maximum current/torque
        cv.Optional(CONF_AUTO_CLOSE): cv.use_id(number.Number),              # group 1 - Auto close timer
        cv.Optional(CONF_SMALL_GATE_TIMER): cv.use_id(number.Number),        # group 8 - Small gate timer
        cv.Optional(CONF_FORCE_DETECTION): cv.use_id(number.Number),         # group 10 - Force detection reaction
        
        # Text sensor components
        cv.Optional(CONF_DEVINFO): cv.use_id(text_sensor.TextSensor),        # Device information
        cv.Optional(CONF_LEARN_STATUS): cv.use_id(text_sensor.TextSensor),   # Learn status information
        
        # Switch components (parameter groups)
        cv.Optional(CONF_PERMALOCK): cv.use_id(switch.Switch),               # group 15 - Permanent lock
        cv.Optional(CONF_INFRA1): cv.use_id(switch.Switch),                  # group 13 - Infrared sensor 1
        cv.Optional(CONF_INFRA2): cv.use_id(switch.Switch),                  # group 14 - Infrared sensor 2
    }).extend(cv.COMPONENT_SCHEMA).extend(cv.polling_component_schema("60s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_SOURCE in config:
        cg.add(var.set_source(config[CONF_SOURCE]))

    # Basic operation button components
    if CONF_OPEN_BTN in config:                                             # Manual open button
        btn = await cg.get_variable(config[CONF_OPEN_BTN])
        cg.add(var.set_btn_open(btn))
    if CONF_CLOSE_BTN in config:                                            # Manual close button
        btn = await cg.get_variable(config[CONF_CLOSE_BTN])
        cg.add(var.set_btn_close(btn))
    if CONF_STOP_BTN in config:                                             # Manual stop button
        btn = await cg.get_variable(config[CONF_STOP_BTN])
        cg.add(var.set_btn_stop(btn))
    
    # Advanced button components
    if CONF_LEARN in config:                                                 # Auto learn function
        btn = await cg.get_variable(config[CONF_LEARN])
        cg.add(var.set_btn_learn(btn))
    if CONF_PARAMS_OD in config:                                            # Read parameters on demand
        btn = await cg.get_variable(config[CONF_PARAMS_OD])
        cg.add(var.set_btn_params_od(btn))
    if CONF_REMOTE_LEARN in config:                                         # Remote control learn
        btn = await cg.get_variable(config[CONF_REMOTE_LEARN])
        cg.add(var.set_btn_remote_learn(btn))
    
    # Number slider components (parameter groups)
    if CONF_SPEED_SLIDER in config:                                         # group 3 - Operation speed
        slider = await cg.get_variable(config[CONF_SPEED_SLIDER])
        cg.add(var.set_speed_slider(slider))
    if CONF_DECEL_DIST_SLIDER in config:                                    # group 4 - Deceleration distance
        slider = await cg.get_variable(config[CONF_DECEL_DIST_SLIDER])
        cg.add(var.set_decel_dist_slider(slider))
    if CONF_DECEL_SPEED_SLIDER in config:                                   # group 5 - Deceleration speed
        slider = await cg.get_variable(config[CONF_DECEL_SPEED_SLIDER])
        cg.add(var.set_decel_speed_slider(slider))
    if CONF_MAX_AMP in config:                                              # group 6 - Maximum current/torque
        slider = await cg.get_variable(config[CONF_MAX_AMP])
        cg.add(var.set_max_amp_slider(slider))
    if CONF_AUTO_CLOSE in config:                                           # group 1 - Auto close timer
        slider = await cg.get_variable(config[CONF_AUTO_CLOSE])
        cg.add(var.set_auto_close_slider(slider))
    if CONF_SMALL_GATE_TIMER in config:                                     # group 8 - Small gate timer
        slider = await cg.get_variable(config[CONF_SMALL_GATE_TIMER])
        cg.add(var.set_small_gate_timer(slider))
    if CONF_FORCE_DETECTION in config:                                      # group 10 - Force detection reaction
        num = await cg.get_variable(config[CONF_FORCE_DETECTION])
        cg.add(var.set_force_detection_number(num))

    # Text sensor components
    if CONF_DEVINFO in config:                                              # Device information
        txt = await cg.get_variable(config[CONF_DEVINFO])
        cg.add(var.set_txt_devinfo(txt))
    if CONF_LEARN_STATUS in config:                                         # Learn status information
        txt = await cg.get_variable(config[CONF_LEARN_STATUS])
        cg.add(var.set_txt_learn_status(txt))
    
    # Switch components (parameter groups)
    if CONF_PERMALOCK in config:                                            # group 15 - Permanent lock
        sw = await cg.get_variable(config[CONF_PERMALOCK])
        cg.add(var.set_sw_permalock(sw))
    if CONF_INFRA1 in config:                                               # group 13 - Infrared sensor 1
        sw = await cg.get_variable(config[CONF_INFRA1])
        cg.add(var.set_sw_infra1(sw))
    if CONF_INFRA2 in config:                                               # group 14 - Infrared sensor 2
        sw = await cg.get_variable(config[CONF_INFRA2])
        cg.add(var.set_sw_infra2(sw))