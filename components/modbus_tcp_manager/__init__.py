import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# Configuration constants
CONF_HOST = "host"
CONF_PORT = "port"
CONF_UNIT_ID = "unit_id"
CONF_WATCHDOG_REGISTER = "watchdog_register"
CONF_WATCHDOG_INTERVAL = "watchdog_interval"
CONF_SAFE_MODE_REGISTERS = "safe_mode_registers"
CONF_CONNECTION_TIMEOUT = "connection_timeout"
CONF_READ_TIMEOUT = "read_timeout"

# Namespace
modbus_tcp_ns = cg.esphome_ns.namespace("modbus_tcp")
ModbusTCPManager = modbus_tcp_ns.class_("ModbusTCPManager", cg.Component)

# Dependencies
DEPENDENCIES = ["network"]
CODEOWNERS = ["@Gucioo"]

# Safe mode register schema
SAFE_MODE_REGISTER_SCHEMA = cv.Schema({
    cv.Required("register"): cv.positive_int,
    cv.Required("value"): cv.int_range(min=-32768, max=32767),
})

# Configuration schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusTCPManager),
    cv.Required(CONF_HOST): cv.string,
    cv.Optional(CONF_PORT, default=502): cv.port,
    cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=255),
    cv.Optional(CONF_WATCHDOG_REGISTER): cv.positive_int,
    cv.Optional(CONF_WATCHDOG_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_SAFE_MODE_REGISTERS, default=[]): cv.All(cv.ensure_list(SAFE_MODE_REGISTER_SCHEMA)),
    # New timeout configurations for slow devices
    cv.Optional(CONF_CONNECTION_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_READ_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_HOST],
        config[CONF_PORT],
        config[CONF_UNIT_ID]
    )
    
    # Add watchdog configuration if specified
    if CONF_WATCHDOG_REGISTER in config:
        cg.add(var.set_watchdog_register(config[CONF_WATCHDOG_REGISTER]))
        cg.add(var.set_watchdog_interval(config[CONF_WATCHDOG_INTERVAL]))
    
    # Add safe mode registers if specified
    for safe_reg in config[CONF_SAFE_MODE_REGISTERS]:
        cg.add(var.add_safe_mode_register(safe_reg["register"], safe_reg["value"]))
    
    # Add timeout configurations
    cg.add(var.set_connection_timeout(config[CONF_CONNECTION_TIMEOUT]))
    cg.add(var.set_read_timeout(config[CONF_READ_TIMEOUT]))
    
    await cg.register_component(var, config)
