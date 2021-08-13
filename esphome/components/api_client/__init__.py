import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_PASSWORD,
    CONF_UART_ID,
)
from esphome.core import coroutine_with_priority
from esphome.components import uart

DEPENDENCIES = []
CODEOWNERS = ["@RoganDawes"]

CONF_API_CLIENT = "api_client"
CONF_API_CLIENT_ID = "api_client_id"
CONF_API_STREAM_ID = "stream_id"
CONF_API_ENTITY_KEY = "key"

api_ns = cg.esphome_ns.namespace("api")
APIClient = api_ns.class_("APIClientConnection", cg.Component)
StreamAPIClient = api_ns.class_("StreamAPIClientConnection", APIClient, cg.Component)

API_CLIENT_STREAM_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(StreamAPIClient),
            cv.Optional(CONF_PASSWORD, default=""): cv.string,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)
API_CLIENT_SCHEMA = cv.Schema(
    cv.Any(
        cv.Required(API_CLIENT_STREAM_SCHEMA),
    )
)

CONFIG_SCHEMA = cv.Schema([API_CLIENT_SCHEMA])


@coroutine_with_priority(40.0)
async def to_code(config):
    for conf in config:
        stream = await cg.get_variable(conf[CONF_UART_ID])
        client = cg.new_Pvariable(conf[CONF_ID], stream)
        await cg.register_component(client, config)

        password = conf.get(CONF_PASSWORD)
        if password is not None:
            cg.add(client.set_password(password))

    cg.add_define("USE_API")
    cg.add_global(api_ns.using)
