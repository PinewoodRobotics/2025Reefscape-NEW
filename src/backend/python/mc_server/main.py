import asyncio

from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.python.common.debug.logger import LogLevel, error, init_logging, success
from backend.python.common.util.system import get_system_name, load_configs

SERVER_JAR_PATH = "/opt/blitz/mc_server"

ALLOWED_GB_MEMORY = 4
ALLOWED_CORES = 4

JAVA_ARGS = f"-XX:ActiveProcessorCount={ALLOWED_CORES} -Xms{ALLOWED_GB_MEMORY}G -Xmx{ALLOWED_GB_MEMORY}G -jar"
SERVER_JAR_NAME = "mc_server-1.0.0.jar"


async def main():
    basic_system_config, _ = load_configs()
    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )

    await autobahn_server.begin()

    init_logging(
        "MC_SERVER",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=autobahn_server,
        system_name=get_system_name(),
    )

    success("Starting MC server")

    process = await asyncio.create_subprocess_shell(
        f"cd {SERVER_JAR_PATH} && java {JAVA_ARGS} {SERVER_JAR_NAME} nogui",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    if not process.stderr or not process.stdout:
        error("Failed to start MC server")
        return

    while True:
        line = await process.stdout.readline()
        if not line:
            break
        success(line.decode().rstrip())

    stderr = await process.stderr.read()
    if stderr:
        error(f"{stderr.decode().rstrip()}")

    await process.wait()


if __name__ == "__main__":
    asyncio.run(main())
