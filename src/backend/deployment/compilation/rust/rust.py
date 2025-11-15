from enum import Enum
import os
from pathlib import Path
import subprocess


class SystemType(Enum):
    PI5 = "pi"


class RustCompilation:
    def __init__(self, system_type: SystemType):
        self.system_type = system_type

    def compile(self, module_name: str):
        if self.system_type == SystemType.PI5:
            return self.compile_pi5(module_name)

    def compile_pi5(self, module_name: str):
        # Build the Docker image with the module name as a build arg
        _ = subprocess.run(
            [
                "docker",
                "build",
                "--build-arg",
                f"MODULE_NAME={module_name}",
                "-f",
                "./src/backend/deployment/compilation/rust/Dockerfile.pi5",
                "-t",
                f"rust-pi5-{module_name}",
                ".",
            ]
        )

        # Run the container with volume mount to access source
        # The compiled binary will be in target/aarch64-unknown-linux-gnu/release/
        _ = subprocess.run(
            [
                "docker",
                "run",
                "-v",
                f"{os.getcwd()}/:/work",
                f"rust-pi5-{module_name}",
            ]
        )


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python rust.py <module_name>")
        sys.exit(1)

    module_name = sys.argv[1]
    e = RustCompilation(SystemType.PI5)
    e.compile(module_name)
