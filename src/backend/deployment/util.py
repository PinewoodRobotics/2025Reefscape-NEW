import dataclasses
from dataclasses import dataclass
from typing import Protocol
import subprocess
import time
import os
from zeroconf import Zeroconf, ServiceBrowser, ServiceListener, ServiceInfo


SERVICE = "_watchdog._udp.local."
DISCOVERY_TIMEOUT = 2.0
BACKEND_DEPLOYMENT_PATH = "/opt/blitz/B.L.I.T.Z/backend"
GITIGNORE_PATH = ".gitignore"
VENV_PATH = ".venv/bin/python"


@dataclass
class Module:
    pass


@dataclass
class CompilableModule(Module):
    project_root_folder_path: str


@dataclass
class RunnableModule(Module):
    extra_run_args: list[tuple[str, str]]
    equivalent_run_definition: str

    def get_run_command(self) -> str:
        return ""

    def get_compile_command(self) -> str | None:
        return None

    def get_lang_folder_name(self) -> str:
        if isinstance(self, PythonModule):
            return "python"
        elif isinstance(self, RustModule):
            return "rust"
        elif isinstance(self, ProtobufModule):
            return "proto"
        elif isinstance(self, ThriftModule):
            return "thrift"
        else:
            raise ValueError(f"Unknown module type: {type(self)}")

    def get_extra_run_args(self) -> str:
        return (
            " ".join([f"--{arg[0]} {arg[1]}" for arg in self.extra_run_args])
            if self.extra_run_args
            else ""
        )


@dataclass
class RustModule(RunnableModule):
    runnable_name: str
    cross_compile_target: str | None = None
    remote_binary_path: str | None = None

    def get_run_command(self) -> str:
        extra_run_args = self.get_extra_run_args()
        remote_path = self.remote_binary_path or f"bin/{self.runnable_name}"
        return f"{remote_path} {extra_run_args}".strip()

    def get_compile_command(self) -> str | None:
        return f"cargo build --release --bin {self.runnable_name}"


@dataclass
class ProtobufModule(CompilableModule):
    pass


@dataclass
class ThriftModule(CompilableModule):
    pass


@dataclass
class PythonModule(RunnableModule):
    local_main_file_path: str
    local_root_folder_path: str

    def get_run_command(self) -> str:
        extra_run_args = self.get_extra_run_args()
        return f"{VENV_PATH} -u backend/{self.local_root_folder_path}/{self.local_main_file_path} {extra_run_args}"


class ZeroconfService(Protocol):
    server: str | None


@dataclass
class RaspberryPi:
    address: str
    host: str = dataclasses.field(default="ubuntu")
    password: str = dataclasses.field(default="ubuntu")
    port: int = dataclasses.field(default=22)

    @classmethod
    def _from_zeroconf(cls, service: ServiceInfo):
        properties = {
            k.decode("utf-8") if isinstance(k, bytes) else k: (
                v.decode("utf-8") if isinstance(v, bytes) else v
            )
            for k, v in (service.properties or {}).items()
        }

        address = (
            (properties.get("hostname_local") or "").rstrip(".")
            or (service.server or "").rstrip(".")
            or None
        )

        if not address:
            raise ValueError("Cannot extract Pi address from zeroconf ServiceInfo")

        return cls(address=address, host=address)

    @classmethod
    def discover_all(cls):
        raspberrypis: list[RaspberryPi] = []
        zc = Zeroconf()

        class _Listener(ServiceListener):
            def __init__(self, out: list[RaspberryPi]):
                self.out: list[RaspberryPi] = out

            def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
                info = zc.get_service_info(type_, name)
                if info is None:
                    return
                try:
                    self.out.append(RaspberryPi._from_zeroconf(info))
                except Exception:
                    pass

            def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
                return

            def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
                return

        _ = ServiceBrowser(zc, SERVICE, listener=_Listener(raspberrypis))
        time.sleep(DISCOVERY_TIMEOUT)
        zc.close()
        return raspberrypis


def with_discovery_timeout(timeout_seconds: float):
    global DISCOVERY_TIMEOUT
    DISCOVERY_TIMEOUT = timeout_seconds  # pyright: ignore[reportConstantRedefinition]


def with_custom_backend_dir(backend_dir: str):
    global BACKEND_DEPLOYMENT_PATH
    BACKEND_DEPLOYMENT_PATH = backend_dir  # pyright: ignore[reportConstantRedefinition]


def _deploy_backend_to_pi(
    pi: RaspberryPi,
    backend_local_path: str = "src/backend/",
):
    base_path = os.path.normpath(backend_local_path)

    if not base_path.endswith("/"):
        base_path = base_path + "/"

    remote_target_dir = f"{BACKEND_DEPLOYMENT_PATH.rstrip('/')}"

    mkdir_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        "-p",
        str(getattr(pi, "port", 22)),
        f"ubuntu@{pi.address}",
        f"sudo mkdir -p {remote_target_dir}",
    ]

    mkdir_proc = subprocess.run(mkdir_cmd)
    if mkdir_proc.returncode != 0:
        raise Exception(
            f"Failed to create remote directory {remote_target_dir} on {pi.address}: {mkdir_proc.returncode}"
        )

    target = f"ubuntu@{pi.address}:{remote_target_dir}"

    rsync_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "rsync",
        "-av",
        "--progress",
        "--rsync-path=sudo rsync",
        "--exclude-from=" + GITIGNORE_PATH,
        "-e",
        f"ssh -p {getattr(pi, 'port', 22)} -o StrictHostKeyChecking=no",
    ]

    rsync_cmd.extend([base_path, target])

    exit_code = subprocess.run(rsync_cmd)
    if exit_code.returncode != 0:
        raise Exception(
            f"Failed to deploy backend from {base_path} on {pi.address}: {exit_code.returncode}"
        )


def _build_rust_locally(module: RunnableModule) -> str:
    """Build a Rust module locally and return the path to the binary."""
    if not isinstance(module, RustModule):
        raise ValueError("Module must be a RustModule")

    target_info = (
        f" for {module.cross_compile_target}" if module.cross_compile_target else ""
    )
    print(f"Building {module.runnable_name} locally{target_info}...")

    # Use 'cargo zigbuild' for cross-compilation (works on Mac M3)
    # Use 'cargo' for native builds
    if module.cross_compile_target:
        build_cmd = [
            "cargo",
            "zigbuild",
            "--release",
            "--bin",
            module.runnable_name,
            "--target",
            module.cross_compile_target,
        ]
    else:
        build_cmd = [
            "cargo",
            "build",
            "--release",
            "--bin",
            module.runnable_name,
        ]

    exit_code = subprocess.run(build_cmd)
    if exit_code.returncode != 0:
        raise Exception(
            f"Failed to build {module.runnable_name} locally: {exit_code.returncode}"
        )

    # Determine the local binary path based on whether we're cross-compiling
    if module.cross_compile_target:
        local_binary_path = (
            f"target/{module.cross_compile_target}/release/{module.runnable_name}"
        )
    else:
        local_binary_path = f"target/release/{module.runnable_name}"

    if not os.path.exists(local_binary_path):
        raise FileNotFoundError(f"Built binary not found at {local_binary_path}")

    print(f"✓ Built {module.runnable_name} successfully")
    return local_binary_path


def _deploy_rust_binary(pi: RaspberryPi, module: RustModule, local_binary_path: str):
    """Deploy a locally-built Rust binary to the Pi."""
    # Determine remote path
    remote_path = module.remote_binary_path or f"../bin/{module.runnable_name}"
    remote_full_path = f"{BACKEND_DEPLOYMENT_PATH.rstrip('/')}/{remote_path}"
    remote_dir = os.path.dirname(remote_full_path)

    print(f"Deploying {module.runnable_name} to {pi.address}:{remote_full_path}...")

    # Create remote directory
    mkdir_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        "-p",
        str(getattr(pi, "port", 22)),
        f"ubuntu@{pi.address}",
        f"sudo mkdir -p {remote_dir}",
    ]

    mkdir_proc = subprocess.run(mkdir_cmd)
    if mkdir_proc.returncode != 0:
        raise Exception(
            f"Failed to create remote directory {remote_dir} on {pi.address}: {mkdir_proc.returncode}"
        )

    # Upload the binary
    rsync_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "rsync",
        "-av",
        "--progress",
        "--rsync-path=sudo rsync",
        "-e",
        f"ssh -p {getattr(pi, 'port', 22)} -o StrictHostKeyChecking=no",
        local_binary_path,
        f"ubuntu@{pi.address}:{remote_full_path}",
    ]

    rsync_proc = subprocess.run(rsync_cmd)
    if rsync_proc.returncode != 0:
        raise Exception(
            f"Failed to upload binary to {remote_full_path} on {pi.address}: {rsync_proc.returncode}"
        )

    # Make it executable
    chmod_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        "-p",
        str(getattr(pi, "port", 22)),
        f"ubuntu@{pi.address}",
        f"sudo chmod +x {remote_full_path}",
    ]

    chmod_proc = subprocess.run(chmod_cmd)
    if chmod_proc.returncode != 0:
        raise Exception(
            f"Failed to chmod +x on {remote_full_path} at {pi.address}: {chmod_proc.returncode}"
        )

    print(f"✓ Deployed {module.runnable_name} successfully")


def _build_runnable(pi: RaspberryPi, module: RunnableModule):
    compile_command = module.get_compile_command()
    if compile_command is None:
        return

    build_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        "-p",
        str(getattr(pi, "port", 22)),
        f"ubuntu@{pi.address}",
        f"bash -c '{compile_command}'",
    ]

    exit_code = subprocess.run(build_cmd)
    if exit_code.returncode != 0:
        raise Exception(f"Failed to build {module}: {exit_code.returncode}")


def _deploy_compilable(pi: RaspberryPi, modules: list[Module]):
    for module in modules:
        if isinstance(module, RunnableModule):
            # if not module.get_compile_command() is None:
            # _build_runnable(pi, module)
            continue

        assert isinstance(module, CompilableModule)

        remote_target_dir = f"{BACKEND_DEPLOYMENT_PATH.rstrip('/')}"
        target = f"ubuntu@{pi.address}:{remote_target_dir}"

        rsync_cmd = [
            "sshpass",
            "-p",
            pi.password,
            "rsync",
            "-av",
            "--progress",
            "--rsync-path=sudo rsync",
            "--exclude-from=" + GITIGNORE_PATH,
            "--delete",
            "-e",
            f"ssh -p {getattr(pi, 'port', 22)} -o StrictHostKeyChecking=no",
            module.project_root_folder_path,
            target,
        ]

        exit_code = subprocess.run(rsync_cmd)
        if exit_code.returncode != 0:
            raise Exception(
                f"Failed to check if module {module.project_root_folder_path} is deployed on {pi.address}: {exit_code.returncode}"
            )


def _deploy_on_pi(
    pi: RaspberryPi,
    modules: list[Module],
    backend_local_path: str = "src/backend/",
):
    # First, build all Rust modules locally if needed
    rust_binaries: dict[str, str] = {}
    for module in modules:
        if isinstance(module, RustModule):
            local_binary_path = _build_rust_locally(module)
            rust_binaries[module.runnable_name] = local_binary_path

    # Deploy backend code (Python, etc.), excluding Rust source if building locally
    _deploy_backend_to_pi(pi, backend_local_path)

    # Deploy locally-built Rust binaries
    for module in modules:
        if isinstance(module, RustModule):
            local_binary_path = rust_binaries[module.runnable_name]
            _deploy_rust_binary(pi, module, local_binary_path)

    # Deploy and build other compilable modules (proto, thrift, remote Rust builds)
    _deploy_compilable(pi, modules)

    restart_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        "-o",
        "StrictHostKeyChecking=no",
        "-p",
        str(getattr(pi, "port", 22)),
        f"ubuntu@{pi.address}",
        f"echo {pi.password} | sudo -S systemctl restart startup.service",
    ]

    exit_code = subprocess.run(restart_cmd)
    if exit_code.returncode != 0:
        raise Exception(
            f"Failed to restart backend on {pi.address}: {exit_code.returncode}"
        )


def with_exclusions_from_gitignore(gitignore_path: str):
    global GITIGNORE_PATH
    GITIGNORE_PATH = gitignore_path  # pyright: ignore[reportConstantRedefinition]


def with_preset_pi_addresses(
    pi_addresses: list[RaspberryPi],
    modules: list[Module],
    backend_local_path: str = "src/backend/",
):
    for pi in pi_addresses:
        _deploy_on_pi(pi, modules, backend_local_path)


def _verify_self():
    import importlib

    deploy_module = importlib.import_module("backend.deploy")
    all_functions = deploy_module.__dict__
    if "get_modules" not in all_functions:
        raise Exception(
            "get_modules() not found in backend.deploy. Please add a function that returns a list[Module] named get_modules(). THIS IS A REQUIRED FUNCTION."
        )

    get_modules = all_functions["get_modules"]
    modules = get_modules()
    if not isinstance(modules, list) or not all(isinstance(m, Module) for m in modules):
        raise Exception(
            f"get_modules() returned {type(modules)} with element types {[type(m) for m in modules] if isinstance(modules, list) else 'N/A'} instead of list[Module]"
        )


def with_automatic_discovery(
    modules: list[Module], backend_local_path: str = "src/backend/"
):
    raspberrypis = RaspberryPi.discover_all()
    with_preset_pi_addresses(raspberrypis, modules, backend_local_path)
    print()
    print()
    print(f"Deployed on {len(raspberrypis)} Pis")
    print()


_verify_self()
