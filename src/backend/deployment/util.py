import dataclasses
from dataclasses import dataclass
import subprocess
import time
import os
from zeroconf import ServiceInfo, Zeroconf, ServiceBrowser


SERVICE = "_deploy._udp.local."
DISCOVERY_TIMEOUT = 2.0
BACKEND_DEPLOYMENT_PATH = "~/Documents/B.L.I.T.Z/backend"
GITIGNORE_PATH = ".gitignore"
VENV_PATH = ".venv/bin/python"
MODULES = []


@dataclass
class CommonModule:
    local_root_folder_path: str
    extra_run_args: list[tuple[str, str]]
    equivalent_run_definition: str

    def get_run_command(self) -> str:
        return ""

    def get_lang_folder_name(self) -> str:
        if isinstance(self, PythonModule):
            return "python"
        elif isinstance(self, RustModule):
            return "rust"
        elif isinstance(self, ProtobufModule):
            return "proto"
        else:
            raise ValueError(f"Unknown module type: {type(self)}")

    def get_folder_name(self) -> str:
        return self.local_root_folder_path.rstrip("/").split("/")[-1]

    def get_extra_run_args(self) -> str:
        return (
            " ".join([f"--{arg[0]} {arg[1]}" for arg in self.extra_run_args])
            if self.extra_run_args
            else ""
        )


@dataclass
class RustModule(CommonModule):
    runnable_name: str

    def get_run_command(self) -> str:
        extra_run_args = self.get_extra_run_args()
        return f"cargo run --bin {self.runnable_name} -- {extra_run_args}"


@dataclass
class ProtobufModule(CommonModule):
    pass


@dataclass
class PythonModule(CommonModule):
    local_main_file_path: str

    def get_run_command(self) -> str:
        extra_run_args = self.get_extra_run_args()

        return f"{VENV_PATH} -m backend/{self.local_root_folder_path}/{self.local_main_file_path} {extra_run_args}"


@dataclass
class RaspberryPi:
    address: str
    host: str = dataclasses.field(default="ubuntu")
    password: str = dataclasses.field(default="ubuntu")

    @classmethod
    def _from_zeroconf(cls, service: ServiceInfo):
        assert service.server is not None
        return cls(
            address=service.server,
            host=service.server,
        )

    @classmethod
    def discover_all(cls):
        raspberrypis: list[RaspberryPi] = []
        zc = Zeroconf()
        _ = ServiceBrowser(
            zc,
            SERVICE,
            handlers=[lambda service: raspberrypis.append(cls._from_zeroconf(service))],
        )
        time.sleep(DISCOVERY_TIMEOUT)
        zc.close()
        return raspberrypis


def with_discovery_timeout(timeout_seconds: float):
    global DISCOVERY_TIMEOUT
    DISCOVERY_TIMEOUT = timeout_seconds  # pyright: ignore[reportConstantRedefinition]


def with_custom_backend_dir(backend_dir: str):
    global BACKEND_DEPLOYMENT_PATH
    BACKEND_DEPLOYMENT_PATH = backend_dir  # pyright: ignore[reportConstantRedefinition]


def _deploy_module(
    module: CommonModule, pi: RaspberryPi, backend_local_path: str = "src/backend/"
):
    normalized_local_root = module.local_root_folder_path.strip().lstrip("./")
    base_path = os.path.normpath(
        os.path.join(backend_local_path, normalized_local_root)
    )

    if not base_path.endswith("/"):
        base_path = base_path + "/"

    remote_lang_dir = (
        f"{BACKEND_DEPLOYMENT_PATH.rstrip('/')}/{module.get_lang_folder_name()}"
    )
    remote_target_dir = f"{remote_lang_dir}/{module.get_folder_name()}"

    mkdir_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        f"ubuntu@{pi.address}",
        f"mkdir -p {remote_target_dir}",
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
        "--exclude-from=" + GITIGNORE_PATH,
        "--delete",
        "-e",
        "ssh -o StrictHostKeyChecking=no",
        base_path,
        target,
    ]

    exit_code = subprocess.run(rsync_cmd)
    if exit_code.returncode != 0:
        raise Exception(
            f"Failed to deploy {base_path} on {pi.address}: {exit_code.returncode}"
        )


def _deploy_on_pi(
    pi: RaspberryPi,
    modules: list[CommonModule],
    backend_local_path: str = "src/backend/",
):
    for module in modules:
        _deploy_module(module, pi, backend_local_path)

    restart_cmd = [
        "sshpass",
        "-p",
        pi.password,
        "ssh",
        f"ubuntu@{pi.address}",
        f"sudo systemctl restart startup.service && cd $(dirname {BACKEND_DEPLOYMENT_PATH}) && make generate",
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
    backend_local_path: str = "src/backend/",
):
    for pi in pi_addresses:
        _deploy_on_pi(pi, MODULES, backend_local_path)


def with_automatic_discovery(backend_local_path: str = "src/backend/"):
    raspberrypis = RaspberryPi.discover_all()
    with_preset_pi_addresses(raspberrypis, backend_local_path)
    print(f"Deployed on {len(raspberrypis)} Pis")


def set_modules(modules: list[CommonModule] | CommonModule):
    global MODULES
    if isinstance(modules, CommonModule):
        modules = [modules]

    MODULES = modules  # pyright: ignore[reportConstantRedefinition]
