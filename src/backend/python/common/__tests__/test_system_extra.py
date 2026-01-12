import socket

from backend.python.common.util.system import get_local_hostname, get_local_ip


def test_get_local_hostname_without_suffix(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "example-host")
    assert get_local_hostname(include_local_suffix=False) == "example-host"


def test_get_local_hostname_adds_local_suffix(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "example-host")
    assert get_local_hostname(include_local_suffix=True) == "example-host.local"


def test_get_local_hostname_preserves_existing_suffix(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "example-host.local")
    assert get_local_hostname(include_local_suffix=True) == "example-host.local"


def test_get_local_ip_returns_none_when_interface_missing(monkeypatch):
    import netifaces

    def raise_value_error(_iface: str):
        raise ValueError("no such iface")

    monkeypatch.setattr(netifaces, "ifaddresses", raise_value_error)
    assert get_local_ip("does-not-exist") is None


def test_get_local_ip_returns_ipv4_address(monkeypatch):
    import netifaces

    monkeypatch.setattr(
        netifaces,
        "ifaddresses",
        lambda _iface: {netifaces.AF_INET: [{"addr": "192.168.0.10"}]},
    )
    assert get_local_ip("eth0") == "192.168.0.10"
