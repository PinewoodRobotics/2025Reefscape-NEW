from backend.python.common.util.system import get_local_hostname


def test_get_local_hostname():
    hostname = get_local_hostname()
    assert isinstance(hostname, str)
    assert hostname != ""
    # By default we ensure a ".local" suffix is present.
    assert hostname.endswith(".local")
