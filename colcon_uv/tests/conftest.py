"""Shared pytest fixtures."""

import shutil
from unittest.mock import patch

import pytest

_real_which = shutil.which


@pytest.fixture(autouse=True)
def _uv_on_path():
    """Pretend uv is on PATH, so the tests do not need a real uv installed.

    The tests that run uv mock subprocess.run; the check for the uv executable
    must still pass. Tests of that check patch shutil.which themselves.
    """

    def which(cmd, *args, **kwargs):
        if cmd == "uv":
            return "/usr/bin/uv"
        return _real_which(cmd, *args, **kwargs)

    with patch("shutil.which", side_effect=which):
        yield
