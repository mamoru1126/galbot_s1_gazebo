import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # galbot_s1_control package (source tree)
SRC = os.environ.get('GALBOT_SRC', os.path.abspath(os.path.join(HERE, '..', '..')))
URDF_DIR = os.environ.get('GALBOT_URDF_DIR', '')


def urdf_text(kind):
    path = os.path.join(URDF_DIR, kind + '.urdf')
    if not URDF_DIR or not os.path.isfile(path):
        pytest.skip('processed URDF not available (set GALBOT_URDF_DIR, see ci/unit_tests.sh)')
    with open(path) as f:
        return f.read()


@pytest.fixture
def gz_urdf():
    return urdf_text('gz')


@pytest.fixture
def classic_urdf():
    return urdf_text('classic')


@pytest.fixture
def description_urdf():
    return urdf_text('description')


@pytest.fixture
def src():
    return SRC
