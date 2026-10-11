import subprocess
from pathlib import Path

import pytest

MESSAGES = [
    "AgcTaskState",
    "AirObservationArray",
    "ObstacleObservation",
    "ObstacleTrack",
    "ObstacleTrackArray",
    "SensorObservation",
    "TargetObservation",
    "TargetTrack",
    "TargetTrackArray",
]

GOLDEN_DIR = Path(__file__).parent / "golden"


def _interface_show(name: str) -> str:
    result = subprocess.run(
        ["ros2", "interface", "show", f"agc_msgs/msg/{name}"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


@pytest.mark.parametrize("name", MESSAGES)
def test_interface_contract(name: str) -> None:
    """ros2 interface show 输出须与冻结 golden 逐行一致。"""
    golden = (GOLDEN_DIR / f"{name}.txt").read_text(encoding="utf-8")
    actual = _interface_show(name)
    assert actual.strip() == golden.strip(), (
        f"{name} 接口与 golden 不一致：\n"
        f"--- golden ---\n{golden}\n--- actual ---\n{actual}"
    )
