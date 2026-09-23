import numpy as np

from fruit_sorter.config import load_config
from fruit_sorter.dashboard import DashboardStore
from fruit_sorter.models import ControllerState, FruitType


def test_dashboard_tracks_live_data_and_sort_counts():
    config = load_config("config/robot.yaml")
    store = DashboardStore(config, dry_run=True)

    store.set_state(ControllerState.IDLE)
    store.publish_cycle(
        annotated_frame=np.zeros((120, 160, 3), dtype=np.uint8),
        frame_number=12,
        detections=[
            {
                "fruit": "apple",
                "track_id": 5,
                "confidence": 0.91,
                "colour_ratio": 0.72,
                "colour_valid": True,
                "stable": True,
                "inside_roi": True,
                "area_px": 2000,
                "center_x_px": 80.0,
                "center_y_px": 60.0,
                "robot_x_mm": 180.0,
                "robot_y_mm": 12.0,
                "status": "ready",
            }
        ],
        processing_fps=14.5,
        inference_ms=52.1,
        robot_telemetry={
            "connected": True,
            "fault": "none",
            "physical_estop": False,
            "outputs_enabled": True,
            "motion_busy": False,
            "pose_deg": [90, 90, 90, 90, 90, 60],
            "communication_error": None,
        },
    )

    store.record_sort(
        fruit=FruitType.APPLE,
        track_id=5,
        x_mm=180.0,
        y_mm=12.0,
        cycle_seconds=4.2,
    )
    snapshot = store.snapshot()

    assert snapshot["system"]["state"] == "idle"
    assert snapshot["system"]["frame_number"] == 12
    assert snapshot["sort_counts"]["apple"] == 1
    assert snapshot["sort_counts"]["total"] == 1
    assert snapshot["last_sort"]["fruit"] == "apple"
    assert snapshot["safety"]["fault"] == "none"
    assert snapshot["detections"][0]["status"] == "ready"

    jpeg, sequence = store.wait_for_jpeg(-1, timeout_s=0.1)
    assert jpeg is not None
    assert sequence >= 1
