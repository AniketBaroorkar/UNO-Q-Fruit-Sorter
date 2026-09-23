import numpy as np

from fruit_sorter.calibration import PixelMapper
from fruit_sorter.config import CalibrationConfig
from fruit_sorter.models import FruitType, Point2D


def test_identity_homography():
    config = CalibrationConfig(
        calibration_complete=False,
        homography=np.eye(3).tolist(),
        roi_polygon_px=[(0, 0), (100, 0), (100, 100), (0, 100)],
        robot_reference_points_mm=[(0, 0), (1, 0), (1, 1), (0, 1)],
        pick_z_mm={
            FruitType.APPLE: 1,
            FruitType.BANANA: 1,
            FruitType.ORANGE: 1,
        },
        pick_offsets_mm={
            FruitType.APPLE: (0, 0),
            FruitType.BANANA: (0, 0),
            FruitType.ORANGE: (0, 0),
        },
    )
    mapper = PixelMapper(config)
    result = mapper.pixel_to_robot(Point2D(25.0, 40.0))
    assert result == Point2D(25.0, 40.0)
    assert mapper.inside_pick_roi(Point2D(50, 50))
    assert not mapper.inside_pick_roi(Point2D(150, 50))
