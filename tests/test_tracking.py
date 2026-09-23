from fruit_sorter.config import TrackingConfig
from fruit_sorter.models import Detection, FruitType, Point2D
from fruit_sorter.tracking import PickedRegistry, StabilityTracker


def cfg():
    return TrackingConfig(
        stable_frames=3,
        max_center_jitter_px=3,
        stale_after_frames=10,
        min_bbox_area_px=1,
        revalidate_max_shift_px=10,
        picked_memory_seconds=60,
        picked_radius_mm=20,
        rejected_cooldown_seconds=1,
    )


def test_stability_requires_multiple_low_jitter_frames():
    tracker = StabilityTracker(cfg())
    detection = Detection(FruitType.APPLE, 0.9, (10, 10, 30, 30), 4)
    for frame in range(1, 4):
        tracker.update([detection], frame)
    assert tracker.is_stable(detection)


def test_picked_registry_checks_track_and_location():
    registry = PickedRegistry(cfg())
    point = Point2D(100, 50)
    registry.mark(track_id=7, fruit=FruitType.ORANGE, point=point)

    assert registry.contains(
        track_id=7,
        fruit=FruitType.ORANGE,
        point=Point2D(500, 500),
    )
    assert registry.contains(
        track_id=99,
        fruit=FruitType.ORANGE,
        point=Point2D(110, 55),
    )
    assert not registry.contains(
        track_id=99,
        fruit=FruitType.APPLE,
        point=Point2D(110, 55),
    )
