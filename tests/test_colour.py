import cv2
import numpy as np

from fruit_sorter.colour import ColourVerifier
from fruit_sorter.config import (
    ColourConfig,
    FruitColourConfig,
    HSVRange,
)
from fruit_sorter.models import Detection, FruitType


def build_config():
    return ColourConfig(
        central_crop=0.8,
        morphology_kernel=3,
        min_pixels=50,
        fruits={
            FruitType.APPLE: FruitColourConfig(
                min_ratio=0.2,
                ranges=[
                    HSVRange(lower=(0, 80, 50), upper=(10, 255, 255)),
                    HSVRange(lower=(170, 80, 50), upper=(179, 255, 255)),
                ],
            ),
            FruitType.BANANA: FruitColourConfig(
                min_ratio=0.2,
                ranges=[HSVRange(lower=(20, 80, 50), upper=(38, 255, 255))],
            ),
            FruitType.ORANGE: FruitColourConfig(
                min_ratio=0.2,
                ranges=[HSVRange(lower=(8, 80, 50), upper=(22, 255, 255))],
            ),
        },
    )


def test_red_apple_is_accepted():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(frame, (100, 100), 55, (0, 0, 255), -1)
    detection = Detection(FruitType.APPLE, 0.9, (40, 40, 160, 160), 1)
    result = ColourVerifier(build_config()).verify(frame, detection)
    assert result.accepted
    assert result.ratio > 0.5


def test_green_apple_is_rejected():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(frame, (100, 100), 55, (0, 255, 0), -1)
    detection = Detection(FruitType.APPLE, 0.9, (40, 40, 160, 160), 1)
    result = ColourVerifier(build_config()).verify(frame, detection)
    assert not result.accepted
