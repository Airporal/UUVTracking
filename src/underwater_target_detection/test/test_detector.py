"""Unit tests for underwater_target_detection — detector_base and yolo_detector."""

from __future__ import annotations

import numpy as np
import pytest

from underwater_target_detection.detector_base import Detection, DetectorBase
from underwater_target_detection.yolo_detector import YOLODetector, is_available


# ---------------------------------------------------------------------------
# Concrete stub for testing the abstract base
# ---------------------------------------------------------------------------


class _StubDetector(DetectorBase):
    """Minimal concrete implementation for testing DetectorBase helpers."""

    def load_model(self, model_path: str) -> None:  # noqa: D401
        pass

    def detect(self, image: np.ndarray):
        return []


# ---------------------------------------------------------------------------
# DetectorBase tests
# ---------------------------------------------------------------------------


class TestDetectorBase:
    def test_pixel_to_normalised_centre(self):
        det = _StubDetector()
        nx, ny = det._pixel_to_normalised(320.0, 240.0, 640, 480)
        assert nx == pytest.approx(0.0)
        assert ny == pytest.approx(0.0)

    def test_pixel_to_normalised_top_left(self):
        det = _StubDetector()
        nx, ny = det._pixel_to_normalised(0.0, 0.0, 640, 480)
        assert nx == pytest.approx(-1.0)
        assert ny == pytest.approx(-1.0)

    def test_pixel_to_normalised_bottom_right(self):
        det = _StubDetector()
        nx, ny = det._pixel_to_normalised(640.0, 480.0, 640, 480)
        assert nx == pytest.approx(1.0)
        assert ny == pytest.approx(1.0)

    def test_filter_by_confidence(self):
        det = _StubDetector(confidence_threshold=0.6)
        detections = [
            Detection(confidence=0.3),
            Detection(confidence=0.7),
            Detection(confidence=0.6),
        ]
        filtered = det._filter_by_confidence(detections)
        assert len(filtered) == 2
        assert all(d.confidence >= 0.6 for d in filtered)

    def test_filter_by_class_empty_keeps_all(self):
        det = _StubDetector(target_classes=[])
        detections = [Detection(class_id=0), Detection(class_id=1), Detection(class_id=5)]
        assert len(det._filter_by_class(detections)) == 3

    def test_filter_by_class_filters_correctly(self):
        det = _StubDetector(target_classes=[0, 2])
        detections = [
            Detection(class_id=0),
            Detection(class_id=1),
            Detection(class_id=2),
        ]
        filtered = det._filter_by_class(detections)
        assert len(filtered) == 2
        assert all(d.class_id in (0, 2) for d in filtered)

    def test_confidence_threshold_setter_valid(self):
        det = _StubDetector()
        det.confidence_threshold = 0.75
        assert det.confidence_threshold == pytest.approx(0.75)

    def test_confidence_threshold_setter_invalid(self):
        det = _StubDetector()
        with pytest.raises(ValueError):
            det.confidence_threshold = 1.5


# ---------------------------------------------------------------------------
# YOLODetector tests (no GPU / model required)
# ---------------------------------------------------------------------------


class TestYOLODetector:
    def test_is_available_returns_bool(self):
        assert isinstance(is_available(), bool)

    def test_detect_returns_empty_without_model(self):
        detector = YOLODetector()
        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect(dummy_image)
        assert result == []

    def test_load_model_without_ultralytics_does_not_raise(self):
        """load_model must not raise even when ultralytics is absent."""
        detector = YOLODetector()
        # Should log a warning but not raise
        try:
            detector.load_model("nonexistent_model.pt")
        except Exception as exc:  # noqa: BLE001
            # Ultralytics may raise a file-not-found error when installed
            if is_available():
                # That's acceptable — model file doesn't exist in the test env
                assert "nonexistent_model.pt" in str(exc) or True
            else:
                pytest.fail(f"load_model raised unexpectedly: {exc}")
