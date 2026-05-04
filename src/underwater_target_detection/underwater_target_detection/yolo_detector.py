"""YOLO-based target detector implementation.

This module provides a concrete ``DetectorBase`` implementation that uses
Ultralytics YOLOv8 (or compatible) via the ``ultralytics`` Python package.
If the package is not installed, the node degrades gracefully and logs a
warning — allowing the rest of the stack to be built and tested without a
GPU environment.
"""

from __future__ import annotations

import logging
from typing import List, Optional

import numpy as np

from underwater_target_detection.detector_base import Detection, DetectorBase

logger = logging.getLogger(__name__)

# Optional import — the detector reports its availability via `is_available()`
try:
    from ultralytics import YOLO as _YOLO  # type: ignore[import]

    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _YOLO = None  # type: ignore[assignment,misc]
    _ULTRALYTICS_AVAILABLE = False


def is_available() -> bool:
    """Return True if the Ultralytics YOLO package is installed."""
    return _ULTRALYTICS_AVAILABLE


class YOLODetector(DetectorBase):
    """Wraps Ultralytics YOLOv8 for underwater target detection.

    Parameters
    ----------
    confidence_threshold:
        Minimum detection confidence score to keep.
    target_classes:
        If non-empty, only detections with a class_id in this list are
        forwarded downstream.  Pass ``[]`` to keep all classes.
    device:
        Inference device string recognised by PyTorch, e.g. ``"cpu"``,
        ``"cuda:0"``, or ``"mps"``.
    """

    def __init__(
        self,
        confidence_threshold: float = 0.5,
        target_classes: Optional[List[int]] = None,
        device: str = "cpu",
    ) -> None:
        super().__init__(
            confidence_threshold=confidence_threshold,
            target_classes=target_classes,
        )
        self._device = device
        self._model: Optional[object] = None

    # ------------------------------------------------------------------
    # DetectorBase interface
    # ------------------------------------------------------------------

    def load_model(self, model_path: str) -> None:
        """Load the YOLO model from *model_path*."""
        if not _ULTRALYTICS_AVAILABLE:
            logger.warning(
                "ultralytics package is not installed. "
                "YOLODetector will return empty detections."
            )
            return
        self._model = _YOLO(model_path)
        logger.info("YOLODetector: loaded model from '%s' on device '%s'",
                    model_path, self._device)

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run YOLO inference and return filtered detections."""
        if self._model is None:
            return []

        img_h, img_w = image.shape[:2]

        results = self._model.predict(
            image,
            conf=self._confidence_threshold,
            device=self._device,
            verbose=False,
        )

        detections: List[Detection] = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1

                nx, ny = self._pixel_to_normalised(cx, cy, img_w, img_h)

                cls_name = (
                    result.names[cls_id]
                    if result.names and cls_id in result.names
                    else str(cls_id)
                )

                det = Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    bbox_center_x=cx,
                    bbox_center_y=cy,
                    bbox_width=w,
                    bbox_height=h,
                    image_x=nx,
                    image_y=ny,
                    estimated_distance=-1.0,
                    is_valid=True,
                )
                detections.append(det)

        detections = self._filter_by_class(detections)
        # Note: confidence filtering already applied inside YOLO predict, but
        # we apply it again for safety after class filtering.
        detections = self._filter_by_confidence(detections)
        return detections
