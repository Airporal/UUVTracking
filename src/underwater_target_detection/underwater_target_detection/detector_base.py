"""Abstract base class for target detectors.

Any concrete detector (YOLO, SSD, classical, …) must inherit from
``DetectorBase`` and implement ``detect()``.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Detection:
    """A single raw detection result returned by a detector backend."""

    class_id: int = 0
    class_name: str = ""
    confidence: float = 0.0

    # Pixel-space bounding box
    bbox_center_x: float = 0.0
    bbox_center_y: float = 0.0
    bbox_width: float = 0.0
    bbox_height: float = 0.0

    # Normalised image coordinates (computed from pixel coords + image size)
    image_x: float = 0.0
    image_y: float = 0.0

    # Optional distance estimate (metres); -1.0 when unknown
    estimated_distance: float = -1.0

    is_valid: bool = True


class DetectorBase(abc.ABC):
    """Abstract interface that all detector backends must implement."""

    def __init__(
        self,
        confidence_threshold: float = 0.5,
        target_classes: Optional[List[int]] = None,
    ) -> None:
        self._confidence_threshold = confidence_threshold
        self._target_classes: List[int] = target_classes or []

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def load_model(self, model_path: str) -> None:
        """Load / initialise the underlying detector model."""

    @abc.abstractmethod
    def detect(
        self, image: np.ndarray
    ) -> List[Detection]:
        """Run inference on *image* and return a list of detections.

        Parameters
        ----------
        image:
            BGR or RGB NumPy array (H×W×3, uint8).

        Returns
        -------
        list[Detection]
            All detections that pass the confidence threshold and class filter.
        """

    # ------------------------------------------------------------------
    # Helpers available to all subclasses
    # ------------------------------------------------------------------

    def _pixel_to_normalised(
        self,
        cx: float,
        cy: float,
        img_w: int,
        img_h: int,
    ) -> Tuple[float, float]:
        """Convert pixel centre to normalised image coordinates [-1, 1]."""
        nx = (cx - img_w / 2.0) / (img_w / 2.0)
        ny = (cy - img_h / 2.0) / (img_h / 2.0)
        return float(nx), float(ny)

    def _filter_by_class(self, detections: List[Detection]) -> List[Detection]:
        """Keep only detections whose class_id is in *_target_classes*.

        If *_target_classes* is empty, all detections are kept.
        """
        if not self._target_classes:
            return detections
        return [d for d in detections if d.class_id in self._target_classes]

    def _filter_by_confidence(self, detections: List[Detection]) -> List[Detection]:
        return [d for d in detections if d.confidence >= self._confidence_threshold]

    @property
    def confidence_threshold(self) -> float:
        return self._confidence_threshold

    @confidence_threshold.setter
    def confidence_threshold(self, value: float) -> None:
        if not 0.0 <= value <= 1.0:
            raise ValueError("confidence_threshold must be in [0.0, 1.0]")
        self._confidence_threshold = value
