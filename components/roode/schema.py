"""Schema definitions for Roode Home Assistant integration.

This module centralizes the data format used by calibration and analysis
components. A session record describes a single ranging measurement and
is shared between ESPHome firmware and Home Assistant post-processing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, List, TypedDict


class SessionRecordDict(TypedDict):
    """Dictionary representation of a session record."""

    type: str
    grid: str
    trial: int
    ranging: int
    timestamp: str
    data: List[Any]


@dataclass
class SessionRecord:
    """Represents one entry in a calibration or ranging session.

    Attributes:
        type: Identifier for the record type.
        grid: ROI grid or positional reference.
        trial: Sequential trial number within the session.
        ranging: Raw ranging result from the sensor in millimetres.
        timestamp: Creation time in ISO 8601 format.
        data: Arbitrary metadata captured with the measurement.
    """

    type: str
    grid: str
    trial: int
    ranging: int
    timestamp: datetime
    data: List[Any] = field(default_factory=list)

    @classmethod
    def from_dict(cls, payload: SessionRecordDict) -> "SessionRecord":
        """Create a :class:`SessionRecord` from its dictionary form."""

        return cls(
            type=payload["type"],
            grid=payload["grid"],
            trial=payload["trial"],
            ranging=payload["ranging"],
            timestamp=datetime.fromisoformat(payload["timestamp"]),
            data=list(payload.get("data", [])),
        )

    def to_dict(self) -> SessionRecordDict:
        """Convert the record to a serialisable dictionary."""

        return {
            "type": self.type,
            "grid": self.grid,
            "trial": self.trial,
            "ranging": self.ranging,
            "timestamp": self.timestamp.isoformat(),
            "data": self.data,
        }
