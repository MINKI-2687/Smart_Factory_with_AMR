#!/usr/bin/env python3
"""Pure fail-closed shape-to-empty-slot selection for the 3x3 rack."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


SHAPE_NAMES = {0: "NONE", 1: "CIRCLE", 2: "SQUARE", 3: "TRIANGLE"}
SHAPE_TO_COLUMN = {1: "C1", 2: "C2", 3: "C3"}

# Physical numbering viewed from the camera:
#   L3: 7 8 9
#   L2: 4 5 6
#   L1: 1 2 3
SLOT_INDEX = {
    "C1_L1": 1,
    "C2_L1": 2,
    "C3_L1": 3,
    "C1_L2": 4,
    "C2_L2": 5,
    "C3_L2": 6,
    "C1_L3": 7,
    "C2_L3": 8,
    "C3_L3": 9,
}
SLOT_NAMES = tuple(SLOT_INDEX)
KNOWN_STATES = {"EMPTY", "OCCUPIED"}


@dataclass(frozen=True)
class SlotSelection:
    accepted: bool
    reason: str
    shape: int
    shape_name: str
    column: str | None = None
    slot_name: str | None = None
    slot_index: int | None = None


def blocked(shape: int, reason: str, column: str | None = None) -> SlotSelection:
    return SlotSelection(
        accepted=False,
        reason=reason,
        shape=shape,
        shape_name=SHAPE_NAMES.get(shape, f"INVALID_{shape}"),
        column=column,
    )


def select_slot(shape: int, camera_status: Mapping[str, Any]) -> SlotSelection:
    """Select the lowest EMPTY slot in the shape's fixed column.

    This function never performs I/O.  It fails closed unless the monitor is
    STABLE, its pose is aligned, and all nine stable states are known.
    """
    if shape == 0:
        return blocked(shape, "NONE")
    if shape not in SHAPE_TO_COLUMN:
        return blocked(shape, "INVALID_SHAPE")

    column = SHAPE_TO_COLUMN[shape]
    monitor_status = camera_status.get("status")
    if monitor_status != "STABLE":
        return blocked(shape, f"CAMERA_NOT_STABLE:{monitor_status}", column)

    pose = camera_status.get("pose")
    if not isinstance(pose, Mapping) or pose.get("aligned") is not True:
        return blocked(shape, "CAMERA_POSE_NOT_ALIGNED", column)

    stable_states = camera_status.get("stable_states")
    if not isinstance(stable_states, Mapping):
        return blocked(shape, "INVALID_STABLE_STATES", column)
    if set(stable_states) != set(SLOT_NAMES):
        return blocked(shape, "INCOMPLETE_STABLE_STATES", column)
    if not set(stable_states.values()) <= (KNOWN_STATES | {"UNKNOWN"}):
        return blocked(shape, "INVALID_SLOT_STATE", column)
    if any(state == "UNKNOWN" for state in stable_states.values()):
        return blocked(shape, "UNKNOWN_OCCUPANCY", column)

    for level in ("L1", "L2", "L3"):
        slot_name = f"{column}_{level}"
        if stable_states[slot_name] == "EMPTY":
            return SlotSelection(
                accepted=True,
                reason="SELECTED",
                shape=shape,
                shape_name=SHAPE_NAMES[shape],
                column=column,
                slot_name=slot_name,
                slot_index=SLOT_INDEX[slot_name],
            )

    return blocked(shape, "COLUMN_FULL", column)
