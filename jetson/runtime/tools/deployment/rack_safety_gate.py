#!/usr/bin/env python3
"""Pure fail-closed temporal consensus and command-gating logic."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence


SLOT_ORDER = (
    "C1_L1",
    "C1_L2",
    "C1_L3",
    "C2_L1",
    "C2_L2",
    "C2_L3",
    "C3_L1",
    "C3_L2",
    "C3_L3",
)
KNOWN_STATES = {"EMPTY", "OCCUPIED"}
ALL_STATES = KNOWN_STATES | {"UNKNOWN"}


@dataclass(frozen=True)
class TemporalConsensus:
    stable_states: dict[str, str]
    votes: dict[str, dict[str, int]]
    occupancy_snapshot_valid: bool
    block_reason: str | None
    camera_fault: bool


class ByteTransport(Protocol):
    def write(self, payload: bytes) -> int: ...


def temporal_consensus(
    frame_states: Sequence[Mapping[str, str]],
    *,
    camera_faults: Sequence[bool],
    window_size: int = 5,
    minimum_votes: int = 4,
) -> TemporalConsensus:
    if len(frame_states) != window_size or len(camera_faults) != window_size:
        raise ValueError(f"Expected exactly {window_size} frames and fault flags")
    if not 1 <= minimum_votes <= window_size:
        raise ValueError("minimum_votes must be inside the temporal window")
    for states in frame_states:
        if set(states) != set(SLOT_ORDER):
            raise ValueError("Each frame must contain exactly the nine rack slots")
        if not set(states.values()) <= ALL_STATES:
            raise ValueError("Invalid rack state")

    fault = any(camera_faults)
    votes: dict[str, dict[str, int]] = {}
    stable: dict[str, str] = {}
    for slot in SLOT_ORDER:
        counts = Counter(states[slot] for states in frame_states)
        votes[slot] = {state: counts[state] for state in sorted(ALL_STATES)}
        known_winners = [
            state for state in sorted(KNOWN_STATES) if counts[state] >= minimum_votes
        ]
        stable[slot] = known_winners[0] if len(known_winners) == 1 else "UNKNOWN"

    if fault:
        # A camera/pose fault invalidates the whole time window even if four
        # older samples agree. A fresh fault-free window is required.
        stable = {slot: "UNKNOWN" for slot in SLOT_ORDER}
        return TemporalConsensus(stable, votes, False, "CAMERA_FAULT", True)
    if any(state == "UNKNOWN" for state in stable.values()):
        return TemporalConsensus(
            stable,
            votes,
            False,
            "UNSTABLE_OR_UNKNOWN_OCCUPANCY",
            False,
        )
    return TemporalConsensus(stable, votes, True, None, False)


def guarded_transport_write(
    consensus: TemporalConsensus,
    *,
    job_valid: bool,
    payload: bytes,
    transport: ByteTransport,
) -> int:
    """Write only when both occupancy and upstream job validation passed."""
    if not consensus.occupancy_snapshot_valid or not job_valid:
        return 0
    if not payload:
        raise ValueError("An authorized command payload must not be empty")
    written = transport.write(payload)
    if written != len(payload):
        raise RuntimeError(f"Short transport write: {written}/{len(payload)}")
    return written
