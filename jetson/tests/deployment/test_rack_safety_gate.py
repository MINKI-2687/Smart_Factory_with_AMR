#!/usr/bin/env python3
"""Deterministic tests for the temporal filter and command block."""

from __future__ import annotations

import unittest

from rack_safety_gate import SLOT_ORDER, guarded_transport_write, temporal_consensus


def state(value: str) -> dict[str, str]:
    return {slot: value for slot in SLOT_ORDER}


class CountingTransport:
    def __init__(self) -> None:
        self.payloads: list[bytes] = []

    def write(self, payload: bytes) -> int:
        self.payloads.append(payload)
        return len(payload)

    @property
    def bytes_written(self) -> int:
        return sum(map(len, self.payloads))


class RackSafetyGateTests(unittest.TestCase):
    def test_four_of_five_known_state_is_stable(self) -> None:
        result = temporal_consensus(
            [state("EMPTY"), state("EMPTY"), state("UNKNOWN"), state("EMPTY"), state("EMPTY")],
            camera_faults=[False] * 5,
        )
        self.assertTrue(result.occupancy_snapshot_valid)
        self.assertEqual(set(result.stable_states.values()), {"EMPTY"})

    def test_three_of_five_is_unknown_and_blocked(self) -> None:
        frames = [state("EMPTY"), state("EMPTY"), state("EMPTY"), state("OCCUPIED"), state("UNKNOWN")]
        result = temporal_consensus(frames, camera_faults=[False] * 5)
        self.assertFalse(result.occupancy_snapshot_valid)
        self.assertEqual(set(result.stable_states.values()), {"UNKNOWN"})
        self.assertEqual(result.block_reason, "UNSTABLE_OR_UNKNOWN_OCCUPANCY")

    def test_camera_fault_invalidates_even_four_matching_frames(self) -> None:
        result = temporal_consensus([state("EMPTY")] * 5, camera_faults=[False, False, True, False, False])
        transport = CountingTransport()
        written = guarded_transport_write(
            result,
            job_valid=True,
            payload=b"would-be-command",
            transport=transport,
        )
        self.assertTrue(result.camera_fault)
        self.assertEqual(result.block_reason, "CAMERA_FAULT")
        self.assertEqual(set(result.stable_states.values()), {"UNKNOWN"})
        self.assertEqual(written, 0)
        self.assertEqual(transport.bytes_written, 0)

    def test_invalid_job_blocks_a_stable_snapshot(self) -> None:
        result = temporal_consensus([state("OCCUPIED")] * 5, camera_faults=[False] * 5)
        transport = CountingTransport()
        written = guarded_transport_write(
            result,
            job_valid=False,
            payload=b"would-be-command",
            transport=transport,
        )
        self.assertTrue(result.occupancy_snapshot_valid)
        self.assertEqual(written, 0)
        self.assertEqual(transport.bytes_written, 0)

    def test_valid_snapshot_and_job_reach_transport(self) -> None:
        result = temporal_consensus([state("EMPTY")] * 5, camera_faults=[False] * 5)
        transport = CountingTransport()
        payload = b"test-only-command"
        written = guarded_transport_write(
            result,
            job_valid=True,
            payload=payload,
            transport=transport,
        )
        self.assertEqual(written, len(payload))
        self.assertEqual(transport.payloads, [payload])


if __name__ == "__main__":
    unittest.main(verbosity=2)
