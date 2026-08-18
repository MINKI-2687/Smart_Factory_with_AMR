#!/usr/bin/env python3
"""Unit tests for shape-to-slot selection."""

import unittest

from slot_selector import SLOT_INDEX, SLOT_NAMES, select_slot


def camera(states=None, *, status="STABLE", aligned=True):
    if states is None:
        states = {slot: "EMPTY" for slot in SLOT_NAMES}
    return {
        "status": status,
        "pose": {"aligned": aligned},
        "stable_states": states,
    }


class SlotSelectorTests(unittest.TestCase):
    def test_slot_numbering(self):
        self.assertEqual(
            SLOT_INDEX,
            {
                "C1_L1": 1, "C2_L1": 2, "C3_L1": 3,
                "C1_L2": 4, "C2_L2": 5, "C3_L2": 6,
                "C1_L3": 7, "C2_L3": 8, "C3_L3": 9,
            },
        )

    def test_circle_selects_lowest_empty_in_c1(self):
        result = select_slot(1, camera())
        self.assertTrue(result.accepted)
        self.assertEqual((result.column, result.slot_name, result.slot_index), ("C1", "C1_L1", 1))

    def test_square_skips_occupied_lower_slot(self):
        states = {slot: "EMPTY" for slot in SLOT_NAMES}
        states["C2_L1"] = "OCCUPIED"
        result = select_slot(2, camera(states))
        self.assertTrue(result.accepted)
        self.assertEqual((result.slot_name, result.slot_index), ("C2_L2", 5))

    def test_triangle_selects_top_after_two_occupied(self):
        states = {slot: "EMPTY" for slot in SLOT_NAMES}
        states["C3_L1"] = "OCCUPIED"
        states["C3_L2"] = "OCCUPIED"
        result = select_slot(3, camera(states))
        self.assertTrue(result.accepted)
        self.assertEqual((result.slot_name, result.slot_index), ("C3_L3", 9))

    def test_full_column_is_blocked(self):
        states = {slot: "EMPTY" for slot in SLOT_NAMES}
        for level in ("L1", "L2", "L3"):
            states[f"C1_{level}"] = "OCCUPIED"
        result = select_slot(1, camera(states))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "COLUMN_FULL")

    def test_camera_fault_is_blocked(self):
        result = select_slot(1, camera(status="CAMERA_FAULT", aligned=False))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "CAMERA_NOT_STABLE:CAMERA_FAULT")

    def test_unknown_anywhere_is_blocked(self):
        states = {slot: "EMPTY" for slot in SLOT_NAMES}
        states["C3_L3"] = "UNKNOWN"
        result = select_slot(1, camera(states))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "UNKNOWN_OCCUPANCY")

    def test_none_does_not_select(self):
        result = select_slot(0, camera())
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "NONE")


if __name__ == "__main__":
    unittest.main()
