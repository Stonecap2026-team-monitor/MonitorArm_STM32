"""Host regressions; no serial connection or motor movement."""
import copy
import json
import struct
from pathlib import Path
import tempfile
import unittest

from three_stage_test import Sequence, frames, load_plan, packet


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.plan = load_plan(Path(__file__).with_name("three_stage_joint_test.json"))
        self.sent = []
        self.seq = Sequence(self.plan, self.sent.append, lambda *args: None)

    def test_commands_from_each_idle_state(self):
        for state in ("unprepared", "home", "top", "angle_limit"):
            for command in ("prepare", "run", "down"):
                with self.subTest(state=state, command=command):
                    seq = Sequence(self.plan, lambda data: None, lambda *args: None)
                    seq.state = state
                    seq.command(command, 0)
                    self.assertTrue(seq.pending is not None or seq.waiting is not None)

    def test_busy_discards_new_command(self):
        self.seq.command("run", 0)
        self.seq.command("prepare", 1)
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.seq.state, "run")

    def stage_prepare(self):
        self.seq.command("prepare", 0)
        self.assertEqual(self.sent, [])
        for i, angle in enumerate((-50, -30, 0)):
            now = i * 2 + 0.1
            self.seq.receive(0x84, struct.pack(">h", 100 + i), now)
            self.seq.poll(now)
            self.assertEqual(list(frames(bytearray(self.sent[-1]))),
                             [(0x1A, struct.pack(">hhh", 100 + i, angle * 10, 0))])
            self.seq.receive(0x82, b"\x1a", i * 2 + 1)

    def test_prepare_stages_fresh_angles_then_waits_three_seconds(self):
        self.stage_prepare()
        self.seq.command("run", 6)
        self.seq.poll(7.999)
        self.assertEqual(len(self.sent), 3)
        self.seq.poll(8)
        self.assertEqual(list(frames(bytearray(self.sent[-1]))),
                         [(0x1B, Sequence.angles([0, 0, 0]))])
        self.seq.receive(0x82, b"\x1b", 9)
        self.assertEqual(self.seq.state, "home")
        self.assertIsNone(self.seq.pending)

    def test_error_at_wait_deadline_prevents_final_move(self):
        self.stage_prepare()
        self.seq.receive_batch([(0x83, b"\x0d")], 8)
        self.seq.poll(8)
        self.assertEqual(len(self.sent), 3)
        self.assertEqual(self.seq.state, "blocked")

    def test_prepare_requires_new_measurement(self):
        self.seq.receive(0x84, struct.pack(">h", 100), 0)
        self.seq.command("prepare", 1)
        self.seq.poll(1)
        self.assertEqual(self.sent, [])
        self.seq.poll(5)
        self.assertEqual(self.seq.state, "blocked")

    def test_prepare_rejects_out_of_range_feedback(self):
        self.seq.command("prepare", 0)
        self.seq.receive(0x84, struct.pack(">h", 870), 0.1)
        self.seq.poll(0.1)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.seq.state, "blocked")

    def test_prepare_rejects_nonzero_start(self):
        self.seq.plan["start"] = [20, -90, 70]
        self.seq.command("prepare", 0)
        self.assertEqual(self.sent, [])
        self.assertIsNone(self.seq.waiting)

    def test_three_targets_complete_in_order(self):
        self.seq.command("run", 0)
        for i, msg in enumerate((0x1A, 0x1A, 0x1B)):
            self.assertEqual(list(frames(bytearray(self.sent[-1]))),
                             [(msg, Sequence.angles(self.plan["up"][i]))])
            self.seq.receive(0x82, bytes([msg]), i + 1)
        self.assertEqual(self.seq.state, "top")
        self.assertEqual(len(self.sent), 3)

    def test_limit_cancels_sequence_until_new_command(self):
        self.seq.command("run", 0)
        self.seq.receive(0x83, b"\x03", 1)
        self.assertEqual(self.seq.steps, [])
        self.assertIsNone(self.seq.pending)
        self.assertEqual(len(self.sent), 1)
        self.seq.command("prepare", 2)
        self.seq.receive(0x84, struct.pack(">h", 100), 2.1)
        self.seq.poll(2.1)
        self.assertEqual(len(self.sent), 2)

    def test_hard_error_stays_blocked_even_if_limit_arrives_later(self):
        for error in (b"\x0c", b"\x0d", b"\x0f"):
            with self.subTest(error=error):
                self.setUp()
                self.seq.command("run", 0)
                self.seq.receive(0x83, error, 1)
                self.seq.receive(0x83, b"\x03", 2)
                self.seq.command("prepare", 3)
                self.assertEqual(self.seq.state, "blocked")
                self.assertEqual(len(self.sent), 1)
                self.seq.command("r", 4)
                self.assertEqual(self.seq.pending, 0x14)

    def test_batch_error_wins_over_done_and_limit(self):
        for errors in (((0x83, b"\x03"), (0x83, b"\x0d")),
                       ((0x83, b"\x0d"), (0x83, b"\x03"))):
            self.setUp()
            self.seq.command("run", 0)
            self.seq.receive_batch([(0x82, b"\x1a"), *errors], 1)
            self.assertEqual(self.seq.state, "blocked")
            self.assertEqual(len(self.sent), 1)

    def test_timeout_cannot_be_unblocked_by_late_limit(self):
        self.seq.command("run", 0)
        self.seq.receive(0x83, b"\x03", 181)
        self.assertEqual(self.seq.state, "blocked")

    def test_late_done_does_not_advance(self):
        self.seq.command("run", 0)
        self.seq.receive(0x82, b"\x1a", 181)
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.seq.state, "blocked")

    def test_plan_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            for angle, valid in ((-45, True), (80, True), (-45.1, False),
                                 (80.1, False), (90, False)):
                plan = copy.deepcopy(self.plan)
                plan["up"][-1][0] = angle
                path.write_text(json.dumps(plan), encoding="utf-8")
                if valid:
                    load_plan(path)
                else:
                    with self.assertRaises(ValueError):
                        load_plan(path)

    def test_partial_frame(self):
        frame = packet(0x83, b"\x03")
        buffer = bytearray(frame[:3])
        self.assertEqual(list(frames(buffer)), [])
        buffer.extend(frame[3:])
        self.assertEqual(list(frames(buffer)), [(0x83, b"\x03")])


if __name__ == "__main__":
    unittest.main()
