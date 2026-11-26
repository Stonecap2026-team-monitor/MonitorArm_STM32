"""Manual three-stage protocol test. Joint-space fixture, NOT an IK arc planner."""
import argparse
import csv
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time
from datetime import datetime
def packet(msg, data=b""):
    checksum = msg ^ len(data)
    for byte in data:
        checksum ^= byte
    return bytes((0xAA, msg, len(data))) + data + bytes((checksum, 0x55))

def heartbeat(counter):
    return bytes((0xAA, 0x01, 0x01, counter, counter, 0x55))

def frames(buffer):
    while buffer:
        start = buffer.find(b"\xaa")
        if start < 0:
            buffer.clear()
            return
        del buffer[:start]
        if len(buffer) < 3:
            return
        size = buffer[2]
        if size > 16:
            del buffer[0]
            continue
        if len(buffer) < size + 5:
            return
        packet = bytes(buffer[:size + 5])
        checksum = 0
        for value in packet[1:-2]:
            checksum ^= value
        if packet[-1] != 0x55 or packet[-2] != checksum:
            del buffer[0]
            continue
        del buffer[:size + 5]
        yield packet[1], packet[3:-2]


def load_plan(path):
    plan = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if len(plan["up"]) != 3 or len(plan["down"]) != 3:
        raise ValueError("Exactly three targets are required in each direction")
    for row in [plan["start"], *plan["up"], *plan["down"]]:
        if len(row) != 3 or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in row):
            raise ValueError("Each target must contain three finite angles")
        # Match motorTask joint limits (servo driver additionally validates calibration).
        for value, bounds in zip(row, [(-45, 80), (-105, 105), (-105, 105)]):
            if not bounds[0] <= value <= bounds[1]:
                raise ValueError(f"Target out of range: {row}")
    if plan["down"][-1] != plan["start"]:
        raise ValueError("Last down target must equal start")
    return plan


class Sequence:
    """One outstanding command. ERROR/timeout cancels remaining targets."""
    def __init__(self, plan, send, log, timeout=180):
        self.plan, self.send, self.log, self.timeout = plan, send, log, timeout
        self.state, self.pending, self.steps = "unprepared", None, []
        self.after = None
        self.index = 0
        self.waiting = None
        self.angle_sample = None
        self.angle_count = 0

    def fail(self, reason):
        self.log("blocked", reason)
        self.state, self.pending, self.steps = "blocked", None, []
        self.waiting = None

    def command(self, name, now):
        if self.pending is not None or self.waiting is not None:
            self.log("rejected", "Busy; command discarded")
            return
        if self.state == "blocked" and name != "r":
            self.log("rejected", "Sequence blocked; inspect the error, then use r to clear")
            return
        if name == "r":
            steps, after = [(0x14, b"")], "unprepared"
        elif name == "prepare":
            if self.plan["start"] != [0, 0, 0]:
                self.log("rejected", "Staged prepare requires JSON start [0, 0, 0]")
                return
            steps = [("servo", angle) for angle in (-50, -30, 0)]
            steps += [("wait", 3.0), (0x1B, self.angles(self.plan["start"]))]
            after = "home"
        elif name == "run":
            steps, after = self.path("up"), "top"
        elif name == "down":
            steps, after = self.path("down"), "home"
        else:
            self.log("rejected", f"Cannot {name} from {self.state}; use r, prepare, run, down, q")
            return
        self.steps, self.after, self.state, self.index = steps, after, name, 0
        self.advance(now)

    @staticmethod
    def angles(row):
        return struct.pack(">hhh", *(int(round(x * 10)) for x in row))

    def path(self, direction):
        return [(0x1B if i == 2 else 0x1A, self.angles(row))
                for i, row in enumerate(self.plan[direction])]

    def advance(self, now):
        if not self.steps:
            self.pending = None
            self.state = self.after
            self.log("sequence_done", self.state)
            return
        msg, data = self.steps.pop(0)
        if msg in ("servo", "wait"):
            self.pending = None
            self.waiting = (msg, data, now + (4.0 if msg == "servo" else data), self.angle_count)
            self.log("waiting", "Fresh theta1 sample" if msg == "servo" else "Settling for 3s")
            return
        self.send_move(msg, data, now)

    def send_move(self, msg, data, now):
        self.index += 1
        self.pending, self.deadline = msg, now + self.timeout
        self.log("request", f"phase={self.state};segment={self.index};id={msg:02x};data={data.hex()}")
        self.send(packet(msg, data))

    def receive(self, msg, data, now):
        # A late DONE must never start another motor command.
        self.poll(now, advance_wait=False)
        if msg == 0x83:
            was_blocked = self.state == "blocked"
            self.fail(f"STM ERROR {data.hex()}; no automatic retry")
            if data == b"\x03" and not was_blocked:
                self.state = "angle_limit"
                self.log("recovery", "Angle limit: enter a new command; STM permits a lower theta1 target within limits")
        elif msg == 0x84 and len(data) == 2:
            self.angle_sample = (struct.unpack(">h", data)[0], now)
            self.angle_count += 1
        elif msg == 0x82:
            if self.pending is not None and data == bytes([self.pending]):
                self.log("done", f"phase={self.state};segment={self.index};id={data.hex()}")
                self.advance(now)
            else:
                self.fail(f"Unexpected DONE {data.hex()}; sequence uncertain")

    def poll(self, now, advance_wait=True):
        if self.pending is not None and now >= self.deadline:
            self.fail("Completion timeout; arrival unknown")
        if self.waiting is None:
            return
        kind, value, deadline, first_sample = self.waiting
        if kind == "servo" and now >= deadline:
            self.fail("No fresh theta1 sample for prepare within 4s")
            return
        if not advance_wait:
            return
        if kind == "wait" and now >= deadline:
            self.waiting = None
            self.advance(now)
        elif kind == "servo" and self.angle_count > first_sample:
            theta1, sample_time = self.angle_sample
            if now - sample_time > 0.5:
                return
            if not -450 <= theta1 <= 800:
                self.fail("Measured theta1 outside command limits; prepare cancelled")
                return
            self.waiting = None
            # Intermediate move avoids final correction during servo staging.
            # STM still checks feedback and may issue small STEP movements.
            self.send_move(0x1A, struct.pack(">hhh", theta1, value * 10, 0), now)

    def receive_batch(self, received, now):
        # ERROR wins even when DONE arrived in the same read.
        errors = [(msg, data) for msg, data in received if msg == 0x83]
        if errors:
            # A simultaneous sensor/link/ESTOP error still requires r.
            error = next((item for item in errors if item[1] != b"\x03"), errors[0])
            self.receive(*error, now)
        elif sum(msg == 0x82 for msg, _ in received) > 1:
            # Unsent commands cannot already have completions in this buffer.
            self.fail("Multiple buffered DONEs; sequence uncertain")
        else:
            for msg, data in received:
                self.receive(msg, data, now)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--plan", default=str(Path(__file__).with_name("three_stage_joint_test.json")))
    parser.add_argument("--preview", action="store_true", help="Print targets without opening serial")
    args = parser.parse_args()
    plan = load_plan(args.plan)
    print(plan.get("description", "Joint target plan"))
    print("Intermediate motion is not guaranteed circular; no camera/gaze control in this test.")
    print(json.dumps(plan, indent=2))
    if args.preview:
        return
    import serial
    commands = queue.Queue()

    def console():
        while True:
            try:
                value = input().strip().lower()
            except EOFError:
                value = "q"
            commands.put(value)
            if value == "q":
                return

    output = Path(f"three_stage_{datetime.now():%Y%m%d_%H%M%S}.csv")
    with serial.Serial(args.port, 115200, timeout=.02, write_timeout=1) as port, output.open("x", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("host_elapsed_s", "event", "detail"))
        started = time.monotonic()

        def log(event, detail):
            writer.writerow((f"{time.monotonic()-started:.6f}", event, detail))
            stream.flush()
            if event != "rx":
                print(event, detail)

        seq = Sequence(plan, port.write, log)
        buffer, next_hb, counter = bytearray(), 0, 0
        print("Choose prepare, run, or down in any order; wait for completion before the next command.")
        print("run/down always visit the JSON targets in order, starting from the current position.")
        print(f"prepare moves ALL axes to {plan['start']}; it does not set STM HOME.")
        print("prepare: measured theta1 + theta2 -50/-30/0, theta3 0; wait 3s, then (0,0,0).")
        print("STEP may move slightly during servo staging; STM feedback protection remains active.")
        print("q saves/exits; it is NOT an immediate motor stop. Use hardware ESTOP when needed.")
        print("On error: inspect before clearing; do not automatically resume the sequence.")
        print("CSV:", output.resolve())
        threading.Thread(target=console, daemon=True).start()
        try:
            while True:
                now = time.monotonic()
                if now >= next_hb:
                    port.write(heartbeat(counter))
                    counter = (counter+1) % 256
                    next_hb = now+1
                buffer.extend(port.read(port.in_waiting or 1))
                received = list(frames(buffer))
                for msg, data in received:
                    log("rx", f"id={msg:02x};data={data.hex()}")
                seq.receive_batch(received, time.monotonic())
                seq.poll(time.monotonic())
                try:
                    command = commands.get_nowait()
                except queue.Empty:
                    continue
                if command == "q":
                    break
                seq.command(command, time.monotonic())
        except KeyboardInterrupt:
            log("exit", "Keyboard interrupt; arrival may be unknown")
    print(f"Saved {output}; heartbeat stopped, STM will enter its communication timeout protection.")


if __name__ == "__main__":
    main()
