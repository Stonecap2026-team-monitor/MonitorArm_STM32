"""Run the existing serial test menu and log its UART receive stream to CSV."""

import argparse
from collections import deque
import csv
from datetime import datetime
from pathlib import Path
import struct
import threading
import time
from unittest.mock import patch

import serialtest_quiet as protocol


# Temporary measurement logger. Remove this standalone file after testing.
# The serial menu retains sole ownership of the port and all transmitted frames.
# BEGIN TEMP MOTION TEST: targets use the zero offsets already applied by STM32.
TEST_TARGETS = {
    "sim": (20.0, -40.0, 30.0),
    "zero": (0.0, 0.0, 0.0),
}
# END TEMP MOTION TEST
COLUMNS = (
    "elapsed_s", "event", "msg_id", "command_id", "command_name",
    "theta1_deg", "error_code", "data_hex",
    "window_count", "window_min_deg", "window_max_deg",
    "window_range_deg", "window_mean_deg", "window_span_s",
    "target_theta1_deg", "target_theta2_deg", "target_theta3_deg",
)


class MotionLog:
    def __init__(self, writer):
        self.writer = writer
        self.start = time.monotonic()
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.active = True
        self.angle_count = 0
        self.angle_window = deque(maxlen=10)
        self.target_done = threading.Event()
        self.test_error = threading.Event()
        self.target_pending = False
        self.trial_count = 0

    def observe(self, chunk):
        received_at = time.monotonic()
        with self.lock:
            if not self.active:
                return
            self.buffer.extend(chunk)
            while self.buffer:
                start = self.buffer.find(bytes([protocol.START]))
                if start < 0:
                    self.buffer.clear()
                    return
                del self.buffer[:start]
                if len(self.buffer) < 3:
                    return
                length = self.buffer[2]
                if length > protocol.MAX_DATA_LENGTH:
                    del self.buffer[0]
                    continue
                frame_size = length + 5
                if len(self.buffer) < frame_size:
                    return
                frame = self.buffer[:frame_size]
                msg_id, payload = frame[1], frame[3:-2]
                if (frame[-1] != protocol.END
                        or frame[-2] != protocol.checksum(msg_id, payload)):
                    del self.buffer[0]
                    continue
                del self.buffer[:frame_size]
                self.record(received_at, msg_id, payload)

    def record(self, received_at, msg_id, payload):
        command_id = command_name = theta1 = error_code = ""
        window_values = ("",) * 6
        if msg_id == protocol.MSG_CURRENT_ANGLE and len(payload) == 2:
            event = "theta1_sample"
            angle = struct.unpack(">h", payload)[0] / 10
            theta1 = f"{angle:.1f}"
            self.angle_count += 1
            # Each new sample replaces the oldest one after the window fills.
            self.angle_window.append((received_at, angle))
            window_values = (len(self.angle_window), "", "", "", "", "")
            if len(self.angle_window) == 10:
                angles = [sample[1] for sample in self.angle_window]
                minimum, maximum = min(angles), max(angles)
                window_values = (
                    10, f"{minimum:.1f}", f"{maximum:.1f}",
                    f"{maximum - minimum:.1f}", f"{sum(angles) / 10:.3f}",
                    f"{received_at - self.angle_window[0][0]:.6f}",
                )
        elif msg_id in (protocol.MSG_ACK, protocol.MSG_COMMAND_DONE) and len(payload) == 1:
            command = payload[0]
            if command == protocol.MSG_HEARTBEAT:
                return
            command_id = f"0x{command:02X}"
            command_name = protocol.COMMAND_NAMES.get(command, "")
            event = "command_ack" if msg_id == protocol.MSG_ACK else "command_done"
            if (msg_id == protocol.MSG_COMMAND_DONE
                    and command == protocol.MSG_SET_TARGET and self.target_pending):
                self.target_pending = False
                self.target_done.set()
            if msg_id == protocol.MSG_COMMAND_DONE and command == protocol.MSG_CLEAR_ERROR:
                self.target_pending = False
        elif msg_id == protocol.MSG_ERROR and len(payload) == 1:
            event = "motor_error"
            error_code = f"0x{payload[0]:02X}"
            self.target_pending = False
            self.test_error.set()
        elif msg_id == protocol.MSG_READY and not payload:
            event = "stm32_ready"
            self.angle_window.clear()
        else:
            return
        self.writer.writerow((
            f"{received_at - self.start:.6f}", event, f"0x{msg_id:02X}",
            command_id, command_name, theta1, error_code, payload.hex(" ").upper(),
            *window_values,
            "", "", "",
        ))

    def mark(self, event, **values):
        with self.lock:
            if not self.active:
                raise OSError("CSV logging is inactive")
            row = dict.fromkeys(COLUMNS, "")
            row.update(elapsed_s=f"{time.monotonic() - self.start:.6f}",
                       event=event, **values)
            self.writer.writerow([row[column] for column in COLUMNS])

    def stop(self):
        with self.lock:
            self.active = False
            self.test_error.set()


# BEGIN TEMP MOTION TEST: uses only existing SET_TARGET and error replies.
def wait_for_test(ser, motion_log, seconds, require_done=False):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if motion_log.test_error.is_set() or not ser.is_open:
            raise RuntimeError("Test stopped after an error or serial disconnect")
        if require_done and motion_log.target_done.is_set():
            return
        time.sleep(0.05)
    if require_done:
        raise RuntimeError("No SET_TARGET completion within 90 seconds")


def run_motion_test(ser, motion_log):
    motion_log.trial_count += 1
    trial = motion_log.trial_count
    try:
        if motion_log.target_pending:
            raise RuntimeError("Wait for the previous target command to finish")
        motion_log.mark(f"test_{trial}_start")
        print(f"Test {trial}: recording initial position for 5 seconds")
        motion_log.test_error.clear()
        wait_for_test(ser, motion_log, 5)
        for name, hold_seconds in (("sim", 5), ("zero", 30)):
            motion_log.mark(f"test_{trial}_{name}")
            print(f"Test {trial}: {name} {TEST_TARGETS[name]}")
            protocol.send_set_target(ser, *TEST_TARGETS[name])
            wait_for_test(ser, motion_log, 90, require_done=True)
            print(f"{name} complete. Recording for {hold_seconds} seconds")
            wait_for_test(ser, motion_log, hold_seconds)
        motion_log.mark(f"test_{trial}_complete")
        print("Test complete. Check debugTheta1PositionErrorDeg and")
        print("debugTheta1CorrectionCount in the debugger.")
        print("Enter test to repeat, or q to finish the CSV.")
    except (RuntimeError, OSError, csv.Error, protocol.serial.SerialException) as exc:
        print(f"Measurement interrupted: {exc}")
        print("Check the reported error before starting another test.")
# END TEMP MOTION TEST


class ObservedReceiver:
    def __init__(self, port, motion_log):
        self.port = port
        self.motion_log = motion_log

    @property
    def is_open(self):
        return self.port.is_open

    def read(self, size):
        chunk = self.port.read(size)
        if chunk:
            try:
                self.motion_log.observe(chunk)
            except (OSError, csv.Error) as exc:
                self.motion_log.stop()
                print(f"CSV logging stopped: {exc}")
        return chunk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=protocol.PORT, help="Serial port")
    parser.add_argument("--output", type=Path, help="New CSV path")
    args = parser.parse_args()
    path = args.output or Path(f"encoder_motion_{datetime.now():%Y%m%d_%H%M%S}.csv")
    receive_thread = protocol.receive_thread
    send_set_target = protocol.send_set_target
    try:
        output = path.open("x", newline="", encoding="utf-8-sig", buffering=1)
    except OSError as exc:
        parser.exit(1, f"Cannot create CSV: {exc}\n")

    with output:
        writer = csv.writer(output)
        writer.writerow(COLUMNS)
        motion_log = MotionLog(writer)

        def receive_and_log(ser):
            receive_thread(ObservedReceiver(ser, motion_log))

        def send_and_log_target(ser, theta1, theta2, theta3):
            # Record the requested angles so sim and zero can be identified.
            motion_log.mark(
                "target_sent", command_id="0x10", command_name="SET_TARGET",
                target_theta1_deg=theta1, target_theta2_deg=theta2,
                target_theta3_deg=theta3,
            )
            motion_log.target_done.clear()
            motion_log.target_pending = True
            send_set_target(ser, theta1, theta2, theta3)

        print(f"Recording to {path.resolve()}")
        print("test: 5s baseline -> sim -> 5s hold -> zero -> 30s hold")
        print("If SYSTEM_LOCKED is reported, use r and wait for CLEAR_ERROR completion.")
        print("x/c: heartbeat off/on for a stationary communication-loss check.")
        print("Use the physical emergency-stop button for the emergency-stop check.")
        try:
            with (patch.object(protocol, "receive_thread", receive_and_log),
                  patch.object(protocol, "send_set_target", send_and_log_target),
                  patch.object(protocol, "PORT", args.port)):
                protocol.main(
                    command_targets=TEST_TARGETS,
                    command_handlers={"test": lambda ser: run_motion_test(ser, motion_log)},
                )
        finally:
            motion_log.stop()
            print(f"Saved {motion_log.angle_count} angle samples to {path.resolve()}")


if __name__ == "__main__":
    main()
