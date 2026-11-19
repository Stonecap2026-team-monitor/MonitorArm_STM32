"""Capture AS5600 diagnostic frames (0x87) from the STM32 UART to CSV.

Install: python -m pip install pyserial
List COM ports: python encoder_csv_logger.py --list
Capture: python encoder_csv_logger.py --port COM9 --seconds 30
Do not run the backend or another serial monitor on the same COM port.
"""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import struct
import time

import serial
from serial.tools import list_ports

START = 0xAA
END = 0x55
DIAG = 0x87


def frames_from_stream(port, seconds):
    """Read fixed framed binary messages, ignore non-diagnostic messages."""
    end_time = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < end_time:
        buf.extend(port.read(256))
        while True:
            start = buf.find(bytes([START]))
            if start < 0:
                buf.clear()
                break
            if start:
                del buf[:start]
            if len(buf) < 3:
                break
            length = buf[2]
            if length > 16:
                del buf[0]
                continue
            total = length + 5
            if len(buf) < total:
                break
            part = bytes(buf[:total])
            checksum = part[1] ^ part[2]
            for b in part[3:3 + length]:
                checksum ^= b
            if part[-1] != END or checksum != part[-2]:
                del buf[0]
                continue
            del buf[:total]
            yield part[1], part[3:-2]


def main():
    p = argparse.ArgumentParser(description='Log STM32 encoder diagnostic data as CSV')
    p.add_argument('--list', action='store_true', help='List available serial ports')
    p.add_argument('--port', help='COM port, e.g. COM9')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--output', help='CSV filename (default: timestamped)')
    a = p.parse_args()
    if a.list:
        for port in list_ports.comports():
            print(f'{port.device}: {port.description}')
        return
    if not a.port:
        p.error('--port is required (or use --list)')
    if a.seconds <= 0:
        p.error('--seconds must be > 0')
    path = Path(a.output or f'encoder_stationary_{datetime.now():%Y%m%d_%H%M%S}.csv')
    path.parent.mkdir(parents=True, exist_ok=True)
    print('Keep the motor stationary. Do not start the backend or another serial program.')
    count = 0
    first_seq = None
    last_seq = None
    start = time.monotonic()
    try:
        with serial.Serial(a.port, a.baud, timeout=0.05) as ser, path.open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['pc_time_iso','elapsed_s','sample_sequence','sample_gap',
                             'raw_angle_12bit','original_angle_deg','moving_avg3_deg',
                             'filter_ready','i2c_error_count'])
            for msg_id, payload in frames_from_stream(ser, a.seconds):
                if msg_id != DIAG or len(payload) != 13:
                    continue
                seq, raw, orig, avg, ready, errors = struct.unpack('>IHhhBH', payload)
                if first_seq is None:
                    first_seq = seq
                gap = 0 if last_seq is None else max(0, seq - last_seq - 1)
                last_seq = seq
                count += 1
                writer.writerow([datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds'),
                                 f'{time.monotonic() - start:.3f}', seq, gap, raw,
                                 f'{orig / 100:.2f}', f'{avg / 100:.2f}', ready, errors])
                if count % 50 == 0:
                    print(f'{count} samples; last angle={orig/100:.2f}°, avg={avg/100:.2f}°, missed={gap}')
    except (serial.SerialException, PermissionError) as exc:
        print(f'Serial access failed: {exc}')
        raise SystemExit(1)
    print(f'Saved: {path.resolve()} | samples: {count}')
    if count == 0:
        print('No diagnostic frames received. Verify modified communicationTask.c is flashed,')
        print('encoderMovingAverageReady is 1, USART2 port is selected, and backend is closed.')


if __name__ == '__main__':
    main()
