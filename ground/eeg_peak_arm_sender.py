#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read 4 useful ADS1299/OpenBCI-compatible EEG channels and trigger SO101 motion.

Data input:
    ESP32 firmware stream format:
    [0xA0][sample_id][8 * int24 channel data][6 aux bytes][0xC0]

Trigger rule:
    CH1 / CH2 / CH7 / CH8, any channel peak relative to its rolling baseline
    exceeds threshold_uv, then send:
        MODE,EEG
        EEG,MOVE_FWD_DOWN

The board-side serial_command_bridge forwards EEG,MOVE_FWD_DOWN to /eeg/command.
The board-side so101_arm_interface executes +X 10 cm and -Z 10 cm by default.
"""

from __future__ import annotations

import argparse
import collections
import statistics
import time
import zlib


FRAME_LEN = 33
HEADER = 0xA0
FOOTER = 0xC0
DEFAULT_REAL_CHANNELS = (1, 2, 7, 8)

# OpenBCI/ADS1299 default scale with Vref=4.5V and gain=24.
# 4.5 / 24 / (2^23 - 1) * 1e6 = 0.0223517 uV/count.
DEFAULT_UV_PER_COUNT = 0.022351744455307063


def encode_frame(body: str) -> bytes:
    checksum = zlib.crc32(body.encode("ascii")) & 0xFFFFFFFF
    return f"{body}*{checksum:08X}\n".encode("ascii")


def int24_be(b0: int, b1: int, b2: int) -> int:
    value = (b0 << 16) | (b1 << 8) | b2
    if value & 0x800000:
        value |= -1 << 24
    return value


def parse_frames(buffer: bytearray):
    frames = []
    i = 0
    while i <= len(buffer) - FRAME_LEN:
        if buffer[i] != HEADER:
            i += 1
            continue
        if buffer[i + FRAME_LEN - 1] != FOOTER:
            i += 1
            continue

        sample_id = buffer[i + 1]
        channels = []
        for ch in range(8):
            base = i + 2 + ch * 3
            channels.append(int24_be(buffer[base], buffer[base + 1], buffer[base + 2]))
        frames.append((sample_id, channels))
        i += FRAME_LEN

    if i:
        del buffer[:i]
    return frames


def parse_channels(text: str) -> tuple[int, ...]:
    channels = []
    for part in text.split(","):
        ch = int(part.strip())
        if ch < 1 or ch > 8:
            raise ValueError("channels must be in 1..8")
        channels.append(ch)
    return tuple(channels)


def open_serial(port: str, baud: int, timeout: float):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请先执行: python -m pip install pyserial") from exc
    return serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=0.5)


def send_radio_command(radio, body: str, dry_run: bool):
    print(f"RADIO -> {body}")
    if dry_run:
        return
    radio.write(encode_frame(body))
    radio.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="EEG peak threshold to SO101 movement sender")
    parser.add_argument("--eeg-port", default="COM7", help="脑电板串口，例如 COM7")
    parser.add_argument("--eeg-baud", type=int, default=115200)
    parser.add_argument("--radio-port", required=True, help="地面数传串口，例如 COM8/COM9，不能和脑电串口相同")
    parser.add_argument("--radio-baud", type=int, default=57600)
    parser.add_argument("--channels", default="1,2,7,8", help="要检测的 OpenBCI 通道，默认 1,2,7,8")
    parser.add_argument("--threshold-uv", type=float, default=10.0, help="峰值阈值，单位 uV")
    parser.add_argument("--uv-per-count", type=float, default=DEFAULT_UV_PER_COUNT)
    parser.add_argument("--baseline-window", type=int, default=125, help="滚动基线窗口样本数，250SPS 下 125≈0.5s")
    parser.add_argument("--warmup-sec", type=float, default=1.0, help="启动后先采集基线，不触发")
    parser.add_argument("--cooldown-sec", type=float, default=2.0, help="触发后的冷却时间")
    parser.add_argument("--dry-run", action="store_true", help="只打印，不发送数传")
    parser.add_argument("--no-hold-on-exit", action="store_true", help="退出时不发送 MODE,HOLD")
    args = parser.parse_args()

    channels = parse_channels(args.channels)
    channel_indexes = [ch - 1 for ch in channels]
    if args.eeg_port.upper() == args.radio_port.upper():
        raise ValueError("脑电串口和数传串口不能相同")

    eeg = open_serial(args.eeg_port, args.eeg_baud, timeout=0.02)
    radio = None if args.dry_run else open_serial(args.radio_port, args.radio_baud, timeout=0)

    history = {idx: collections.deque(maxlen=max(5, args.baseline_window)) for idx in channel_indexes}
    buffer = bytearray()
    last_trigger_time = 0.0
    start_time = time.monotonic()
    frame_count = 0

    try:
        time.sleep(0.5)
        eeg.reset_input_buffer()
        try:
            eeg.write(b"s")
            time.sleep(0.2)
            eeg.reset_input_buffer()
            eeg.write(b"b")
        except Exception:
            pass

        print(
            f"EEG ready: eeg={args.eeg_port}@{args.eeg_baud}, "
            f"radio={args.radio_port}@{args.radio_baud}, channels={channels}, "
            f"threshold={args.threshold_uv}uV"
        )
        print("Press Ctrl+C to stop.")

        while True:
            data = eeg.read(4096)
            if data:
                buffer.extend(data)

            for _sample_id, raw_channels in parse_frames(buffer):
                frame_count += 1
                now = time.monotonic()

                peaks = {}
                for idx in channel_indexes:
                    value_uv = raw_channels[idx] * args.uv_per_count
                    hist = history[idx]
                    baseline = statistics.fmean(hist) if hist else value_uv
                    peak = abs(value_uv - baseline)
                    peaks[idx + 1] = peak
                    hist.append(value_uv)

                if now - start_time < args.warmup_sec:
                    continue
                if now - last_trigger_time < args.cooldown_sec:
                    continue

                hit_channel, hit_peak = max(peaks.items(), key=lambda item: item[1])
                if hit_peak >= args.threshold_uv:
                    print(
                        f"TRIGGER ch{hit_channel} peak={hit_peak:.1f}uV "
                        f"frames={frame_count}"
                    )
                    send_radio_command(radio, "MODE,EEG", args.dry_run)
                    time.sleep(0.05)
                    send_radio_command(radio, "EEG,MOVE_FWD_DOWN", args.dry_run)
                    last_trigger_time = now

            if not data:
                time.sleep(0.005)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        try:
            eeg.write(b"s")
        except Exception:
            pass
        try:
            eeg.close()
        except Exception:
            pass
        if radio is not None:
            try:
                if not args.no_hold_on_exit:
                    send_radio_command(radio, "MODE,HOLD", False)
                radio.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
