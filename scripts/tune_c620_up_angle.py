#!/usr/bin/env python3
"""
Serial auto-tuner for firmware debug PID targets.

Firmware protocol on USART6, 115200:
  LIST
  SELECT name|idx
  MODE SPEED|POSITION
  PID akp aki akd skp ski skd [amax smax]
  GOTO output_deg
  SPEED rpm
  TARGET motor_deg
  ZERO
  STOP
  EXIT
  LOG 0|1 [divider]
  STATUS

Telemetry line:
  UPI,ms,target_cdeg,angle_cdeg,error_cdeg,rpm,torque,out,
      akp_milli,aki_milli,akd_milli,skp_milli,ski_milli,skd_milli,amax,smax
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import struct
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing dependency: pip install pyserial") from exc


GEAR_RATIO = 19.0


@dataclass(frozen=True)
class Pid:
    akp: float
    aki: float
    akd: float
    skp: float
    ski: float
    skd: float
    amax: int
    smax: int


@dataclass
class Sample:
    ms: int
    target: float
    angle: float
    error: float
    rpm: float
    torque: int
    out: int
    akp: float
    aki: float
    akd: float
    skp: float
    ski: float
    skd: float
    amax: int
    smax: int


def parse_upi(line: str) -> Sample | None:
    parts = line.strip().split(",")
    if parts[0] != "UPI":
        return None
    if len(parts) == 10:
        sample = Sample(
            ms=int(parts[1]),
            target=int(parts[2]) / 100.0,
            angle=int(parts[3]) / 100.0,
            error=int(parts[4]) / 100.0,
            rpm=float(int(parts[5])),
            torque=int(parts[6]),
            out=int(parts[7]),
            akp=0.0,
            aki=0.0,
            akd=0.0,
            skp=0.0,
            ski=0.0,
            skd=0.0,
            amax=int(parts[8]),
            smax=int(parts[9]),
        )
        if abs(sample.target) > 20000.0 or abs(sample.angle) > 20000.0 or abs(sample.error) > 20000.0:
            return None
        return sample
    if len(parts) != 16:
        return None
    return Sample(
        ms=int(parts[1]),
        target=int(parts[2]) / 100.0,
        angle=int(parts[3]) / 100.0,
        error=int(parts[4]) / 100.0,
        rpm=float(int(parts[5])),
        torque=int(parts[6]),
        out=int(parts[7]),
        akp=int(parts[8]) / 1000.0,
        aki=int(parts[9]) / 1000.0,
        akd=int(parts[10]) / 1000.0,
        skp=int(parts[11]) / 1000.0,
        ski=int(parts[12]) / 1000.0,
        skd=int(parts[13]) / 1000.0,
        amax=int(parts[14]),
        smax=int(parts[15]),
    )


class Link:
    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float = 0.08,
        char_delay: float = 0.001,
        boot_delay: float = 2.8,
        verbose: bool = False,
    ):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.char_delay = max(0.0, char_delay)
        self.verbose = verbose
        time.sleep(max(0.0, boot_delay))
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def send(self, command: str) -> None:
        data = (command.strip() + "\n").encode("ascii")
        if self.char_delay <= 0.0:
            self.ser.write(data)
            return
        for byte in data:
            self.ser.write(bytes([byte]))
            self.ser.flush()
            time.sleep(self.char_delay)

    def command(self, command: str, wait: float = 0.25) -> list[str]:
        self.ser.reset_input_buffer()
        self.send(command)
        end = time.monotonic() + wait
        lines: list[str] = []
        while time.monotonic() < end:
            raw = self.ser.readline()
            if raw:
                lines.append(raw.decode("ascii", errors="ignore").strip())
        return lines

    def command_expect(self, command: str, prefix: str, wait: float = 0.5, required: bool = False) -> list[str]:
        lines = self.command(command, wait=wait)
        if self.verbose:
            print(f"CMD {command!r} -> {lines[:12]}")
        if prefix == "OK LOG":
            ok = any(line == prefix or line.startswith(prefix + " ") for line in lines)
        else:
            ok = any(line == prefix for line in lines)
        if not ok:
            message = f"no {prefix} after {command!r}; got: {lines[:5]}"
            if required:
                raise RuntimeError(message)
            print(f"WARN {message}", file=sys.stderr)
        return lines

    def collect(self, seconds: float, vofa: "VofaForwarder | None" = None) -> list[Sample]:
        end = time.monotonic() + seconds
        samples: list[Sample] = []
        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            sample = parse_upi(line)
            if sample is not None:
                samples.append(sample)
                if vofa is not None:
                    vofa.send(sample)
        return samples


class VofaForwarder:
    def __init__(self, port: str | None, baud: int, mode: str, ratio: float):
        self.ser = None
        self.mode = mode
        self.ratio = ratio
        if port:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0)

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()

    def send(self, sample: Sample) -> None:
        if self.ser is None:
            return
        scale = self.ratio if self.mode == "position" else 1.0
        values = [
            sample.target / scale,
            sample.angle / scale,
            sample.error / scale,
            sample.rpm,
            float(sample.torque),
            float(sample.out),
            sample.akp,
            sample.akd,
            sample.skp,
            sample.skd,
        ]
        payload = struct.pack("<" + "f" * len(values), *values)
        self.ser.write(payload + b"\x00\x00\x80\x7f")


def apply_pid(link: Link, pid: Pid) -> None:
    link.command(
        f"PID {pid.akp:.5g} {pid.aki:.5g} {pid.akd:.5g} "
        f"{pid.skp:.5g} {pid.ski:.5g} {pid.skd:.5g} {pid.amax} {pid.smax}",
        wait=0.3,
    )


def score_position_step(samples: list[Sample], output_target_deg: float, settle_deg: float, ratio: float) -> dict[str, float]:
    if len(samples) < 5:
        return {"score": 1e9, "settle_ms": 1e9, "overshoot": 1e9, "steady": 1e9}

    target_motor = output_target_deg * ratio
    direction = 1.0 if target_motor >= samples[0].angle else -1.0
    target_output = target_motor / ratio
    angles_output = [s.angle / ratio for s in samples]
    errors_output = [(target_motor - s.angle) / ratio for s in samples]

    if direction > 0:
        overshoot = max(0.0, max(angles_output) - target_output)
    else:
        overshoot = max(0.0, target_output - min(angles_output))

    settle_ms = 1e9
    threshold = abs(settle_deg)
    for idx, sample in enumerate(samples):
        tail = errors_output[idx:]
        if tail and max(abs(e) for e in tail) <= threshold:
            settle_ms = sample.ms - samples[0].ms
            break

    last = samples[-max(3, len(samples) // 5) :]
    steady = sum(abs((target_motor - s.angle) / ratio) for s in last) / len(last)
    sat_ratio = sum(1 for s in samples if abs(s.out) >= 0.95 * max(1, s.smax)) / len(samples)
    ripple = 0.0
    if len(last) > 2:
        mean_angle = sum(s.angle / ratio for s in last) / len(last)
        ripple = math.sqrt(sum((s.angle / ratio - mean_angle) ** 2 for s in last) / len(last))

    score = (
        0.004 * settle_ms
        + 3.0 * overshoot
        + 6.0 * steady
        + 8.0 * ripple
        + 30.0 * sat_ratio
    )
    return {
        "score": score,
        "settle_ms": settle_ms,
        "overshoot": overshoot,
        "steady": steady,
        "ripple": ripple,
        "sat_ratio": sat_ratio,
    }


def score_speed_step(samples: list[Sample], target_rpm: float, settle_rpm: float) -> dict[str, float]:
    if len(samples) < 5:
        return {"score": 1e9, "settle_ms": 1e9, "overshoot": 1e9, "steady": 1e9}

    direction = 1.0 if target_rpm >= samples[0].angle else -1.0
    speeds = [s.angle for s in samples]
    errors = [target_rpm - s.angle for s in samples]

    if direction > 0:
        overshoot = max(0.0, max(speeds) - target_rpm)
    else:
        overshoot = max(0.0, target_rpm - min(speeds))

    settle_ms = 1e9
    threshold = abs(settle_rpm)
    for idx, sample in enumerate(samples):
        tail = errors[idx:]
        if tail and max(abs(e) for e in tail) <= threshold:
            settle_ms = sample.ms - samples[0].ms
            break

    last = samples[-max(3, len(samples) // 5) :]
    steady = sum(abs(target_rpm - s.angle) for s in last) / len(last)
    sat_ratio = sum(1 for s in samples if abs(s.out) >= 0.95 * max(1, s.smax)) / len(samples)
    ripple = 0.0
    if len(last) > 2:
        mean_speed = sum(s.angle for s in last) / len(last)
        ripple = math.sqrt(sum((s.angle - mean_speed) ** 2 for s in last) / len(last))

    score = (
        0.004 * settle_ms
        + 0.015 * overshoot
        + 0.05 * steady
        + 0.06 * ripple
        + 30.0 * sat_ratio
    )
    return {
        "score": score,
        "settle_ms": settle_ms,
        "overshoot": overshoot,
        "steady": steady,
        "ripple": ripple,
        "sat_ratio": sat_ratio,
    }


def run_trial(
    link: Link,
    vofa: VofaForwarder | None,
    pid: Pid,
    mode: str,
    target_value: float,
    ratio: float,
    duration: float,
    settle_threshold: float,
    csv_writer: csv.DictWriter | None,
    trial_index: int,
) -> dict[str, float]:
    link.command_expect("LOG 0", "OK LOG", wait=0.5)
    apply_pid(link, pid)
    if mode == "speed":
        link.command_expect("SPEED 0", "OKV", wait=0.5)
    else:
        link.command_expect("GOTO 0", "OKG", wait=0.5)
    time.sleep(0.25)
    link.ser.reset_input_buffer()

    if mode == "speed":
        link.command_expect(f"SPEED {target_value:.5g}", "OKV", wait=0.5)
    else:
        link.command_expect(f"GOTO {target_value:.5g}", "OKG", wait=0.5)
    link.ser.reset_input_buffer()
    link.command_expect("LOG 1 1", "OK LOG", wait=0.5)
    link.ser.reset_input_buffer()
    samples = link.collect(duration, vofa)
    if mode == "speed":
        metric = score_speed_step(samples, target_value, settle_threshold)
    else:
        metric = score_position_step(samples, target_value, settle_threshold, ratio)

    if csv_writer is not None:
        for s in samples:
            row = asdict(s)
            row["trial"] = trial_index
            row["score"] = metric["score"]
            csv_writer.writerow(row)

    link.command_expect("LOG 0", "OK LOG", wait=0.5)
    if mode == "speed":
        link.command_expect("SPEED 0", "OKV", wait=0.5)
    else:
        link.command_expect("GOTO 0", "OKG", wait=0.5)
    link.collect(max(0.35, duration * 0.35), vofa)
    return metric


def candidate_grid(base: Pid, rounds: int) -> Iterable[Pid]:
    akp_scale = [0.7, 1.0, 1.3]
    skp_scale = [0.7, 1.0, 1.3]
    skd_scale = [0.5, 1.0, 1.6]
    akd_values = sorted({0.0, base.akd, max(0.02, base.akd * 2.0)})

    for ak, sk, sd, ad in itertools.product(akp_scale, skp_scale, skd_scale, akd_values):
        yield Pid(
            akp=max(0.0, base.akp * ak),
            aki=base.aki,
            akd=ad,
            skp=max(0.0, base.skp * sk),
            ski=base.ski,
            skd=max(0.0, base.skd * sd),
            amax=base.amax,
            smax=base.smax,
        )

    best = base
    for _ in range(max(0, rounds - 1)):
        for ak, sk, sd in itertools.product([0.85, 1.0, 1.15], repeat=3):
            yield Pid(best.akp * ak, best.aki, best.akd, best.skp * sk, best.ski, best.skd * sd, best.amax, best.smax)


def main() -> int:
    parser = argparse.ArgumentParser(description="Tune selected firmware PID target over USART6.")
    parser.add_argument("port", help="Serial port, for example COM8 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--char-delay", type=float, default=0.001, help="Delay between command bytes for polling UART firmware")
    parser.add_argument("--boot-delay", type=float, default=2.8, help="Delay after opening the port before sending commands")
    parser.add_argument("--verbose", action="store_true", help="Print command acknowledgements")
    parser.add_argument("--select", help="Firmware tune target name or index, for example up or pitch_speed")
    parser.add_argument("--mode", choices=("position", "speed"), default="position", help="Tune position double-loop or speed loop")
    parser.add_argument("--target", type=float, help="Position mode: output shaft degrees. Speed mode: rpm")
    parser.add_argument("--angle", type=float, default=30.0, help="Backward-compatible position target in output shaft degrees")
    parser.add_argument("--duration", type=float, default=1.3, help="Seconds to record each step")
    parser.add_argument("--settle-deg", type=float, default=1.0, help="Settle threshold: degrees in position mode, rpm in speed mode")
    parser.add_argument("--ratio", type=float, default=GEAR_RATIO, help="Output-to-motor ratio used for position scoring and VOFA display")
    parser.add_argument("--rounds", type=int, default=1)
    parser.add_argument("--max-trials", type=int, default=0, help="0 means all generated candidates")
    parser.add_argument("--zero", action="store_true", help="Send ZERO before tuning")
    parser.add_argument("--csv", type=Path, default=Path("c620_up_angle_tune.csv"))
    parser.add_argument("--json", type=Path, default=Path("c620_up_angle_best.json"))
    parser.add_argument("--vofa-port", help="Optional virtual serial port for VOFA JustFloat forwarding")
    parser.add_argument("--vofa-baud", type=int, default=115200)
    parser.add_argument("--base", nargs=8, type=float, metavar=("AKP", "AKI", "AKD", "SKP", "SKI", "SKD", "AMAX", "SMAX"), default=[10, 0, 0, 15, 0, 0.3, 5000, 5000])
    args = parser.parse_args()
    target_value = args.target if args.target is not None else args.angle

    base = Pid(args.base[0], args.base[1], args.base[2], args.base[3], args.base[4], args.base[5], int(args.base[6]), int(args.base[7]))
    candidates = list(candidate_grid(base, args.rounds))
    if args.max_trials > 0:
        candidates = candidates[: args.max_trials]

    link = Link(args.port, args.baud, char_delay=args.char_delay, boot_delay=args.boot_delay, verbose=args.verbose)
    vofa = VofaForwarder(args.vofa_port, args.vofa_baud, args.mode, args.ratio)
    best_pid = base
    best_metric = {"score": 1e9}

    fieldnames = ["trial", *Sample.__dataclass_fields__.keys(), "score"]
    try:
        link.command_expect("LOG 0", "OK LOG", wait=0.8)
        link.command_expect("STOP", "OKS", wait=0.8, required=True)
        if args.select:
            link.command_expect(f"SELECT {args.select}", "OKSEL", wait=0.8, required=True)
        link.command_expect(f"MODE {args.mode.upper()}", "OKMODE", wait=0.8, required=True)
        if args.zero:
            link.command_expect("ZERO", "OKZ", wait=0.8, required=True)

        with args.csv.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            writer.writeheader()
            for idx, pid in enumerate(candidates, start=1):
                metric = run_trial(link, vofa, pid, args.mode, target_value, args.ratio, args.duration, args.settle_deg, writer, idx)
                print(f"{idx:03d} score={metric['score']:.3f} settle={metric['settle_ms']:.0f}ms "
                      f"overshoot={metric['overshoot']:.2f} steady={metric['steady']:.2f} pid={pid}")
                if metric["score"] < best_metric["score"]:
                    best_metric = metric
                    best_pid = pid

        apply_pid(link, best_pid)
        link.command_expect("LOG 0", "OK LOG", wait=0.5)
        link.command_expect("STOP", "OKS", wait=0.5)
        link.command_expect("EXIT", "OK EXIT", wait=0.5)
    finally:
        try:
            link.command_expect("EXIT", "OK EXIT", wait=0.5)
        except Exception:
            pass
        vofa.close()
        link.close()

    result = {
        "pid": asdict(best_pid),
        "metric": best_metric,
        "mode": args.mode,
        "target": args.select,
        "test_value": target_value,
    }
    args.json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("BEST", json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
