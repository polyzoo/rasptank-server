#!/usr/bin/env python3
"""Low-level HC-SR04 check without gpiozero DistanceSensor.

This script drives TRIG and polls ECHO directly with short deadlines, so a dead
or miswired sensor should report NO_RISE/NO_FALL instead of hanging inside
gpiozero's DistanceSensor thread.

Run on Raspberry Pi from the repository root:

    python scripts/raw_hcsr04_check.py
    python scripts/raw_hcsr04_check.py --trigger 23 --echo 24 --samples 20

BCM numbering is used. Default HC-SR04 wiring in this project:

    TRIG = BCM 23, physical pin 16
    ECHO = BCM 24, physical pin 18, through a 5 V -> 3.3 V divider
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Protocol


class GpioBackend(Protocol):
    name: str

    def setup(self, trigger_pin: int, echo_pin: int) -> None: ...

    def write_trigger(self, value: int) -> None: ...

    def read_echo(self) -> int: ...

    def close(self) -> None: ...


class LgpioBackend:
    name = "lgpio"

    def __init__(self) -> None:
        import lgpio

        self._lgpio = lgpio
        self._chip: int | None = None
        self._trigger_pin: int | None = None
        self._echo_pin: int | None = None

    def setup(self, trigger_pin: int, echo_pin: int) -> None:
        self._chip = self._lgpio.gpiochip_open(0)
        self._trigger_pin = trigger_pin
        self._echo_pin = echo_pin
        self._lgpio.gpio_claim_output(self._chip, trigger_pin, 0)
        self._lgpio.gpio_claim_input(self._chip, echo_pin)

    def write_trigger(self, value: int) -> None:
        if self._chip is None or self._trigger_pin is None:
            raise RuntimeError("GPIO backend is not initialized")
        self._lgpio.gpio_write(self._chip, self._trigger_pin, value)

    def read_echo(self) -> int:
        if self._chip is None or self._echo_pin is None:
            raise RuntimeError("GPIO backend is not initialized")
        return int(self._lgpio.gpio_read(self._chip, self._echo_pin))

    def close(self) -> None:
        if self._chip is None:
            return
        for pin in (self._trigger_pin, self._echo_pin):
            if pin is not None:
                try:
                    self._lgpio.gpio_free(self._chip, pin)
                except Exception:
                    pass
        self._lgpio.gpiochip_close(self._chip)
        self._chip = None


class RpiGpioBackend:
    name = "RPi.GPIO"

    def __init__(self) -> None:
        import RPi.GPIO as GPIO

        self._gpio = GPIO
        self._trigger_pin: int | None = None
        self._echo_pin: int | None = None

    def setup(self, trigger_pin: int, echo_pin: int) -> None:
        self._trigger_pin = trigger_pin
        self._echo_pin = echo_pin
        self._gpio.setmode(self._gpio.BCM)
        self._gpio.setup(trigger_pin, self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup(echo_pin, self._gpio.IN, pull_up_down=self._gpio.PUD_DOWN)

    def write_trigger(self, value: int) -> None:
        if self._trigger_pin is None:
            raise RuntimeError("GPIO backend is not initialized")
        self._gpio.output(self._trigger_pin, self._gpio.HIGH if value else self._gpio.LOW)

    def read_echo(self) -> int:
        if self._echo_pin is None:
            raise RuntimeError("GPIO backend is not initialized")
        return int(self._gpio.input(self._echo_pin))

    def close(self) -> None:
        self._gpio.cleanup()


@dataclass(frozen=True)
class Measurement:
    status: str
    pulse_us: float | None = None
    distance_cm: float | None = None


def _make_backend(preferred: str) -> GpioBackend:
    errors: list[str] = []
    names = ["lgpio", "rpi"] if preferred == "auto" else [preferred]
    for name in names:
        try:
            if name == "lgpio":
                return LgpioBackend()
            if name == "rpi":
                return RpiGpioBackend()
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    joined = "; ".join(errors) if errors else "unknown backend"
    raise RuntimeError(f"Cannot initialize GPIO backend ({joined})")


def _wait_for_echo(backend: GpioBackend, value: int, timeout_sec: float) -> float | None:
    deadline = time.perf_counter() + timeout_sec
    while time.perf_counter() < deadline:
        if backend.read_echo() == value:
            return time.perf_counter()
    return None


def measure_once(backend: GpioBackend, timeout_sec: float) -> Measurement:
    backend.write_trigger(0)
    time.sleep(0.000002)
    backend.write_trigger(1)
    time.sleep(0.000010)
    backend.write_trigger(0)

    started = _wait_for_echo(backend, 1, timeout_sec)
    if started is None:
        return Measurement("NO_RISE")

    ended = _wait_for_echo(backend, 0, timeout_sec)
    if ended is None:
        return Measurement("NO_FALL")

    pulse_sec = ended - started
    pulse_us = pulse_sec * 1_000_000.0
    distance_cm = pulse_sec * 34_300.0 / 2.0
    return Measurement("OK", pulse_us=pulse_us, distance_cm=distance_cm)


def main() -> int:
    parser = argparse.ArgumentParser(description="Raw HC-SR04 GPIO check (BCM numbering).")
    parser.add_argument("--trigger", type=int, default=23, help="TRIG BCM pin.")
    parser.add_argument("--echo", type=int, default=24, help="ECHO BCM pin.")
    parser.add_argument("--samples", type=int, default=10, help="Number of measurements.")
    parser.add_argument("--interval-sec", type=float, default=0.15, help="Pause between samples.")
    parser.add_argument("--timeout-ms", type=float, default=60.0, help="ECHO wait deadline.")
    parser.add_argument(
        "--backend",
        choices=("auto", "lgpio", "rpi"),
        default="auto",
        help="GPIO backend. auto tries lgpio, then RPi.GPIO.",
    )
    args = parser.parse_args()

    backend = _make_backend(args.backend)
    timeout_sec = args.timeout_ms / 1000.0
    ok_count = 0

    print(
        f"HC-SR04 raw check: backend={backend.name}, TRIG=BCM{args.trigger}, "
        f"ECHO=BCM{args.echo}, timeout={args.timeout_ms:.1f} ms"
    )

    try:
        backend.setup(args.trigger, args.echo)
        print(f"Initial ECHO level: {backend.read_echo()}")
        for i in range(args.samples):
            result = measure_once(backend, timeout_sec)
            if result.status == "OK":
                ok_count += 1
                print(
                    f"[{i + 1:02d}/{args.samples}] OK       "
                    f"pulse={result.pulse_us:.0f} us  distance={result.distance_cm:.1f} cm"
                )
            elif result.status == "NO_RISE":
                print(
                    f"[{i + 1:02d}/{args.samples}] NO_RISE  "
                    "ECHO stayed low: wrong pin, no power, no common ground, dead sensor, "
                    "or TRIG not reaching the module"
                )
            else:
                print(
                    f"[{i + 1:02d}/{args.samples}] NO_FALL  "
                    "ECHO went high but stayed high: divider/wiring fault or sensor stuck"
                )
            time.sleep(max(0.0, args.interval_sec))
    finally:
        backend.close()

    if ok_count:
        print(f"\nSensor responds: {ok_count}/{args.samples} successful samples.")
        return 0

    print("\nNo successful samples. This is a hardware-level failure or wrong BCM pins.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
