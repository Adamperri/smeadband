#!/usr/bin/env python3
"""
smeadband — Ultrasonic + Buzzer helpers (Raspberry Pi, RPi.GPIO, **BCM** mode)

- BCM pin defaults are all <= 27 (safe on Pi 4 Model B accessible header).
- Supports two HC-SR04 ultrasonic sensors (front/back) and a single buzzer.
- Warn at <= 12 in (30.48 cm); continuous at <= 4 in (10.16 cm).

NOTE: If you're using 5V HC-SR04, level-shift ECHO down to 3.3V for the Pi.
"""

import time
import statistics
from dataclasses import dataclass
import RPi.GPIO as GPIO  # Requires RPi.GPIO on Raspberry Pi

SPEED_OF_SOUND_M_S = 343.0  # ~20°C air
GPIO.setmode(GPIO.BCM)      # Use BCM numbering

@dataclass
class SensorPins:
    trig: int
    echo: int
    name: str = "sensor"

class UltrasonicSensor:
    """HC-SR04 distance reader with timeouts and small-sample median filtering."""
    def __init__(self, pins: SensorPins, echo_pull=GPIO.PUD_DOWN):
        self.pins = pins
        GPIO.setup(self.pins.trig, GPIO.OUT)
        GPIO.setup(self.pins.echo, GPIO.IN, pull_up_down=echo_pull)
        GPIO.output(self.pins.trig, GPIO.LOW)
        time.sleep(0.05)  # settle

    def _single_measure_seconds(self, start_timeout=0.02, echo_timeout=0.03):
        """Trigger 10µs pulse and measure ECHO high-time. None on timeout."""
        GPIO.output(self.pins.trig, GPIO.LOW); time.sleep(0.000002)
        GPIO.output(self.pins.trig, GPIO.HIGH); time.sleep(0.00001)
        GPIO.output(self.pins.trig, GPIO.LOW)

        t_start_wait = time.perf_counter()
        while GPIO.input(self.pins.echo) == GPIO.LOW:
            if time.perf_counter() - t_start_wait > start_timeout:
                return None

        t_rise = time.perf_counter()
        while GPIO.input(self.pins.echo) == GPIO.HIGH:
            if time.perf_counter() - t_rise > echo_timeout:
                return None
        t_fall = time.perf_counter()
        return t_fall - t_rise

    def distance_cm(self, samples=3):
        """Median of N samples; float cm or None if no echo."""
        readings = []
        for _ in range(samples):
            pulse_s = self._single_measure_seconds()
            if pulse_s is None:
                continue
            dist_cm = (pulse_s * SPEED_OF_SOUND_M_S / 2.0) * 100.0
            readings.append(dist_cm)
            time.sleep(0.01)
        return statistics.median(readings) if readings else None

class Buzzer:
    """Active buzzer; default active-LOW."""
    def __init__(self, pin: int, active_low: bool = True):
        self.pin = pin
        self.active_low = active_low
        GPIO.setup(self.pin, GPIO.OUT)
        self.off()
    def on(self):
        GPIO.output(self.pin, GPIO.LOW if self.active_low else GPIO.HIGH)
    def off(self):
        GPIO.output(self.pin, GPIO.HIGH if self.active_low else GPIO.LOW)
    def pulse(self, on_time_s: float):
        self.on(); time.sleep(max(on_time_s, 0.0)); self.off()

def inches_to_cm(inches: float) -> float:
    return inches * 2.54

def cleanup():
    GPIO.cleanup()
