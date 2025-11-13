#!/usr/bin/env python3
"""
ultrasonic_buzzer.py — TEMPORARY TESTBED for smeadband buzzer + tilt hardware

This file is now dedicated to debugging the physical buzzer + tilt switch on the Pi.

Features:
  - Single buzzer on a chosen GPIO pin
  - Single tilt switch on a chosen GPIO pin
  - Multiple test modes:
      * buzz-always      : Buzzer on continuously (simple wiring sanity check)
      * buzz-toggle      : Buzzer on/off every second (no tilt involved)
      * tilt-on-off      : Buzzer on when tilt is active, off otherwise
      * tilt-graded      : Smoothed tilt severity in [0,1], no sound 0..threshold,
                           then beeping with rising frequency up to full tilt

Usage examples (on the Pi):

  # 1) Simple: does the buzzer make *any* sound? (active-HIGH buzzer on GPIO13)
  sudo python3 ultrasonic_buzzer.py --mode buzz-always --buzzer-pin 13

  # 2) Toggle test: click on/off every second
  sudo python3 ultrasonic_buzzer.py --mode buzz-toggle --buzzer-pin 13

  # 3) Tilt ON/OFF test (tilt switch active-LOW on GPIO5 to GND, buzzer on 13)
  sudo python3 ultrasonic_buzzer.py --mode tilt-on-off --tilt-pin 5 --buzzer-pin 13

  # 4) Tilt graded beep test
  sudo python3 ultrasonic_buzzer.py --mode tilt-graded --tilt-pin 5 --buzzer-pin 13 --print
"""

import argparse
import time
import math
import RPi.GPIO as GPIO


# ------------- Core GPIO helpers -------------

class Buzzer:
    """
    Simple buzzer abstraction.

    active_low = False : writing HIGH => buzzer ON (typical active-HIGH modules)
    active_low = True  : writing LOW  => buzzer ON (active-LOW modules)
    """
    def __init__(self, pin: int, active_low: bool = False):
        self.pin = pin
        self.active_low = active_low
        GPIO.setup(self.pin, GPIO.OUT)
        self.off()

    def on(self):
        if self.active_low:
            GPIO.output(self.pin, GPIO.LOW)
        else:
            GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        if self.active_low:
            GPIO.output(self.pin, GPIO.HIGH)
        else:
            GPIO.output(self.pin, GPIO.LOW)


class BeepScheduler:
    """
    Non-blocking driver for buzzer:
      - 'off'       : buzzer off
      - 'cont'      : buzzer continuously on
      - 'pulse'     : repeating buzzer pulses (beep_len on, interval off)
    """
    def __init__(self, buzzer: Buzzer, beep_len: float):
        self.buzzer = buzzer
        self.beep_len = beep_len
        self.next_toggle = 0.0
        self.is_on = False
        self.mode = "off"

    def set_off(self):
        if self.is_on or self.mode != "off":
            self.buzzer.off()
        self.mode = "off"
        self.is_on = False
        self.next_toggle = 0.0

    def continuous(self):
        if not self.is_on or self.mode != "cont":
            self.buzzer.on()
        self.mode = "cont"
        self.is_on = True
        self.next_toggle = 0.0

    def pulse_tick(self, interval: float, tnow: float):
        """
        Call this every loop iteration with the desired OFF interval.
        """
        if self.mode != "pulse":
            self.mode = "pulse"
            self.buzzer.on()
            self.is_on = True
            self.next_toggle = tnow + self.beep_len
            return

        if tnow >= self.next_toggle:
            if self.is_on:
                self.buzzer.off()
                self.is_on = False
                self.next_toggle = tnow + max(interval, 0.0)
            else:
                self.buzzer.on()
                self.is_on = True
                self.next_toggle = tnow + self.beep_len


# ------------- Test modes -------------


def mode_buzz_always(buzzer: Buzzer, args):
    """
    Simple wiring sanity check: buzzer ON continuously.
    """
    print("[test] Mode: buzz-always  ->  buzzer should be ON continuously.")
    buzzer.on()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[test] Stopping buzz-always...")
    finally:
        buzzer.off()


def mode_buzz_toggle(buzzer: Buzzer, args):
    """
    Buzzer toggles on/off every second.
    """
    print("[test] Mode: buzz-toggle  ->  buzzer should click on/off every ~1s.")
    state = False
    try:
        while True:
            state = not state
            if state:
                buzzer.on()
                if args.print:
                    print("[test] buzzer ON")
            else:
                buzzer.off()
                if args.print:
                    print("[test] buzzer OFF")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[test] Stopping buzz-toggle...")
    finally:
        buzzer.off()


def read_tilt_raw(args):
    """
    Read the raw tilt GPIO and convert to 'active' boolean based on active-high/low.
    """
    raw = GPIO.input(args.tilt_pin)
    # If tilt is active-high, HIGH means active; else LOW means active.
    return (raw == GPIO.HIGH) if args.tilt_active_high else (raw == GPIO.LOW)


def mode_tilt_on_off(buzzer: Buzzer, args):
    """
    Buzzer ON when tilt is active, OFF otherwise. No smoothing, no graded frequency.
    """
    print("[test] Mode: tilt-on-off")
    print(f"[test] Tilt pin={args.tilt_pin}, active-{'HIGH' if args.tilt_active_high else 'LOW'}")
    print(f"[test] Buzzer pin={args.buzzer_pin}, active-{'LOW' if args.buzzer_active_low else 'HIGH'}")
    print("[test] Tilt the switch and watch/listen if buzzer tracks ON/OFF.")

    try:
        while True:
            active = read_tilt_raw(args)
            if active:
                buzzer.on()
                if args.print:
                    print("[test] tilt=ACTIVE  -> buzzer ON")
            else:
                buzzer.off()
                if args.print:
                    print("[test] tilt=inactive -> buzzer OFF")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[test] Stopping tilt-on-off...")
    finally:
        buzzer.off()


def mode_tilt_graded(buzzer: Buzzer, args):
    """
    Tilt severity in [0,1] with exponential smoothing.
      - severity < threshold : buzzer OFF
      - severity >= threshold : buzzer pulses with OFF interval shrinking
                                as severity increases.
    """
    print("[test] Mode: tilt-graded")
    print(f"[test] Tilt pin={args.tilt_pin}, active-{'HIGH' if args.tilt_active_high else 'LOW'}")
    print(f"[test] Buzzer pin={args.buzzer_pin}, active-{'LOW' if args.buzzer_active_low else 'HIGH'}")
    print(f"[test] Threshold={args.tilt_threshold}, OFF interval range=[{args.tilt_min_interval}, {args.tilt_max_interval}]")
    print("[test] Tilt the switch; more sustained tilt = faster beeps.\n")

    sched = BeepScheduler(buzzer, args.tilt_beep_len)
    t_prev = time.monotonic()
    severity = 0.0

    try:
        sched.set_off()
        while True:
            t_now = time.monotonic()
            dt = max(t_now - t_prev, 0.0)
            t_prev = t_now

            active = read_tilt_raw(args)
            target = 1.0 if active else 0.0

            # Exponential smoothing
            alpha = 1.0 if args.tilt_window <= 0 else 1.0 - math.exp(-dt / args.tilt_window)
            severity += (target - severity) * alpha

            if severity >= args.tilt_threshold:
                s = max(min(severity, 1.0), args.tilt_threshold)
                frac = (s - args.tilt_threshold) / (1.0 - args.tilt_threshold)  # 0 at threshold, 1 at max
                # OFF interval shrinks as frac -> 1
                interval = args.tilt_min_interval + (1.0 - frac) * (args.tilt_max_interval - args.tilt_min_interval)
                sched.pulse_tick(interval, t_now)
            else:
                sched.set_off()

            if args.print:
                print(f"raw_active={active}  severity={severity:0.3f}")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[test] Stopping tilt-graded...")
    finally:
        sched.set_off()


# ------------- Arg parsing + main -------------

def build_parser():
    p = argparse.ArgumentParser(description="smeadband buzzer + tilt hardware testbed")

    p.add_argument(
        "--mode",
        choices=["buzz-always", "buzz-toggle", "tilt-on-off", "tilt-graded"],
        default="buzz-always",
        help="Test mode to run."
    )

    # Buzzer
    p.add_argument(
        "--buzzer-pin",
        type=int,
        default=13,
        help="BCM pin for buzzer (default: 13).",
    )
    p.add_argument(
        "--buzzer-active-low",
        action="store_true",
        help="If set, buzzer is active-LOW (driving pin LOW turns buzzer ON). Default is active-HIGH.",
    )

    # Tilt
    p.add_argument(
        "--tilt-pin",
        type=int,
        default=5,
        help="BCM pin for tilt SIG (default: 5). <0 disables tilt modes.",
    )
    p.add_argument(
        "--tilt-active-high",
        action="store_true",
        help="If set, tilt is active-HIGH. Default assumes active-LOW (switch pulls pin LOW when tilted).",
    )

    # Tilt-graded parameters
    p.add_argument(
        "--tilt-beep-len",
        type=float,
        default=0.06,
        help="On-time (seconds) for each tilt beep in tilt-graded mode.",
    )
    p.add_argument(
        "--tilt-min-interval",
        type=float,
        default=0.08,
        help="Minimum OFF interval between tilt beeps at full tilt.",
    )
    p.add_argument(
        "--tilt-max-interval",
        type=float,
        default=0.80,
        help="Maximum OFF interval between tilt beeps just above threshold.",
    )
    p.add_argument(
        "--tilt-window",
        type=float,
        default=0.8,
        help="Smoothing window (seconds) for tilt severity (larger = slower response).",
    )
    p.add_argument(
        "--tilt-threshold",
        type=float,
        default=0.20,
        help="Tilt severity threshold in [0,1]. Below threshold => silent.",
    )

    # Debug
    p.add_argument(
        "--print",
        action="store_true",
        help="Print debug info each loop iteration.",
    )

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # If we’re using a tilt mode, configure tilt pin
    if args.mode.startswith("tilt"):
        if args.tilt_pin < 0:
            raise SystemExit("[error] Tilt mode selected but --tilt-pin < 0 (disabled).")
        # Pull-up by default; we’ll interpret HIGH/LOW according to tilt_active_high flag
        pull = GPIO.PUD_UP if not args.tilt_active_high else GPIO.PUD_DOWN
        GPIO.setup(args.tilt_pin, GPIO.IN, pull_up_down=pull)

    buzzer = Buzzer(args.buzzer_pin, active_low=args.buzzer_active_low)

    try:
        if args.mode == "buzz-always":
            mode_buzz_always(buzzer, args)
        elif args.mode == "buzz-toggle":
            mode_buzz_toggle(buzzer, args)
        elif args.mode == "tilt-on-off":
            mode_tilt_on_off(buzzer, args)
        elif args.mode == "tilt-graded":
            mode_tilt_graded(buzzer, args)
        else:
            raise SystemExit(f"[error] Unknown mode: {args.mode}")
    finally:
        buzzer.off()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
