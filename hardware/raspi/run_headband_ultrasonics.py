#!/usr/bin/env python3
"""
smeadband — Headband runtime for two ultrasonic sensors + one buzzer.

Modes:
  - dual  : read front and back; use the *closest* distance to drive the buzzer
  - front : read front only (use this while you only have one sensor connected)
  - back  : read back only

Thresholds (defaults to your spec):
  - warn  <= 12 in (30.48 cm): start pulsing
  - close <=  4 in (10.16 cm): continuous buzz

Pin defaults (GPIO.BOARD):
  - Front: TRIG=11, ECHO=12
  - Back : TRIG=15, ECHO=16
  - Buzzer: 13 (active-low ON)

Usage examples (run with sudo on a Pi):
  sudo python3 hardware/raspi/run_headband_ultrasonics.py --mode front
  sudo python3 hardware/raspi/run_headband_ultrasonics.py --mode dual --print
"""

import argparse
import time

import RPi.GPIO as GPIO
from ultrasonic_buzzer import (
    UltrasonicSensor, Buzzer, SensorPins, inches_to_cm, cleanup
)

# --- Defaults (BOARD numbering) ---
DEFAULT_FRONT_TRIG = 11
DEFAULT_FRONT_ECHO = 12
DEFAULT_BACK_TRIG = 15
DEFAULT_BACK_ECHO = 16
DEFAULT_BUZZER_PIN = 13


def map_interval(distance_cm: float,
                 close_cm: float,
                 warn_cm: float,
                 min_interval_s: float = 0.08,
                 max_interval_s: float = 0.60) -> float:
    """
    Map distance in [close_cm, warn_cm] to an OFF interval in [min_interval, max_interval].
    Closer -> shorter OFF interval (i.e., higher beep frequency).
    Largest OFF interval at initial activation (near warn_cm), shortest near close_cm.
    """
    x = max(min(distance_cm, warn_cm), close_cm)  # clamp to [close, warn]
    if warn_cm == close_cm:
        return min_interval_s
    f = (x - close_cm) / (warn_cm - close_cm)  # 0 at close, 1 at warn
    return min_interval_s + f * (max_interval_s - min_interval_s)


def choose_distance(mode, front_cm, back_cm):
    if mode == "front":
        return front_cm
    if mode == "back":
        return back_cm
    # dual: pick the closest valid reading
    vals = [v for v in (front_cm, back_cm) if v is not None]
    if not vals:
        return None
    return min(vals)


def run(args):
    # Sensors
    front = UltrasonicSensor(SensorPins(args.front_trig, args.front_echo, "front")) \
            if args.mode in ("front", "dual") else None
    back = UltrasonicSensor(SensorPins(args.back_trig, args.back_echo, "back")) \
           if args.mode in ("back", "dual") else None

    # Buzzer
    buzzer = Buzzer(args.buzzer_pin, active_low=not args.active_high)

    warn_cm = inches_to_cm(args.warn_in)
    close_cm = inches_to_cm(args.close_in)

    print(f"[smeadband] Mode={args.mode}  warn<= {args.warn_in} in  close<= {args.close_in} in")
    print(f"[smeadband] Pins (BOARD): front(T{args.front_trig}/E{args.front_echo})  "
          f"back(T{args.back_trig}/E{args.back_echo})  buzzer({args.buzzer_pin})")
    print("[smeadband] Ctrl+C to exit.")

    try:
        while True:
            front_cm = front.distance_cm() if front else None
            back_cm = back.distance_cm() if back else None
            dist_cm = choose_distance(args.mode, front_cm, back_cm)

            if args.print:
                fs = f"{front_cm:6.1f}" if front_cm is not None else "  None"
                bs = f"{back_cm:6.1f}" if back_cm is not None else "  None"
                cs = f"{dist_cm:6.1f}" if dist_cm is not None else "  None"
                print(f"front={fs} cm   back={bs} cm   chosen={cs} cm")

            # Drive the buzzer
            if dist_cm is None or dist_cm > warn_cm:
                buzzer.off()
                time.sleep(0.10)
                continue

            if dist_cm <= close_cm:
                # Continuous buzz when very close (≤ close threshold)
                buzzer.on()
                time.sleep(0.02)  # small yield to avoid a busy loop
                continue

            # Pulsed beeps between close and warn:
            #   - ON for beep_len
            #   - OFF interval shrinks as the object gets closer
            interval = map_interval(dist_cm, close_cm, warn_cm,
                                    min_interval_s=args.min_interval,
                                    max_interval_s=args.max_interval)
            buzzer.pulse(args.beep_len)     # ON for beep_len
            time.sleep(interval)            # OFF for 'interval'

    except KeyboardInterrupt:
        print("\n[smeadband] Stopping...")
    finally:
        buzzer.off()
        cleanup()


def build_parser():
    p = argparse.ArgumentParser(description="smeadband ultrasonic headband runtime")
    p.add_argument("--mode", choices=["dual", "front", "back"], default="dual",
                   help="Which sensors to use.")
    p.add_argument("--warn-in", type=float, default=12.0,
                   help="Warn threshold (inches). Start pulsing at <= this.")
    p.add_argument("--close-in", type=float, default=4.0,
                   help="Close threshold (inches). Continuous buzz at <= this.")

    p.add_argument("--front-trig", type=int, default=DEFAULT_FRONT_TRIG)
    p.add_argument("--front-echo", type=int, default=DEFAULT_FRONT_ECHO)
    p.add_argument("--back-trig", type=int, default=DEFAULT_BACK_TRIG)
    p.add_argument("--back-echo", type=int, default=DEFAULT_BACK_ECHO)
    p.add_argument("--buzzer-pin", type=int, default=DEFAULT_BUZZER_PIN)

    p.add_argument("--active-high", action="store_true",
                   help="Set if your buzzer is active-HIGH (most are active-LOW).")

    p.add_argument("--beep-len", type=float, default=0.06,
                   help="Pulse ON length (seconds) when beeping.")
    p.add_argument("--min-interval", type=float, default=0.08,
                   help="Shortest OFF interval (s) near the close threshold.")
    p.add_argument("--max-interval", type=float, default=0.60,
                   help="Longest OFF interval (s) at the warn threshold.")

    p.add_argument("--print", action="store_true",
                   help="Print measured distances each loop.")
    return p


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    run(args)
