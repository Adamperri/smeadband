#!/usr/bin/env python3
"""
smeadband — Headband runtime:
- Two ultrasonic sensors (front/back) -> Buzzer #1 (distance-graded beeps)
- SunFounder Tilt-Switch V1.0 (SIG/VCC/GND) -> Buzzer #2 (tilt-graded beeps)

What’s new:
- Optional tilt alert: when unsafe tilt is detected, Buzzer #2 beeps; the OFF interval
  shrinks (beeps get faster) as "tilt severity" increases. Severity is computed as a
  smoothed fraction of recent time the tilt switch reports "tilt active" (no IMU required).

Pins (GPIO.BOARD):
- Ultrasonic front: TRIG=11, ECHO=12
- Ultrasonic back : TRIG=15, ECHO=16
- Buzzer #1 (obstacles): 13 (active-LOW by default)
- Tilt-switch SIG: disabled by default (pass --tilt-pin 29 to enable)
- Buzzer #2 (tilt): 33 (active-LOW by default)

Run examples on the Pi:
  # Front-only ultrasonic + tilt on pin 29, buzzer2 on 33:
  # sudo python3 run_headband_ultrasonics.py --mode front --tilt-pin 29 --buzzer2-pin 33 --print
  # Dual ultrasonics + tilt (same pins):
  # sudo python3 run_headband_ultrasonics.py --mode dual --tilt-pin 29 --buzzer2-pin 33 --print
"""

import argparse
import time
import math

import RPi.GPIO as GPIO
from ultrasonic_buzzer import (
    UltrasonicSensor, Buzzer, SensorPins, inches_to_cm, cleanup
)

# --- Defaults (BOARD numbering) ---
DEFAULT_FRONT_TRIG = 11
DEFAULT_FRONT_ECHO = 12
DEFAULT_BACK_TRIG = 15
DEFAULT_BACK_ECHO = 16
DEFAULT_BUZZER_PIN = 13          # Buzzer #1 for obstacles
DEFAULT_BUZZER2_PIN = 33         # Buzzer #2 for tilt (same model as #1)

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
    vals = [v for v in (front_cm, back_cm) if v is not None]
    if not vals:
        return None
    return min(vals)

class BeepScheduler:
    """
    Non-blocking beeper:
      - continuous() keeps buzzer ON
      - set_off() turns buzzer OFF
      - pulse_tick(interval, tnow) produces ON pulses of beep_len with OFF gaps 'interval'
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
        # Start pulsing by turning ON immediately
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

def run(args):
    # Sensors
    front = UltrasonicSensor(SensorPins(args.front_trig, args.front_echo, "front")) \
            if args.mode in ("front", "dual") else None
    back = UltrasonicSensor(SensorPins(args.back_trig, args.back_echo, "back")) \
           if args.mode in ("back", "dual") else None

    # Buzzers
    buzzer1 = Buzzer(args.buzzer_pin, active_low=not args.active_high)  # obstacle
    obs_sched = BeepScheduler(buzzer1, args.beep_len)

    tilt_enabled = args.tilt_pin >= 0
    if tilt_enabled:
        # Many LM393-based tilt modules default to active-LOW digital output; use PUD_UP by default.
        GPIO.setup(args.tilt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        buzzer2 = Buzzer(args.buzzer2_pin, active_low=not args.buzzer2_active_high)
        tilt_sched = BeepScheduler(buzzer2, args.tilt_beep_len)
    else:
        buzzer2 = None
        tilt_sched = None

    warn_cm = inches_to_cm(args.warn_in)
    close_cm = inches_to_cm(args.close_in)

    print(f"[smeadband] Mode={args.mode}  warn<= {args.warn_in} in  close<= {args.close_in} in")
    print(f"[smeadband] Pins (BOARD): front(T{args.front_trig}/E{args.front_echo})  "
          f"back(T{args.back_trig}/E{args.back_echo})  buzzer1({args.buzzer_pin})")
    if tilt_enabled:
        print(f"[smeadband] Tilt: pin={args.tilt_pin} active-{'HIGH' if args.tilt_active_high else 'LOW'}  "
              f"buzzer2({args.buzzer2_pin}) active-{'HIGH' if args.buzzer2_active_high else 'LOW'}")
    print("[smeadband] Ctrl+C to exit.")

    t_prev = time.monotonic()
    tilt_severity = 0.0  # 0..1 smoothed fraction of time the tilt switch is active

    try:
        while True:
            t_now = time.monotonic()
            dt = max(t_now - t_prev, 0.0)
            t_prev = t_now

            # --- Ultrasonic distances ---
            front_cm = front.distance_cm() if front else None
            back_cm = back.distance_cm() if back else None
            dist_cm = choose_distance(args.mode, front_cm, back_cm)

            if args.print:
                fs = f"{front_cm:6.1f}" if front_cm is not None else "  None"
                bs = f"{back_cm:6.1f}" if back_cm is not None else "  None"
                cs = f"{dist_cm:6.1f}" if dist_cm is not None else "  None"

            # --- Obstacle beeper (buzzer1) ---
            if dist_cm is None or dist_cm > warn_cm:
                obs_sched.set_off()
            elif dist_cm <= close_cm:
                obs_sched.continuous()
            else:
                interval = map_interval(dist_cm, close_cm, warn_cm,
                                        min_interval_s=args.min_interval,
                                        max_interval_s=args.max_interval)
                obs_sched.pulse_tick(interval, t_now)

            # --- Tilt beeper (buzzer2) ---
            if tilt_enabled:
                raw = GPIO.input(args.tilt_pin)
                tilt_active = (raw == GPIO.HIGH) if args.tilt_active_high else (raw == GPIO.LOW)

                # Exponential smoothing over ~tilt_window seconds
                # severity -> 0..1; when consistently active, approaches 1
                if args.tilt_window <= 0:
                    alpha = 1.0
                else:
                    alpha = 1.0 - math.exp(-dt / args.tilt_window)
                target = 1.0 if tilt_active else 0.0
                tilt_severity += (target - tilt_severity) * alpha

                if tilt_severity >= args.tilt_threshold:
                    # Map severity in [threshold,1] -> OFF interval in [tilt_min_interval, tilt_max_interval],
                    # higher severity => shorter OFF interval (faster beeps)
                    s = max(min(tilt_severity, 1.0), args.tilt_threshold)
                    frac = (s - args.tilt_threshold) / (1.0 - args.tilt_threshold)
                    interval_tilt = args.tilt_min_interval + (1.0 - frac) * (args.tilt_max_interval - args.tilt_min_interval)
                    tilt_sched.pulse_tick(interval_tilt, t_now)
                else:
                    tilt_sched.set_off()

                if args.print:
                    print(f"front={fs} cm   back={bs} cm   chosen={cs} cm   "
                          f"tilt_active={int(tilt_active)} sev={tilt_severity:0.2f}")
            else:
                if args.print:
                    print(f"front={fs} cm   back={bs} cm   chosen={cs} cm")
            # Small tick to avoid busy-loop
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[smeadband] Stopping...")
    finally:
        if tilt_enabled:
            tilt_sched.set_off()
        obs_sched.set_off()
        cleanup()

def build_parser():
    p = argparse.ArgumentParser(description="smeadband ultrasonic + tilt headband runtime")
    # Ultrasonic options
    p.add_argument("--mode", choices=["dual", "front", "back"], default="dual",
                   help="Which ultrasonic sensors to use.")
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
                   help="Set if obstacle buzzer is active-HIGH (default active-LOW).")
    p.add_argument("--beep-len", type=float, default=0.06,
                   help="Obstacle pulse ON length (seconds) when beeping.")
    p.add_argument("--min-interval", type=float, default=0.08,
                   help="Obstacle shortest OFF interval (s) near the close threshold.")
    p.add_argument("--max-interval", type=float, default=0.60,
                   help="Obstacle longest OFF interval (s) at the warn threshold.")

    # Tilt options (SunFounder Tilt-Switch V1.0)
    p.add_argument("--tilt-pin", type=int, default=-1,
                   help="BOARD pin for tilt switch SIG. <0 disables tilt alerts (default).")
    p.add_argument("--tilt-active-high", action="store_true",
                   help="Set if the tilt switch outputs HIGH when tilted. Default expects LOW.")
    p.add_argument("--buzzer2-pin", type=int, default=DEFAULT_BUZZER2_PIN,
                   help="BOARD pin for tilt buzzer (same model as obstacle buzzer).")
    p.add_argument("--buzzer2-active-high", action="store_true",
                   help="Set if tilt buzzer is active-HIGH (default active-LOW).")
    p.add_argument("--tilt-beep-len", type=float, default=0.06,
                   help="Tilt pulse ON length (seconds).")
    p.add_argument("--tilt-min-interval", type=float, default=0.08,
                   help="Shortest OFF interval when tilt severity is max.")
    p.add_argument("--tilt-max-interval", type=float, default=0.80,
                   help="Longest OFF interval at minimal tilt severity (just unsafe).")
    p.add_argument("--tilt-window", type=float, default=0.8,
                   help="Seconds for severity smoothing window.")
    p.add_argument("--tilt-threshold", type=float, default=0.25,
                   help="Severity threshold (0..1) to start tilt beeping.")

    p.add_argument("--print", action="store_true",
                   help="Print measured distances and tilt info each loop.")
    return p

if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    run(args)
