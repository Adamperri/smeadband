#!/usr/bin/env python3
"""
smeadband — Headband runtime (BCM mode, pins <= 27):
- Two ultrasonic sensors (front/back) -> Buzzer #1 (distance-graded beeps)
- SunFounder Tilt-Switch V1.0 (SIG/VCC/GND) -> Buzzer #2 (tilt-graded beeps)

BCM Default pins (all ≤ 27):
  Front: TRIG=GPIO17, ECHO=GPIO18
  Back : TRIG=GPIO22, ECHO=GPIO23
  Buzzer #1 (obstacles): GPIO27 (active-LOW)
  Tilt switch SIG: GPIO5   (enable via --tilt-pin 5)
  Buzzer #2 (tilt):  GPIO13 (active-LOW)

Examples on the Pi:
  sudo python3 run_headband_ultrasonics.py --mode front --tilt-pin 5 --buzzer2-pin 13 --print
  sudo python3 run_headband_ultrasonics.py --mode dual  --tilt-pin 5 --buzzer2-pin 13 --print
"""

import argparse
import time
import math
import RPi.GPIO as GPIO
from ultrasonic_buzzer import UltrasonicSensor, Buzzer, SensorPins, inches_to_cm, cleanup

# Defaults (BCM numbering, all <= 27)
DEFAULT_FRONT_TRIG = 17
DEFAULT_FRONT_ECHO = 18
DEFAULT_BACK_TRIG  = 22
DEFAULT_BACK_ECHO  = 23
DEFAULT_BUZZER_PIN = 27          # Buzzer #1 for obstacles
DEFAULT_BUZZER2_PIN = 13         # Buzzer #2 for tilt

def map_interval(distance_cm, close_cm, warn_cm, min_interval_s=0.08, max_interval_s=0.60):
    """Map distance in [close,warn] -> OFF interval in [min,max]; closer => faster."""
    x = max(min(distance_cm, warn_cm), close_cm)
    if warn_cm == close_cm: return min_interval_s
    f = (x - close_cm) / (warn_cm - close_cm)  # 0 at close, 1 at warn
    return min_interval_s + f * (max_interval_s - min_interval_s)

def choose_distance(mode, front_cm, back_cm):
    if mode == "front": return front_cm
    if mode == "back" : return back_cm
    vals = [v for v in (front_cm, back_cm) if v is not None]
    return min(vals) if vals else None

class BeepScheduler:
    """Non-blocking beeper with continuous or pulsed modes."""
    def __init__(self, buzzer: Buzzer, beep_len: float):
        self.buzzer = buzzer; self.beep_len = beep_len
        self.next_toggle = 0.0; self.is_on = False; self.mode = "off"
    def set_off(self):
        if self.is_on or self.mode != "off": self.buzzer.off()
        self.mode = "off"; self.is_on = False; self.next_toggle = 0.0
    def continuous(self):
        if not self.is_on or self.mode != "cont": self.buzzer.on()
        self.mode = "cont"; self.is_on = True; self.next_toggle = 0.0
    def pulse_tick(self, interval: float, tnow: float):
        if self.mode != "pulse":
            self.mode = "pulse"; self.buzzer.on(); self.is_on = True; self.next_toggle = tnow + self.beep_len; return
        if tnow >= self.next_toggle:
            if self.is_on:
                self.buzzer.off(); self.is_on = False; self.next_toggle = tnow + max(interval, 0.0)
            else:
                self.buzzer.on(); self.is_on = True; self.next_toggle = tnow + self.beep_len

def run(args):
    front = UltrasonicSensor(SensorPins(args.front_trig, args.front_echo, "front")) if args.mode in ("front","dual") else None
    back  = UltrasonicSensor(SensorPins(args.back_trig,  args.back_echo,  "back"))  if args.mode in ("back","dual")  else None

    buzzer1 = Buzzer(args.buzzer_pin, active_low=not args.active_high)
    obs_sched = BeepScheduler(buzzer1, args.beep_len)

    tilt_enabled = args.tilt_pin >= 0
    if tilt_enabled:
        GPIO.setup(args.tilt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # default expects active-LOW modules
        buzzer2 = Buzzer(args.buzzer2_pin, active_low=not args.buzzer2_active_high)
        tilt_sched = BeepScheduler(buzzer2, args.tilt_beep_len)
    else:
        buzzer2 = None; tilt_sched = None

    warn_cm, close_cm = inches_to_cm(args.warn_in), inches_to_cm(args.close_in)
    print(f"[smeadband] Mode={args.mode}  warn<= {args.warn_in} in  close<= {args.close_in} in")
    print(f"[smeadband] Pins (BCM): front(T{args.front_trig}/E{args.front_echo})  back(T{args.back_trig}/E{args.back_echo})  buzzer1({args.buzzer_pin})")
    if tilt_enabled:
        print(f"[smeadband] Tilt: pin={args.tilt_pin} active-{'HIGH' if args.tilt_active_high else 'LOW'}  buzzer2({args.buzzer2_pin}) active-{'HIGH' if args.buzzer2_active_high else 'LOW'}")
    print("[smeadband] Ctrl+C to exit.")
    t_prev = time.monotonic(); tilt_severity = 0.0

    try:
        while True:
            t_now = time.monotonic(); dt = max(t_now - t_prev, 0.0); t_prev = t_now
            front_cm = front.distance_cm() if front else None
            back_cm  = back.distance_cm()  if back  else None
            dist_cm  = choose_distance(args.mode, front_cm, back_cm)

            # Obstacle beeps
            if dist_cm is None or dist_cm > warn_cm: obs_sched.set_off()
            elif dist_cm <= close_cm:               obs_sched.continuous()
            else:
                interval = map_interval(dist_cm, close_cm, warn_cm, args.min_interval, args.max_interval)
                obs_sched.pulse_tick(interval, t_now)

            # Tilt beeps
            if tilt_enabled:
                raw = GPIO.input(args.tilt_pin)
                tilt_active = (raw == GPIO.HIGH) if args.tilt_active_high else (raw == GPIO.LOW)
                alpha = 1.0 if args.tilt_window <= 0 else 1.0 - math.exp(-dt / args.tilt_window)
                target = 1.0 if tilt_active else 0.0
                tilt_severity += (target - tilt_severity) * alpha
                if tilt_severity >= args.tilt_threshold:
                    s = max(min(tilt_severity, 1.0), args.tilt_threshold)
                    frac = (s - args.tilt_threshold) / (1.0 - args.tilt_threshold)
                    interval_tilt = args.tilt_min_interval + (1.0 - frac) * (args.tilt_max_interval - args.tilt_min_interval)
                    tilt_sched.pulse_tick(interval_tilt, t_now)
                else:
                    tilt_sched.set_off()

            if args.print:
                fs = f"{front_cm:6.1f}" if front_cm is not None else "  None"
                bs = f"{back_cm:6.1f}"  if back_cm  is not None else "  None"
                cs = f"{dist_cm:6.1f}"  if dist_cm  is not None else "  None"
                if tilt_enabled:
                    print(f"front={fs} cm  back={bs} cm  chosen={cs} cm  tilt_sev={tilt_severity:0.2f}")
                else:
                    print(f"front={fs} cm  back={bs} cm  chosen={cs} cm")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[smeadband] Stopping...")
    finally:
        if tilt_enabled: tilt_sched.set_off()
        obs_sched.set_off()
        cleanup()

def build_parser():
    p = argparse.ArgumentParser(description="smeadband ultrasonic + tilt headband runtime (BCM pins <= 27)")
    # Ultrasonic
    p.add_argument("--mode", choices=["dual","front","back"], default="dual")
    p.add_argument("--warn-in", type=float, default=12.0)
    p.add_argument("--close-in", type=float, default=4.0)
    p.add_argument("--front-trig", type=int, default=DEFAULT_FRONT_TRIG)
    p.add_argument("--front-echo", type=int, default=DEFAULT_FRONT_ECHO)
    p.add_argument("--back-trig",  type=int, default=DEFAULT_BACK_TRIG)
    p.add_argument("--back-echo",  type=int, default=DEFAULT_BACK_ECHO)
    p.add_argument("--buzzer-pin", type=int, default=DEFAULT_BUZZER_PIN)
    p.add_argument("--active-high", action="store_true")
    p.add_argument("--beep-len", type=float, default=0.06)
    p.add_argument("--min-interval", type=float, default=0.08)
    p.add_argument("--max-interval", type=float, default=0.60)
    # Tilt
    p.add_argument("--tilt-pin", type=int, default=-1, help="BCM SIG pin (e.g., 5). <0 disables tilt (default).")
    p.add_argument("--tilt-active-high", action="store_true")
    p.add_argument("--buzzer2-pin", type=int, default=DEFAULT_BUZZER2_PIN)
    p.add_argument("--buzzer2-active-high", action="store_true")
    p.add_argument("--tilt-beep-len", type=float, default=0.06)
    p.add_argument("--tilt-min-interval", type=float, default=0.08)
    p.add_argument("--tilt-max-interval", type=float, default=0.80)
    p.add_argument("--tilt-window", type=float, default=0.8)
    p.add_argument("--tilt-threshold", type=float, default=0.25)
    p.add_argument("--print", action="store_true")
    return p

if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    run(args)
