#!/usr/bin/env python3
"""
smeadband — Headband runtime (BCM mode, pins <= 27):

Hardware:
  - Two ultrasonic sensors (front & back)
      Front: TRIG=GPIO17, ECHO=GPIO18
      Back : TRIG=GPIO22, ECHO=GPIO23
  - Two vibration motors:
      Vib1 (front feedback) = GPIO12  (default)
      Vib2 (back feedback)  = GPIO16  (default)
  - One tilt switch (SunFounder Tilt-Switch V1.0):
      SIG pin (e.g., GPIO5)
  - One buzzer (for tilt only):
      Buzzer = GPIO13 (default)

Behavior:
  - Vibrators:
      * OFF if no obstacle or object beyond warn distance
      * PULSED faster as object approaches from warn -> close
      * CONTINUOUS when very close (<= close distance)
  - Tilt buzzer:
      * Internally: tilt_level in [0,1], where 1.0 = baseline (no tilt), 0.0 = max tilt
      * We use tilt_intensity = 1 - tilt_level
      * No buzzing for tilt_intensity in [0.0, threshold)
      * Pulsed beeps, increasing in frequency as tilt_intensity goes from threshold -> 1.0
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

DEFAULT_VIB1_PIN   = 12
DEFAULT_VIB2_PIN   = 16

DEFAULT_TILT_PIN       = 5   # Tilt SIG
DEFAULT_TILT_BUZZERPIN = 13  # Single buzzer used ONLY for tilt


def map_interval(distance_cm, close_cm, warn_cm, min_interval_s=0.08, max_interval_s=0.60):
    """Map distance in [close,warn] -> OFF interval in [min,max]; closer => faster."""
    x = max(min(distance_cm, warn_cm), close_cm)
    if warn_cm == close_cm:
        return min_interval_s
    f = (x - close_cm) / (warn_cm - close_cm)  # 0 at close, 1 at warn
    return min_interval_s + f * (max_interval_s - min_interval_s)


def choose_distance(mode, front_cm, back_cm):
    """Choose which distance to consider 'primary' based on mode."""
    if mode == "front":
        return front_cm
    if mode == "back":
        return back_cm
    vals = [v for v in (front_cm, back_cm) if v is not None]
    return min(vals) if vals else None


class BeepScheduler:
    """Non-blocking driver for buzzer/vibrator with continuous or pulsed modes."""
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


def run(args):
    # Ensure close <= warn
    if args.close_in > args.warn_in:
        print("[smeadband] NOTE: --close-in > --warn-in; swapping so close <= warn.")
        args.close_in, args.warn_in = args.warn_in, args.close_in

    # Ultrasonic sensors
    front = UltrasonicSensor(SensorPins(args.front_trig, args.front_echo, "front")) if args.mode in ("front", "dual") else None
    back  = UltrasonicSensor(SensorPins(args.back_trig,  args.back_echo,  "back"))  if args.mode in ("back", "dual")  else None

    # Vibrators (active-LOW modules by default; "vib-active-high" flips logic)
    vib1 = Buzzer(args.vib_1_pin, active_low=not args.vib_active_high)
    vib2 = Buzzer(args.vib_2_pin, active_low=not args.vib_active_high)
    vib1.off()
    vib2.off()
    vib1_sched = BeepScheduler(vib1, args.vib_pulse_len)
    vib2_sched = BeepScheduler(vib2, args.vib_pulse_len)

    # Tilt + single buzzer
    tilt_enabled = args.tilt_pin >= 0
    if tilt_enabled:
        pull = GPIO.PUD_DOWN if args.tilt_active_high else GPIO.PUD_UP
        GPIO.setup(args.tilt_pin, GPIO.IN, pull_up_down=pull)

        # buzzer is active-LOW by default (your actual wiring)
        tilt_buzzer = Buzzer(args.buzzer_pin, active_low=not args.buzzer_active_high)
        tilt_buzzer.off()
        tilt_sched = BeepScheduler(tilt_buzzer, args.tilt_beep_len)
    else:
        tilt_sched = None

    warn_cm, close_cm = inches_to_cm(args.warn_in), inches_to_cm(args.close_in)

    print(f"[smeadband] Mode={args.mode}  warn<= {args.warn_in} in  close<= {args.close_in} in")
    print(f"[smeadband] Ultrasonic pins (BCM): front(T{args.front_trig}/E{args.front_echo})  back(T{args.back_trig}/E{args.back_echo})")
    print(f"[smeadband] Vibrators: V1={args.vib_1_pin} V2={args.vib_2_pin} active-{'HIGH' if args.vib_active_high else 'LOW'}")
    if tilt_enabled:
        print(f"[smeadband] Tilt: pin={args.tilt_pin} active-{'HIGH' if args.tilt_active_high else 'LOW'}")
        print(f"[smeadband] Buzzer (tilt only): pin={args.buzzer_pin} active-{'HIGH' if args.buzzer_active_high else 'LOW'}")
    else:
        print("[smeadband] Tilt disabled (no tilt pin provided).")
    print("[smeadband] Ctrl+C to exit.")

    t_prev = time.monotonic()

    # NEW: tilt_level starts at baseline 1.0 (no tilt)
    tilt_level = 1.0  # 1.0 = baseline, 0.0 = maximum tilt

    try:
        vib1_sched.set_off()
        vib2_sched.set_off()
        if tilt_enabled:
            tilt_sched.set_off()

        while True:
            t_now = time.monotonic()
            dt = max(t_now - t_prev, 0.0)
            t_prev = t_now

            front_cm = front.distance_cm() if front else None
            back_cm  = back.distance_cm()  if back  else None
            dist_cm  = choose_distance(args.mode, front_cm, back_cm)

            # -------------------------
            # VIBRATOR LOGIC
            # -------------------------
            # Vib1 = front
            if front_cm is None or front_cm > warn_cm:
                vib1_sched.set_off()
            elif front_cm <= close_cm:
                vib1_sched.continuous()
            else:
                interval = map_interval(front_cm, close_cm, warn_cm,
                                        args.vib_min_interval, args.vib_max_interval)
                vib1_sched.pulse_tick(interval, t_now)

            # Vib2 = back
            if back_cm is None or back_cm > warn_cm:
                vib2_sched.set_off()
            elif back_cm <= close_cm:
                vib2_sched.continuous()
            else:
                interval = map_interval(back_cm, close_cm, warn_cm,
                                        args.vib_min_interval, args.vib_max_interval)
                vib2_sched.pulse_tick(interval, t_now)

            # -------------------------
            # TILT BUZZER LOGIC (single buzzer)
            #
            #   We maintain tilt_level ∈ [0,1]:
            #       1.0 ≈ baseline / no tilt
            #       0.0 ≈ fully tilted
            #
            #   We derive tilt_intensity = 1 - tilt_level:
            #       0   = baseline
            #       1   = max tilt
            #
            #   No sound for tilt_intensity in [0, threshold),
            #   then increasing frequency as tilt_intensity -> 1.0
            # -------------------------
            if tilt_enabled:
                raw = GPIO.input(args.tilt_pin)
                tilt_active = (raw == GPIO.HIGH) if args.tilt_active_high else (raw == GPIO.LOW)

                # For reversed semantics:
                #   tilt_active  => level wants to go to 0.0
                #   not active   => level wants to go to 1.0
                target_level = 0.0 if tilt_active else 1.0

                # Exponential smoothing on tilt_level
                alpha = 1.0 if args.tilt_window <= 0 else 1.0 - math.exp(-dt / args.tilt_window)
                tilt_level += (target_level - tilt_level) * alpha

                # Convert level [0,1] to intensity [0,1] = 1 - level
                tilt_intensity = 1.0 - tilt_level

                if tilt_intensity >= args.tilt_threshold:
                    s = max(min(tilt_intensity, 1.0), args.tilt_threshold)
                    frac = (s - args.tilt_threshold) / (1.0 - args.tilt_threshold)  # 0 at threshold, 1 at max
                    interval_tilt = args.tilt_min_interval + (1.0 - frac) * (args.tilt_max_interval - args.tilt_min_interval)
                    tilt_sched.pulse_tick(interval_tilt, t_now)
                else:
                    tilt_sched.set_off()

            if args.print:
                fs = f"{front_cm:6.1f}" if front_cm is not None else "  None"
                bs = f"{back_cm:6.1f}"  if back_cm  is not None else "  None"
                cs = f"{dist_cm:6.1f}"  if dist_cm  is not None else "  None"
                if tilt_enabled:
                    print(
                        f"front={fs} cm  back={bs} cm  chosen={cs} cm  "
                        f"tilt_level={tilt_level:0.2f}  tilt_intensity={tilt_intensity:0.2f}"
                    )
                else:
                    print(f"front={fs} cm  back={bs} cm  chosen={cs} cm")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[smeadband] Stopping...")
    finally:
        try:
            vib1_sched.set_off()
            vib2_sched.set_off()
            if tilt_enabled:
                tilt_sched.set_off()
        finally:
            cleanup()


def build_parser():
    p = argparse.ArgumentParser(description="smeadband ultrasonic + tilt headband runtime (BCM pins <= 27)")

    # Ultrasonic distances
    p.add_argument("--mode", choices=["dual", "front", "back"], default="dual",
                   help="Which ultrasonic sensors to use for feedback.")
    p.add_argument("--warn-in", type=float, default=12.0,
                   help="Warn distance in inches (beyond this, vib stops).")
    p.add_argument("--close-in", type=float, default=4.0,
                   help="Close distance in inches (inside this, vib is continuous).")

    # Ultrasonic pins
    p.add_argument("--front-trig", type=int, default=DEFAULT_FRONT_TRIG)
    p.add_argument("--front-echo", type=int, default=DEFAULT_FRONT_ECHO)
    p.add_argument("--back-trig",  type=int, default=DEFAULT_BACK_TRIG)
    p.add_argument("--back-echo",  type=int, default=DEFAULT_BACK_ECHO)

    # Vibrator motors
    p.add_argument("--vib-1-pin", type=int, default=DEFAULT_VIB1_PIN)
    p.add_argument("--vib-2-pin", type=int, default=DEFAULT_VIB2_PIN)
    p.add_argument("--vib-active-high", action="store_true",
                   help="If set, vibrators are driven active-HIGH. Default assumes active-LOW modules.")
    p.add_argument("--vib-pulse-len", type=float, default=0.06,
                   help="On-time (seconds) for each vibration pulse.")
    p.add_argument("--vib-min-interval", type=float, default=0.08,
                   help="Minimum off-interval for vibration pulses (closest distance).")
    p.add_argument("--vib-max-interval", type=float, default=0.60,
                   help="Maximum off-interval for vibration pulses (furthest distance).")

    # Tilt + single buzzer
    p.add_argument("--tilt-pin", type=int, default=DEFAULT_TILT_PIN,
                   help="BCM pin for tilt SIG (e.g., 5). <0 disables tilt (default).")
    p.add_argument("--tilt-active-high", action="store_true",
                   help="If set, tilt switch is active-HIGH; default assumes active-LOW (SIG pulled low when tilted).")

    p.add_argument("--buzzer-pin", type=int, default=DEFAULT_TILT_BUZZERPIN,
                   help="Single buzzer pin used only for tilt feedback.")
    p.add_argument("--buzzer-active-high", action="store_true",
                   help="If set, buzzer is active-HIGH; default assumes active-LOW modules (your case).")

    p.add_argument("--tilt-beep-len", type=float, default=0.06,
                   help="On-time (seconds) for each tilt beep.")
    p.add_argument("--tilt-min-interval", type=float, default=0.08,
                   help="Minimum off-interval between tilt beeps at max tilt.")
    p.add_argument("--tilt-max-interval", type=float, default=0.80,
                   help="Maximum off-interval between tilt beeps just above threshold.")
    p.add_argument("--tilt-window", type=float, default=0.8,
                   help="Smoothing window (seconds) for tilt level (bigger = slower response).")
    p.add_argument("--tilt-threshold", type=float, default=0.20,
                   help="Tilt *intensity* threshold in [0,1]. 0–threshold: silent. Above: beeping with rising frequency.")

    # Debug
    p.add_argument("--print", action="store_true",
                   help="Print sensor distances and tilt info each cycle.")

    return p


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    run(args)
