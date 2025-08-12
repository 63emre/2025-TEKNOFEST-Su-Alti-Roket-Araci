#!/usr/bin/env python3
import os
import json
import time
import math
import threading

from pymavlink import mavutil


def load_config(config_path: str = os.path.join(os.path.dirname(__file__), 'config.json')) -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        return json.load(f)


class SerialMAV:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.master = None
        self.connected = False
        port = os.getenv('MAV_ADDRESS', cfg['mavlink']['port'])
        baud = int(os.getenv('MAV_BAUD', str(cfg['mavlink']['baud'])))
        self.port = port
        self.baud = baud
        self.timeout = cfg['mavlink'].get('timeout', 10)

    def connect(self) -> bool:
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud, autoreconnect=True)
            hb = self.master.wait_heartbeat(timeout=self.timeout)
            self.connected = hb is not None
            return self.connected
        except Exception:
            self.connected = False
            return False

    def arm(self) -> bool:
        if not self.connected:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1)
            return True
        except Exception:
            return False

    def disarm(self) -> bool:
        if not self.connected:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1)
            return True
        except Exception:
            return False

    def set_servo_pwm(self, channel: int, pwm: int) -> bool:
        if not self.connected:
            return False
        pwm = max(1000, min(2000, int(pwm)))
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, channel, pwm, 0, 0, 0, 0, 0
            )
            return True
        except Exception:
            return False

    def get_attitude(self):
        if not self.connected:
            return None
        msg = self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
        if not msg:
            return None
        return (msg.roll, msg.pitch, msg.yaw)


def xwing_mix_and_send(mav: SerialMAV, cfg: dict, roll: float, pitch: float, yaw: float) -> bool:
    """Inputs are angles in degrees. Mix to 4 fins and send PWM.
    Mapping per HARDWARE_PIN_MAPPING.md: AUX1,3,4,5.
    """
    ch = cfg['pixhawk']['servo_channels']
    pwm_cfg = cfg['pixhawk']['pwm']
    neutral = pwm_cfg['neutral']

    # Simple proportional mixing (scale to microseconds)
    # Gains tuned conservatively; will be adjusted by PID tuner
    roll_us = roll * 10
    pitch_us = pitch * 8
    yaw_us = yaw * 6

    fl = neutral + int(pitch_us + roll_us + yaw_us)   # AUX1
    fr = neutral + int(pitch_us - roll_us - yaw_us)   # AUX3
    rl = neutral + int(-pitch_us + roll_us - yaw_us)  # AUX4
    rr = neutral + int(-pitch_us - roll_us + yaw_us)  # AUX5

    ok = True
    ok &= mav.set_servo_pwm(8 + ch['fin_front_left'], fl)   # MAVLink SERVO1.. → channel = 8+aux
    ok &= mav.set_servo_pwm(8 + ch['fin_front_right'], fr)
    ok &= mav.set_servo_pwm(8 + ch['fin_rear_left'], rl)
    ok &= mav.set_servo_pwm(8 + ch['fin_rear_right'], rr)
    return ok


def motor_percent_to_pwm(cfg: dict, percent: float) -> int:
    pwm_cfg = cfg['pixhawk']['pwm']
    neutral = pwm_cfg['neutral']
    span = (pwm_cfg['max'] - pwm_cfg['min']) // 2
    return max(pwm_cfg['min'], min(pwm_cfg['max'], neutral + int(percent * span / 100.0)))


class D300Sensor:
    """Wrapper for App.depth_sensor.D300DepthSensor with safe fallback."""
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.sensor = None
        self.available = False
        # Try to import App.depth_sensor
        try:
            import sys
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            if repo_root not in sys.path:
                sys.path.append(repo_root)
            from App.depth_sensor import D300DepthSensor  # type: ignore
            self.sensor = D300DepthSensor(simulation_mode=False)
        except Exception:
            self.sensor = None

    def connect(self) -> bool:
        try:
            if self.sensor and self.sensor.connect():
                try:
                    # Start monitoring for continuous updates
                    self.sensor.start_monitoring(interval=0.1)
                except Exception:
                    pass
                self.available = True
                return True
        except Exception:
            pass
        # Fallback simulation
        try:
            if self.sensor:
                # Recreate in simulation mode
                from App.depth_sensor import D300DepthSensor  # type: ignore
                self.sensor = D300DepthSensor(simulation_mode=True)
                self.sensor.connect()
                self.sensor.start_monitoring(interval=0.1)
                self.available = True
                return True
        except Exception:
            pass
        self.available = False
        return False

    def get(self):
        if not self.sensor:
            return None
        try:
            return self.sensor.get_sensor_data()
        except Exception:
            return None



