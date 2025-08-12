#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Türkçe Terminal Arayüzü (CANLI KONTROL)

Kontroller:
- W / S : Pitch (Eğim) ↑ / ↓
- A / D : Roll (Yatma) ← / →
- Q / E : Yaw (Dönüş) sol / sağ
- O / L : Motor gücü +%5 / -%5
- I / K : Derinlik hedefi +0.2m / -0.2m (D300 varsa)
- X     : Tüm servo komutlarını sıfırla (Roll/Pitch/Yaw = 0)
- Space : ARM / DISARM
- H     : Yardım
- ESC   : Çıkış

Ekranda Gösterilen Veriler:
- MAVLink bağlantı ve ARM durumu
- IMU: Roll / Pitch / Yaw (derece)
- D300: Derinlik (metre) ve hedef (varsa)
- Motor gücü (%)
"""
import curses
import time

from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


class TRTerminal:
    def __init__(self):
        self.cfg = load_config()
        self.mav = SerialMAV(self.cfg)
        assert self.mav.connect(), 'MAVLink bağlantısı başarısız'
        # ARM denemesi otomatik yapılmaz; kullanıcı space ile yapar

        # Komut durumları (derece)
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0
        self.yaw_cmd = 0.0
        self.motor_pct = 0.0

        # Derinlik hedefi (D300) opsiyonel
        self.d300 = D300Sensor(self.cfg)
        self.d300.connect()
        self.depth_target = None  # metre

        # Çalışma bayrağı
        self.running = True
        self.armed = False

    def arm_toggle(self):
        if self.armed:
            self.mav.disarm()
            self.armed = False
        else:
            if self.mav.arm():
                self.armed = True

    def apply_controls(self):
        # Derinlik hedefi varsa basit P kontrol ile pitch'e katkı uygula
        pitch_correction = 0.0
        if self.depth_target is not None and self.d300.available:
            data = self.d300.get() or {}
            depth_now = data.get('depth_m', None)
            if depth_now is not None:
                err = self.depth_target - depth_now
                pitch_correction = max(-10.0, min(10.0, err * 6.0))

        pitch_to_send = self.pitch_cmd + pitch_correction

        # Servo komutlarını gönder
        xwing_mix_and_send(self.mav, self.cfg, self.roll_cmd, pitch_to_send, self.yaw_cmd)

        # Motor PWM gönder
        motor_ch = 8 + self.cfg['pixhawk']['servo_channels']['motor']
        pwm = motor_percent_to_pwm(self.cfg, self.motor_pct)
        self.mav.set_servo_pwm(motor_ch, pwm)

    def draw(self, stdscr):
        stdscr.erase()
        stdscr.nodelay(True)
        curses.curs_set(0)

        # Başlık
        stdscr.addstr(0, 2, '🚀 TEKNOFEST ROV - Türkçe Terminal Kontrol', curses.A_BOLD)

        # Durum satırı
        status = 'ARMED' if self.armed else 'DISARMED'
        stdscr.addstr(1, 2, f'Bağlantı: ✅  | Durum: {status}  | Motor: {self.motor_pct:+.0f}%')

        # IMU verisi
        att = self.mav.get_attitude()
        if att:
            r, p, y = [v * 180.0 / 3.14159 for v in att]
            stdscr.addstr(3, 2, f'IMU  Roll: {r:+6.1f}°   Pitch: {p:+6.1f}°   Yaw: {y:+6.1f}°')
        else:
            stdscr.addstr(3, 2, 'IMU  veri alınamıyor')

        # D300 verisi
        if self.d300.available:
            data = self.d300.get() or {}
            depth = data.get('depth_m', None)
            if depth is not None:
                line = f'D300 Derinlik: {depth:5.2f} m'
                if self.depth_target is not None:
                    line += f'  | Hedef: {self.depth_target:5.2f} m'
                stdscr.addstr(4, 2, line)
            else:
                stdscr.addstr(4, 2, 'D300 veri alınamıyor')
        else:
            stdscr.addstr(4, 2, 'D300 bağlı değil (simülasyon veya kapalı)')

        # Komut değerleri
        stdscr.addstr(6, 2, f'Komutlar  Roll: {self.roll_cmd:+5.1f}°  Pitch: {self.pitch_cmd:+5.1f}°  Yaw: {self.yaw_cmd:+5.1f}°')

        # Yardım
        stdscr.addstr(8, 2, 'Tuşlar: W/S=Pitch  A/D=Roll  Q/E=Yaw  O/L=Motor ±%5  I/K=Derinlik hedef ±0.2m  X=Reset  Space=ARM  H=Yardım  ESC=Çıkış')

        stdscr.refresh()

    def help_popup(self, stdscr):
        h = [
            'YARDIM:',
            'W/S : Pitch (Eğim) ↑ / ↓',
            'A/D : Roll (Yatma) ← / →',
            'Q/E : Yaw (Dönüş) sol / sağ',
            'O/L : Motor gücü +%5 / -%5',
            'I/K : Derinlik hedefi +0.2m / -0.2m (D300 varsa)',
            'X   : Tüm servo komutlarını sıfırla',
            'Space: ARM / DISARM',
            'ESC : Çıkış'
        ]
        rows, cols = stdscr.getmaxyx()
        top = 10
        left = 4
        for i, line in enumerate(h):
            if top + i < rows - 1:
                stdscr.addstr(top + i, left, line)
        stdscr.refresh()
        time.sleep(3)

    def loop(self, stdscr):
        last_ui = 0
        while self.running:
            # Klavye
            try:
                key = stdscr.getch()
            except Exception:
                key = -1

            if key != -1:
                if key in (27,):  # ESC
                    self.running = False
                    break
                elif key in (ord(' '),):
                    self.arm_toggle()
                elif key in (ord('w'), ord('W')):
                    self.pitch_cmd = min(45.0, self.pitch_cmd + 2.0)
                elif key in (ord('s'), ord('S')):
                    self.pitch_cmd = max(-45.0, self.pitch_cmd - 2.0)
                elif key in (ord('a'), ord('A')):
                    self.roll_cmd = min(45.0, self.roll_cmd + 2.0)
                elif key in (ord('d'), ord('D')):
                    self.roll_cmd = max(-45.0, self.roll_cmd - 2.0)
                elif key in (ord('q'), ord('Q')):
                    self.yaw_cmd = min(45.0, self.yaw_cmd + 3.0)
                elif key in (ord('e'), ord('E')):
                    self.yaw_cmd = max(-45.0, self.yaw_cmd - 3.0)
                elif key in (ord('x'), ord('X')):
                    self.roll_cmd = self.pitch_cmd = self.yaw_cmd = 0.0
                elif key in (ord('o'), ord('O')):
                    self.motor_pct = min(100.0, self.motor_pct + 5.0)
                elif key in (ord('l'), ord('L')):
                    self.motor_pct = max(-100.0, self.motor_pct - 5.0)
                elif key in (ord('i'), ord('I')):
                    base = 0.0 if self.depth_target is None else self.depth_target
                    self.depth_target = max(0.3, base + 0.2)
                elif key in (ord('k'), ord('K')):
                    base = 0.0 if self.depth_target is None else self.depth_target
                    self.depth_target = max(0.3, base - 0.2)
                elif key in (ord('h'), ord('H')):
                    self.help_popup(stdscr)

            # Kontrolleri uygula (ARM değilse sadece gönderir ama araç tepkisiz olabilir)
            self.apply_controls()

            # UI 20Hz
            now = time.time()
            if now - last_ui > 0.05:
                self.draw(stdscr)
                last_ui = now

            time.sleep(0.01)

        # Çıkış öncesi nötrle
        xwing_mix_and_send(self.mav, self.cfg, 0, 0, 0)
        motor_ch = 8 + self.cfg['pixhawk']['servo_channels']['motor']
        self.mav.set_servo_pwm(motor_ch, self.cfg['pixhawk']['pwm']['neutral'])
        if self.armed:
            self.mav.disarm()


def main():
    curses.wrapper(lambda stdscr: TRTerminal().loop(stdscr))


if __name__ == '__main__':
    main()


