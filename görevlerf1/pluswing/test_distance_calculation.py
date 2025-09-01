#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GELİŞMİŞ MESAFE HESAPLAMA TESTİ
Yeni gelişmiş mesafe hesaplaması sistemini test eder
"""

import sys
import time

# GPIO olmadan config import
sys.path.append('/Users/onurtas/2025-TEKNOFEST-Su-Alti-Roket-Araci/görevlerf1/pluswing')

# Config sabitlerini doğrudan tanımla (GPIO olmadan)
SPEED_SLOW = 1600
SPEED_MEDIUM = 1700
SPEED_FAST = 1800
MOTOR_STOP = 1500
MOTOR_FORWARD_MIN = 1550
MOTOR_FORWARD_MAX = 1900

# Mesafe hesaplama sabitleri
ESTIMATED_SPEED_SLOW = 1.0
ESTIMATED_SPEED_MEDIUM = 1.5
ESTIMATED_SPEED_FAST = 2.0

# Motor karakteristik eğrisi
MOTOR_SPEED_CURVE = {
    MOTOR_STOP: 0.0,
    MOTOR_FORWARD_MIN: 0.2,
    SPEED_SLOW: 1.0,
    SPEED_MEDIUM: 1.5,
    SPEED_FAST: 2.0,
    MOTOR_FORWARD_MAX: 2.5
}

# Motor dinamik özellikleri
MOTOR_STARTUP_DELAY = 0.5
MOTOR_ACCELERATION_TIME = 2.0
MOTOR_DECELERATION_TIME = 1.5

# Çevresel faktörler
WATER_DRAG_COEFFICIENT = 0.3
FRICTION_COEFFICIENT = 0.1

# Hız yumuşatma
SPEED_SMOOTHING_FACTOR = 0.3

# Mesafe hesaplama modu
DISTANCE_CALC_MODE = "ADVANCED"

# Basit mesafe hesaplaması fonksiyonu
def estimate_distance_simple(speed_pwm, elapsed_time):
    """Basit hız hesaplama (eski yöntem)"""
    if speed_pwm <= MOTOR_STOP:
        estimated_speed = 0
    elif speed_pwm <= SPEED_SLOW:
        estimated_speed = ESTIMATED_SPEED_SLOW
    elif speed_pwm <= SPEED_MEDIUM:
        estimated_speed = ESTIMATED_SPEED_MEDIUM
    else:
        estimated_speed = ESTIMATED_SPEED_FAST

    return estimated_speed * elapsed_time

# clamp fonksiyonu
def clamp(value, min_val, max_val):
    """Değeri belirtilen aralıkta sınırla"""
    if value is None:
        return (min_val + max_val) / 2
    return max(min_val, min(max_val, value))

# Gelişmiş hesaplayıcı sınıfı
class AdvancedDistanceCalculator:
    """Gelişmiş mesafe hesaplayıcı sınıfı"""

    def __init__(self):
        self.last_speed = 0.0
        self.last_pwm = MOTOR_STOP
        self.speed_history = []
        self.time_history = []
        self.max_history_size = 50

    def get_speed_from_pwm_curve(self, pwm_value):
        """PWM değerini hız karakteristik eğrisinden hesapla"""
        if pwm_value <= MOTOR_STOP:
            return 0.0

        pwm_points = sorted(MOTOR_SPEED_CURVE.keys())
        speed_points = [MOTOR_SPEED_CURVE[pwm] for pwm in pwm_points]

        for i in range(len(pwm_points) - 1):
            if pwm_points[i] <= pwm_value <= pwm_points[i + 1]:
                pwm1, pwm2 = pwm_points[i], pwm_points[i + 1]
                speed1, speed2 = speed_points[i], speed_points[i + 1]

                if pwm2 != pwm1:
                    speed = speed1 + (pwm_value - pwm1) * (speed2 - speed1) / (pwm2 - pwm1)
                else:
                    speed = speed1
                return speed

        return max(speed_points)

    def apply_motor_dynamics(self, target_speed, elapsed_time):
        """Motor dinamiklerini uygula"""
        if elapsed_time < MOTOR_STARTUP_DELAY:
            return 0.0

        speed_diff = target_speed - self.last_speed

        if speed_diff > 0:
            max_speed_change = elapsed_time / MOTOR_ACCELERATION_TIME * target_speed
            actual_speed_change = min(speed_diff, max_speed_change)
        else:
            max_speed_change = elapsed_time / MOTOR_DECELERATION_TIME * abs(target_speed)
            actual_speed_change = max(speed_diff, -max_speed_change)

        current_speed = self.last_speed + actual_speed_change
        current_speed = max(0.0, current_speed)

        return current_speed

    def apply_environmental_factors(self, speed, elapsed_time):
        """Çevresel faktörleri uygula"""
        if speed <= 0:
            return 0.0

        drag_force = WATER_DRAG_COEFFICIENT * speed * speed
        friction_force = FRICTION_COEFFICIENT * speed
        total_resistance = drag_force + friction_force
        resistance_factor = max(0.1, 1.0 - total_resistance / 10.0)

        return speed * resistance_factor

    def smooth_speed(self, new_speed):
        """Hız değerini yumuşat"""
        if not self.speed_history:
            self.speed_history.append(new_speed)
            return new_speed

        smoothed = (SPEED_SMOOTHING_FACTOR * new_speed +
                   (1 - SPEED_SMOOTHING_FACTOR) * self.speed_history[-1])

        self.speed_history.append(smoothed)

        if len(self.speed_history) > self.max_history_size:
            self.speed_history.pop(0)

        return smoothed

    def calculate_distance(self, pwm_value, elapsed_time):
        """Tam gelişmiş mesafe hesaplaması"""
        target_speed = self.get_speed_from_pwm_curve(pwm_value)
        actual_speed = self.apply_motor_dynamics(target_speed, elapsed_time)
        corrected_speed = self.apply_environmental_factors(actual_speed, elapsed_time)
        smoothed_speed = self.smooth_speed(corrected_speed)
        distance = smoothed_speed * elapsed_time

        self.last_speed = smoothed_speed
        self.last_pwm = pwm_value

        return distance

    def reset(self):
        """Hesaplayıcıyı sıfırla"""
        self.last_speed = 0.0
        self.last_pwm = MOTOR_STOP
        self.speed_history.clear()
        self.time_history.clear()

# Global hesaplayıcı
_advanced_calculator = AdvancedDistanceCalculator()

def estimate_distance_advanced(speed_pwm, elapsed_time):
    """Gelişmiş mesafe hesaplaması"""
    global _advanced_calculator

    if speed_pwm is None or elapsed_time is None or elapsed_time <= 0:
        return 0.0

    pwm_clamped = clamp(speed_pwm, MOTOR_STOP, MOTOR_FORWARD_MAX)

    try:
        distance = _advanced_calculator.calculate_distance(pwm_clamped, elapsed_time)
        return max(0.0, distance)
    except Exception as e:
        print(f"Gelişmiş mesafe hesaplaması hatası: {e}")
        return estimate_distance_simple(speed_pwm, elapsed_time)

def test_basic_distance_calculation():
    """Temel mesafe hesaplaması testi"""
    print("=" * 60)
    print("TEMEL MESAFE HESAPLAMA TESTİ")
    print("=" * 60)

    test_cases = [
        (SPEED_SLOW, 10.0, "Yavaş Hız"),
        (SPEED_MEDIUM, 15.0, "Orta Hız"),
        (SPEED_FAST, 20.0, "Hızlı Hız"),
        (MOTOR_FORWARD_MIN, 5.0, "Minimum Hız"),
        (MOTOR_FORWARD_MAX, 25.0, "Maksimum Hız"),
    ]

    for pwm, time_sec, description in test_cases:
        # Basit yöntem
        simple_dist = estimate_distance_simple(pwm, time_sec)

        # Gelişmiş yöntem
        advanced_dist = estimate_distance_advanced(pwm, time_sec)

        print(f"{description:15} | PWM: {pwm:4} | Süre: {time_sec:4.1f}s | "
              f"Basit: {simple_dist:5.2f}m | Gelişmiş: {advanced_dist:5.2f}m")

def test_motor_characteristics():
    """Motor karakteristik eğrisi testi"""
    print("\n" + "=" * 60)
    print("MOTOR KARAKTERİSTİK EĞRİSİ TESTİ")
    print("=" * 60)

    # Motor hız eğrisini test et
    print("PWM | Hız (m/s) | Açıklama")
    print("-" * 35)

    for pwm in sorted(MOTOR_SPEED_CURVE.keys()):
        speed = MOTOR_SPEED_CURVE[pwm]
        description = ""
        if pwm == MOTOR_STOP:
            description = "DUR"
        elif pwm == MOTOR_FORWARD_MIN:
            description = "BAŞLANGIÇ"
        elif pwm == SPEED_SLOW:
            description = "YAVAŞ"
        elif pwm == SPEED_MEDIUM:
            description = "ORTA"
        elif pwm == SPEED_FAST:
            description = "HIZLI"
        elif pwm == MOTOR_FORWARD_MAX:
            description = "MAKSİMUM"

        print(f"{pwm:4} | {speed:8.2f} | {description}")

def test_interpolation():
    """Doğrusal interpolasyon testi"""
    print("\n" + "=" * 60)
    print("DOĞRUSAL İNTERPOLASYON TESTİ")
    print("=" * 60)

    test_pwms = [1575, 1650, 1725, 1800, 1850]

    print("PWM  | Hedef Hız | Hesaplanan | Fark")
    print("-" * 40)

    for pwm in test_pwms:
        # Basit yöntem
        simple_speed = estimate_distance_simple(pwm, 1.0)

        # Gelişmiş yöntem (1 saniye için)
        _advanced_calculator.reset()
        advanced_speed = estimate_distance_advanced(pwm, 1.0)

        diff = advanced_speed - simple_speed

        print(f"{pwm:4} | {simple_speed:9.2f} | {advanced_speed:10.2f} | {diff:+6.2f}")

def test_motor_dynamics():
    """Motor dinamikleri testi"""
    print("\n" + "=" * 60)
    print("MOTOR DİNAMİKLERİ TESTİ")
    print("=" * 60)

    pwm = SPEED_FAST
    test_times = [0.5, 1.0, 2.0, 5.0, 10.0]

    print("Süre(s) | Basit Mesafe | Gelişmiş Mesafe | Fark")
    print("-" * 50)

    for t in test_times:
        # Hesaplayıcıyı sıfırla
        _advanced_calculator.reset()

        # Basit yöntem
        simple_dist = estimate_distance_simple(pwm, t)

        # Gelişmiş yöntem
        advanced_dist = estimate_distance_advanced(pwm, t)

        diff = advanced_dist - simple_dist

        print(f"{t:7.1f} | {simple_dist:12.2f} | {advanced_dist:14.2f} | {diff:+6.2f}")

def test_environmental_factors():
    """Çevresel faktörler testi"""
    print("\n" + "=" * 60)
    print("ÇEVRESEL FAKTÖRLER TESTİ")
    print("=" * 60)

    pwm = SPEED_MEDIUM
    time_sec = 10.0

    # Hesaplayıcıyı sıfırla
    _advanced_calculator.reset()

    # Temiz hız (çevresel faktörler olmadan)
    clean_speed = estimate_distance_advanced(pwm, time_sec)

    # Çevresel faktörler ile
    environmental_dist = estimate_distance_advanced(pwm, time_sec)

    # Direnç etkisi
    resistance_factor = WATER_DRAG_COEFFICIENT * (clean_speed / time_sec) ** 2
    efficiency = max(0.1, 1.0 - resistance_factor / 10.0)

    print(f"Temiz Hız: {clean_speed/time_sec:.2f} m/s")
    print(f"Çevresel Hız: {environmental_dist/time_sec:.2f} m/s")
    print(f"Verimlilik: {efficiency:.1%}")
    print(f"Direnç Katsayısı: {resistance_factor:.3f}")

def test_smoothing():
    """Hız yumuşatma testi"""
    print("\n" + "=" * 60)
    print("HIZ YUMUŞATMA TESTİ")
    print("=" * 60)

    pwm = SPEED_MEDIUM
    time_sec = 1.0

    print("Ölçüm | Ham Hız | Yumuşatılmış")
    print("-" * 30)

    # Hesaplayıcıyı sıfırla
    _advanced_calculator.reset()

    # Birkaç ölçüm yap
    for i in range(10):
        # Her ölçümde hafif farklı PWM simüle et
        test_pwm = pwm + (i - 5) * 5  # ±25 PWM varyasyonu

        # Gelişmiş hesaplama
        distance = estimate_distance_advanced(test_pwm, time_sec)
        current_speed = _advanced_calculator.last_speed

        # Son yumuşatılmış hız
        smoothed_speed = _advanced_calculator.speed_history[-1] if _advanced_calculator.speed_history else 0

        print(f"{i+1:6} | {current_speed:7.2f} | {smoothed_speed:11.2f}")

def main():
    """Ana test fonksiyonu"""
    print("🚀 GELİŞMİŞ MESAFE HESAPLAMA TESTİ")
    print(f"Mesafe Hesaplama Modu: {DISTANCE_CALC_MODE}")
    print("=" * 60)

    try:
        # Temel testler
        test_basic_distance_calculation()
        test_motor_characteristics()
        test_interpolation()
        test_motor_dynamics()
        test_environmental_factors()
        test_smoothing()

        print("\n" + "=" * 60)
        print("✅ TÜM TESTLER TAMAMLANDI")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ TEST HATASI: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
