"""
TEKNOFEST 2025 Su Altı Roket Aracı
Servo Kontrol Helper Modülü

Bu modül fin servolarının kontrol edilmesi için kullanılır.
MAVLink üzerinden Pixhawk servo çıkışlarını kontrol eder.
"""

import time
import logging
from typing import Dict, List, Optional
from .mavlink_helper import MAVLinkController

class ServoController:
    """Servo kontrol sınıfı"""
    
    def __init__(self, mavlink_controller: MAVLinkController, servo_config: Dict):
        """
        Servo controller'ı başlat
        
        Args:
            mavlink_controller: MAVLink bağlantı nesnesi
            servo_config: Servo konfigürasyon dict'i
        """
        self.mav = mavlink_controller
        self.servo_config = servo_config
        self.current_positions = {}
        
        # Servo sınırları
        self.servo_min = 1000
        self.servo_max = 2000
        self.servo_neutral = 1500
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
        # Tüm servolar nötr konumda başlasın
        self.initialize_servos()
    
    def initialize_servos(self):
        """Tüm servoları nötr konuma getir"""
        self.logger.info("Servolar nötr konuma getiriliyor...")
        for servo_name, config in self.servo_config.items():
            aux_port = config["aux_port"]
            self.set_servo_position(servo_name, self.servo_neutral)
            self.current_positions[servo_name] = self.servo_neutral
        time.sleep(1)  # Servoların konuma gelmesi için bekle
    
    def set_servo_position(self, servo_name: str, pwm_value: int) -> bool:
        """
        Belirli bir servoyu belirli konuma getir
        
        Args:
            servo_name: Servo adı (config'deki key)
            pwm_value: PWM değeri (1000-2000)
            
        Returns:
            bool: İşlem başarı durumu
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"Bilinmeyen servo: {servo_name}")
            return False
        
        # PWM değerini sınırla
        pwm_value = max(self.servo_min, min(self.servo_max, pwm_value))
        
        # Servo AUX port numarasını al
        aux_port = self.servo_config[servo_name]["aux_port"]
        
        # MAVLink üzerinden servo komutunu gönder
        self.mav.set_servo_pwm(aux_port, pwm_value)
        
        # Güncel konumu kaydet
        self.current_positions[servo_name] = pwm_value
        
        servo_desc = self.servo_config[servo_name]["name"]
        self.logger.debug(f"{servo_desc} (AUX{aux_port}): PWM {pwm_value}")
        
        return True
    
    def set_multiple_servos(self, servo_positions: Dict[str, int], delay: float = 0.01):
        """
        Birden fazla servoyu aynı anda kontrol et
        
        Args:
            servo_positions: {servo_name: pwm_value} dict'i
            delay: Komutlar arası gecikme
        """
        for servo_name, pwm_value in servo_positions.items():
            self.set_servo_position(servo_name, pwm_value)
            if delay > 0:
                time.sleep(delay)
    
    def move_servo_smooth(self, servo_name: str, target_pwm: int, duration: float = 1.0, steps: int = 20):
        """
        Servoyu yumuşak hareket ile hedefe götür
        
        Args:
            servo_name: Servo adı
            target_pwm: Hedef PWM değeri
            duration: Hareket süresi (saniye)
            steps: Hareket adım sayısı
        """
        if servo_name not in self.current_positions:
            self.logger.error(f"Servo {servo_name} başlatılmamış")
            return
        
        current_pwm = self.current_positions[servo_name]
        step_size = (target_pwm - current_pwm) / steps
        step_delay = duration / steps
        
        for i in range(steps + 1):
            pwm_value = int(current_pwm + (step_size * i))
            self.set_servo_position(servo_name, pwm_value)
            time.sleep(step_delay)
    
    def execute_movement_command(self, movement_commands: Dict[str, int]):
        """
        Hareket komutunu çalıştır (hardware_pinmap'ten gelen)
        
        Args:
            movement_commands: Fin pozisyonları dict'i
        """
        self.logger.info(f"Hareket komutu çalıştırılıyor: {list(movement_commands.keys())}")
        self.set_multiple_servos(movement_commands)
    
    def all_servos_neutral(self):
        """Tüm servoları nötr konuma getir"""
        self.logger.info("Tüm servolar nötr konuma getiriliyor")
        neutral_commands = {}
        for servo_name in self.servo_config.keys():
            neutral_commands[servo_name] = self.servo_neutral
        self.set_multiple_servos(neutral_commands)
    
    def test_servo_range(self, servo_name: str, test_duration: float = 3.0):
        """
        Belirli bir servoyu test et (min-max-neutral)
        
        Args:
            servo_name: Test edilecek servo adı
            test_duration: Her pozisyon için bekleme süresi
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"Bilinmeyen servo: {servo_name}")
            return
        
        servo_desc = self.servo_config[servo_name]["name"]
        self.logger.info(f"{servo_desc} test ediliyor...")
        
        # Test sırası: Neutral -> Min -> Neutral -> Max -> Neutral
        test_positions = [
            (self.servo_neutral, "Nötr"),
            (self.servo_min, "Minimum"),
            (self.servo_neutral, "Nötr"), 
            (self.servo_max, "Maksimum"),
            (self.servo_neutral, "Nötr")
        ]
        
        for pwm_value, position_name in test_positions:
            self.logger.info(f"  {servo_desc} -> {position_name} (PWM: {pwm_value})")
            self.set_servo_position(servo_name, pwm_value)
            time.sleep(test_duration)
        
        self.logger.info(f"{servo_desc} test tamamlandı")
    
    def test_all_servos(self, test_duration: float = 2.0):
        """Tüm servoları sırayla test et"""
        self.logger.info("Tüm servolar test ediliyor...")
        
        for servo_name in self.servo_config.keys():
            self.test_servo_range(servo_name, test_duration)
            time.sleep(0.5)  # Servo testleri arası kısa mola
        
        self.logger.info("Tüm servo testleri tamamlandı")
    
    def get_current_positions(self) -> Dict[str, int]:
        """Güncel servo konumlarını al"""
        return self.current_positions.copy()
    
    def get_servo_status(self) -> Dict[str, Dict]:
        """Tüm servoların durum bilgisini al"""
        status = {}
        for servo_name, config in self.servo_config.items():
            status[servo_name] = {
                "name": config["name"],
                "aux_port": config["aux_port"],
                "current_pwm": self.current_positions.get(servo_name, 0),
                "description": config["description"]
            }
        return status
    
    def emergency_stop(self):
        """Acil durdurma - tüm servolar nötr"""
        self.logger.warning("SERVO ACİL DURDURMA!")
        self.all_servos_neutral()
    
    def calibrate_servo_limits(self, servo_name: str):
        """
        Manuel servo limit kalibrasyonu
        Kullanıcı etkileşimiyle servo limitlerini ayarla
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"Bilinmeyen servo: {servo_name}")
            return
        
        servo_desc = self.servo_config[servo_name]["name"]
        print(f"\n{servo_desc} Kalibrasyon Modu")
        print("Komutlar: 'min', 'max', 'neutral', 'test', 'done'")
        
        while True:
            command = input(f"{servo_desc} komutu: ").strip().lower()
            
            if command == 'min':
                self.set_servo_position(servo_name, self.servo_min)
                print(f"Minimum pozisyon: {self.servo_min}")
                
            elif command == 'max':
                self.set_servo_position(servo_name, self.servo_max)
                print(f"Maksimum pozisyon: {self.servo_max}")
                
            elif command == 'neutral':
                self.set_servo_position(servo_name, self.servo_neutral)
                print(f"Nötr pozisyon: {self.servo_neutral}")
                
            elif command == 'test':
                self.test_servo_range(servo_name, 1.5)
                
            elif command == 'done':
                self.set_servo_position(servo_name, self.servo_neutral)
                print(f"{servo_desc} kalibrasyonu tamamlandı")
                break
                
            else:
                print("Geçersiz komut!")

# Kullanım örneği (test amaçlı)
if __name__ == "__main__":
    # Bu kısım gerçek kullanımda import edilir
    pass
