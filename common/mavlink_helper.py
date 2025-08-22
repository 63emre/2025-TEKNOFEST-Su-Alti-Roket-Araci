"""
TEKNOFEST 2025 Su Altı Roket Aracı
MAVLink Bağlantı ve Kontrol Helper Modülü

Bu modül Pixhawk ile MAVLink protokolü üzerinden haberleşme sağlar.
"""

import time
import logging
from pymavlink import mavutil
import threading
from typing import Optional, Dict, Any

class MAVLinkController:
    """Pixhawk MAVLink bağlantı ve kontrol sınıfı"""
    
    def __init__(self, connection_string: str, baud_rate: int = 115200):
        """
        MAVLink bağlantısını başlat
        
        Args:
            connection_string: Bağlantı string'i (örn: /dev/ttyACM0)
            baud_rate: Baud hızı
        """
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.master = None
        self.connected = False
        self.last_heartbeat = 0
        self.attitude_data = {"roll": 0, "pitch": 0, "yaw": 0}
        self.distance_data = {"distance": 0}
        self.battery_data = {"voltage": 0, "current": 0, "remaining": 100}
        self.gps_data = {"lat": 0, "lon": 0, "alt": 0}
        
        # Logging ayarları
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Veri okuma thread'i için kontrol
        self._stop_reading = threading.Event()
        self._reading_thread = None
        
    def connect(self) -> bool:
        """
        Pixhawk'a bağlan
        
        Returns:
            bool: Bağlantı durumu
        """
        try:
            self.logger.info(f"Pixhawk'a bağlanılıyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baud_rate
            )
            
            # Heartbeat bekle
            self.logger.info("Heartbeat bekleniyor...")
            self.master.wait_heartbeat()
            self.logger.info("Pixhawk bağlantısı başarılı!")
            
            self.connected = True
            self.last_heartbeat = time.time()
            
            # Veri okuma thread'ini başlat
            self.start_data_reading()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Bağlantı hatası: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        self.connected = False
        self.stop_data_reading()
        if self.master:
            self.master.close()
            self.logger.info("Pixhawk bağlantısı kapatıldı")
    
    def start_data_reading(self):
        """Veri okuma thread'ini başlat"""
        if not self._reading_thread or not self._reading_thread.is_alive():
            self._stop_reading.clear()
            self._reading_thread = threading.Thread(target=self._read_data_loop)
            self._reading_thread.daemon = True
            self._reading_thread.start()
            self.logger.info("Veri okuma thread'i başlatıldı")
    
    def stop_data_reading(self):
        """Veri okuma thread'ini durdur"""
        self._stop_reading.set()
        if self._reading_thread:
            self._reading_thread.join(timeout=2)
            self.logger.info("Veri okuma thread'i durduruldu")
    
    def _read_data_loop(self):
        """Sürekli veri okuma döngüsü"""
        while not self._stop_reading.is_set() and self.connected:
            try:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    self._process_message(msg)
                time.sleep(0.01)  # 100Hz okuma hızı
            except Exception as e:
                self.logger.error(f"Veri okuma hatası: {e}")
                time.sleep(0.1)
    
    def _process_message(self, msg):
        """Gelen MAVLink mesajını işle"""
        if msg.get_type() == 'HEARTBEAT':
            self.last_heartbeat = time.time()
            
        elif msg.get_type() == 'ATTITUDE':
            # Roll, Pitch, Yaw verileri (radyan -> derece)
            self.attitude_data = {
                "roll": msg.roll * 57.2958,    # Radyan to derece
                "pitch": msg.pitch * 57.2958,
                "yaw": msg.yaw * 57.2958
            }
            
        elif msg.get_type() == 'BATTERY_STATUS':
            # Batarya durumu
            if hasattr(msg, 'voltages') and len(msg.voltages) > 0:
                self.battery_data = {
                    "voltage": msg.voltages[0] / 1000.0,  # mV to V
                    "current": msg.current_battery / 100.0 if msg.current_battery != -1 else 0,
                    "remaining": msg.battery_remaining
                }
        
        elif msg.get_type() == 'DISTANCE_SENSOR':
            # Mesafe sensörü
            self.distance_data = {
                "distance": msg.current_distance / 100.0  # cm to m
            }
            
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            # GPS verisi (GPS yoksa 0 olacak)
            self.gps_data = {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7, 
                "alt": msg.alt / 1000.0
            }
    
    def get_attitude(self) -> Dict[str, float]:
        """Roll, pitch, yaw verilerini al"""
        return self.attitude_data.copy()
    
    def get_distance(self) -> float:
        """Mesafe sensörü verisini al"""
        return self.distance_data["distance"]
    
    def get_battery_status(self) -> Dict[str, float]:
        """Batarya durumunu al"""
        return self.battery_data.copy()
    
    def is_connected(self) -> bool:
        """Bağlantı durumunu kontrol et"""
        if not self.connected:
            return False
        
        # Son heartbeat'ten 3 saniye geçtiyse bağlantı kopmuş sayılır
        return (time.time() - self.last_heartbeat) < 3.0
    
    def set_servo_pwm(self, servo_num: int, pwm_value: int):
        """
        Servo PWM değerini ayarla
        
        Args:
            servo_num: Servo numarası (1-8 arası AUX çıkışları)
            pwm_value: PWM değeri (1000-2000)
        """
        if not self.is_connected():
            self.logger.warning("Pixhawk bağlı değil, servo komutu gönderilemiyor")
            return
        
        # PWM değerini sınırla
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                servo_num,  # param1: servo number
                pwm_value,  # param2: PWM value
                0, 0, 0, 0, 0  # param3-7: unused
            )
            
            self.logger.debug(f"Servo {servo_num} PWM: {pwm_value}")
            
        except Exception as e:
            self.logger.error(f"Servo komutu gönderme hatası: {e}")
    
    def set_motor_speed(self, speed_percent: float):
        """
        Motor hızını ayarla
        
        Args:
            speed_percent: Hız yüzdesi (0-100)
        """
        # Yüzdeyi PWM değerine çevir (1000-2000 arası)
        speed_percent = max(0, min(100, speed_percent))
        pwm_value = int(1000 + (speed_percent * 10))  # 0% = 1000, 100% = 2000
        
        self.set_servo_pwm(1, pwm_value)  # Motor AUX 1'de
        self.logger.info(f"Motor hızı: %{speed_percent} (PWM: {pwm_value})")
    
    def emergency_stop(self):
        """Acil durdurma - tüm servolar nötr, motor durdur"""
        self.logger.warning("ACİL DURDURMA AKTİF!")
        
        # Motor durdur
        self.set_motor_speed(0)
        
        # Tüm servolar nötr konuma
        for servo_num in [3, 4, 5, 6]:  # AUX 3-6 fin servolarımız
            self.set_servo_pwm(servo_num, 1500)
            
        time.sleep(0.1)  # Komutların gönderilmesi için bekle
    
    def arm_vehicle(self) -> bool:
        """Aracı arm et"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # param1: arm
                0, 0, 0, 0, 0, 0  # param2-7: unused
            )
            self.logger.info("Araç arm edildi")
            return True
        except Exception as e:
            self.logger.error(f"Arm etme hatası: {e}")
            return False
    
    def disarm_vehicle(self) -> bool:
        """Aracı disarm et"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # param1: disarm
                0, 0, 0, 0, 0, 0  # param2-7: unused
            )
            self.logger.info("Araç disarm edildi")
            return True
        except Exception as e:
            self.logger.error(f"Disarm etme hatası: {e}")
            return False
    
    def get_status_summary(self) -> Dict[str, Any]:
        """Durum özetini al"""
        return {
            "connected": self.is_connected(),
            "attitude": self.get_attitude(),
            "distance": self.get_distance(),
            "battery": self.get_battery_status(),
            "last_heartbeat": self.last_heartbeat
        }

# Kullanım örneği
if __name__ == "__main__":
    import os
    
    # Bağlantı ayarları
    connection_string = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    
    # MAVLink controller oluştur
    mav = MAVLinkController(connection_string, baud_rate)
    
    try:
        # Bağlan
        if mav.connect():
            print("Pixhawk'a başarıyla bağlanıldı!")
            
            # 10 saniye boyunca veri oku
            start_time = time.time()
            while time.time() - start_time < 10:
                status = mav.get_status_summary()
                print(f"Attitude: {status['attitude']}")
                print(f"Distance: {status['distance']} m")
                print(f"Battery: {status['battery']}")
                print("-" * 40)
                time.sleep(1)
                
        else:
            print("Pixhawk bağlantısı başarısız!")
            
    finally:
        mav.disconnect()
