"""
TEKNOFEST 2025 Su Altı Roket Aracı
Buton Sistemi - Otonom Görev Tetikleme

Bu modül sistem kontrol butonunu yönetir ve otonom görevleri tetikler.
"""

import os
import sys
import time
import threading
import subprocess
import logging
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass
from enum import Enum

# GPIO helper import
try:
    from .gpio_helper import GPIOController
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    from gpio_helper import GPIOController

class MissionType(Enum):
    """Görev türleri"""
    AUTONOMOUS = "autonomous"
    MANUAL = "manual"

@dataclass
class MissionConfig:
    """Görev konfigürasyonu"""
    name: str
    script_path: str
    mission_type: MissionType
    wing_type: str  # "x_wing" veya "+_wing"
    description: str
    duration_estimate: int  # Tahmini süre (saniye)

class ButtonSystem:
    """Buton sistemi yönetici sınıfı"""
    
    def __init__(self, wing_type: str = "x_wing"):
        """
        Buton sistemini başlat
        
        Args:
            wing_type: Kanat tipi ("x_wing" veya "+_wing")
        """
        self.wing_type = wing_type
        self.gpio = GPIOController()
        
        # Sistem durumu
        self.running = False
        self.current_mission = None
        self.mission_process = None
        
        # Mission konfigürasyonları
        self.missions = self._load_mission_configs()
        self.current_mission_index = 0
        
        # Buton durumu
        self.last_button_press = 0
        self.button_debounce_time = 0.5  # 500ms debounce
        self.long_press_threshold = 3.0  # 3 saniye uzun basma
        
        # Thread kontrolü
        self._stop_system = threading.Event()
        self._system_thread = None
        
        # Logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Callback fonksiyonları
        self.mission_start_callback: Optional[Callable] = None
        self.mission_end_callback: Optional[Callable] = None
        
    def _load_mission_configs(self) -> List[MissionConfig]:
        """Görev konfigürasyonlarını yükle"""
        base_path = os.path.dirname(__file__)
        project_root = os.path.dirname(base_path)
        
        missions = [
            # X Wing Görevleri
            MissionConfig(
                name="X Wing Otonom Görev 1",
                script_path=os.path.join(project_root, "x_wing", "Görevler", "autonomous_mission_1.py"),
                mission_type=MissionType.AUTONOMOUS,
                wing_type="x_wing",
                description="X Wing stabilizasyon ve derinlik kontrolü",
                duration_estimate=300
            ),
            MissionConfig(
                name="X Wing Otonom Görev 2", 
                script_path=os.path.join(project_root, "x_wing", "Görevler", "autonomous_mission_2.py"),
                mission_type=MissionType.AUTONOMOUS,
                wing_type="x_wing",
                description="X Wing mesafe sensörü ve navigasyon",
                duration_estimate=240
            ),
            MissionConfig(
                name="X Wing Manuel Görev 1",
                script_path=os.path.join(project_root, "x_wing", "Görevler", "manual_mission_1.py"),
                mission_type=MissionType.MANUAL,
                wing_type="x_wing", 
                description="X Wing manuel kontrol ve test",
                duration_estimate=180
            ),
            MissionConfig(
                name="X Wing Manuel Görev 2",
                script_path=os.path.join(project_root, "x_wing", "Görevler", "manual_mission_2.py"),
                mission_type=MissionType.MANUAL,
                wing_type="x_wing",
                description="X Wing servo kalibrasyonu",
                duration_estimate=120
            ),
            # + Wing Görevleri
            MissionConfig(
                name="+ Wing Otonom Görev 1",
                script_path=os.path.join(project_root, "+_wing", "Görevler", "autonomous_mission_1.py"),
                mission_type=MissionType.AUTONOMOUS,
                wing_type="+_wing",
                description="+ Wing stabilizasyon ve derinlik kontrolü",
                duration_estimate=300
            ),
            MissionConfig(
                name="+ Wing Otonom Görev 2",
                script_path=os.path.join(project_root, "+_wing", "Görevler", "autonomous_mission_2.py"),
                mission_type=MissionType.AUTONOMOUS,
                wing_type="+_wing",
                description="+ Wing mesafe sensörü ve navigasyon",
                duration_estimate=240
            ),
            MissionConfig(
                name="+ Wing Manuel Görev 1",
                script_path=os.path.join(project_root, "+_wing", "Görevler", "manual_mission_1.py"),
                mission_type=MissionType.MANUAL,
                wing_type="+_wing",
                description="+ Wing manuel kontrol ve test",
                duration_estimate=180
            ),
            MissionConfig(
                name="+ Wing Manuel Görev 2",
                script_path=os.path.join(project_root, "+_wing", "Görevler", "manual_mission_2.py"),
                mission_type=MissionType.MANUAL,
                wing_type="+_wing",
                description="+ Wing servo kalibrasyonu",
                duration_estimate=120
            )
        ]
        
        # Sadece mevcut wing type'a ait görevleri filtrele
        return [mission for mission in missions if mission.wing_type == self.wing_type]
    
    def start_system(self):
        """Buton sistemini başlat"""
        if self.running:
            self.logger.warning("Sistem zaten çalışıyor")
            return
        
        self.logger.info(f"Buton sistemi başlatılıyor - Wing Type: {self.wing_type}")
        
        # GPIO ayarla
        self.gpio.setup_gpio()
        if not self.gpio.setup_complete:
            self.logger.error("GPIO ayarlanamadı!")
            return False
        
        # Buton callback ayarla
        self.gpio.set_button_callback(self._button_pressed_handler)
        
        # Sistem thread'ini başlat
        self.running = True
        self._stop_system.clear()
        self._system_thread = threading.Thread(target=self._system_loop)
        self._system_thread.daemon = True
        self._system_thread.start()
        
        # Başlangıç sequence
        self.gpio.startup_sequence()
        
        self.logger.info("Buton sistemi başlatıldı")
        self._show_system_status()
        
        return True
    
    def stop_system(self):
        """Buton sistemini durdur"""
        self.logger.info("Buton sistemi durduruluyor...")
        
        self.running = False
        self._stop_system.set()
        
        # Çalışan görev varsa durdur
        if self.mission_process:
            self._stop_current_mission()
        
        # Thread'i durdur
        if self._system_thread:
            self._system_thread.join(timeout=2)
        
        # GPIO temizle
        self.gpio.cleanup_gpio()
        
        self.logger.info("Buton sistemi durduruldu")
    
    def _system_loop(self):
        """Ana sistem döngüsü"""
        while not self._stop_system.is_set() and self.running:
            try:
                # Sistem durumunu kontrol et
                self._check_system_health()
                
                # Görev durumunu kontrol et
                self._check_mission_status()
                
                # LED durumunu güncelle
                self._update_status_led()
                
                time.sleep(0.1)  # 10Hz döngü
                
            except Exception as e:
                self.logger.error(f"Sistem döngüsü hatası: {e}")
                time.sleep(1)
    
    def _button_pressed_handler(self):
        """Buton basma event handler"""
        current_time = time.time()
        
        # Debounce kontrolü
        if current_time - self.last_button_press < self.button_debounce_time:
            return
        
        self.last_button_press = current_time
        
        # Uzun basma kontrolü için thread başlat
        threading.Thread(target=self._handle_button_press, daemon=True).start()
    
    def _handle_button_press(self):
        """Buton basma işlemini yönet"""
        press_start = time.time()
        
        # Buton basılı mı diye kontrol et
        while self.gpio.is_button_pressed():
            time.sleep(0.1)
            if time.time() - press_start >= self.long_press_threshold:
                # Uzun basma
                self._handle_long_press()
                return
        
        # Kısa basma
        press_duration = time.time() - press_start
        if press_duration < self.long_press_threshold:
            self._handle_short_press()
    
    def _handle_short_press(self):
        """Kısa basma işlemi - Mission başlat/durdur"""
        self.logger.info("Kısa buton basışı algılandı")
        
        if self.current_mission:
            # Çalışan görev var - durdur
            self.logger.info("Çalışan görev durduruluyor...")
            self.gpio.buzzer_beep(0.3)
            self._stop_current_mission()
        else:
            # Yeni görev başlat
            self._start_next_mission()
    
    def _handle_long_press(self):
        """Uzun basma işlemi - Mission seçimini değiştir"""
        self.logger.info("Uzun buton basışı algılandı")
        
        # Ses geri bildirimi
        self.gpio.buzzer_beep_pattern([(0.1, 0.1), (0.1, 0.1), (0.3, 0)])
        
        # Sonraki görevi seç
        self.current_mission_index = (self.current_mission_index + 1) % len(self.missions)
        
        selected_mission = self.missions[self.current_mission_index]
        self.logger.info(f"Görev seçimi: {selected_mission.name}")
        
        # LED ile görev numarasını göster
        self._indicate_mission_selection()
        
        self._show_system_status()
    
    def _start_next_mission(self):
        """Seçilen görevi başlat"""
        if not self.missions:
            self.logger.error("Hiç görev tanımlanmamış!")
            self.gpio.error_sequence()
            return
        
        selected_mission = self.missions[self.current_mission_index]
        self.logger.info(f"Görev başlatılıyor: {selected_mission.name}")
        
        # Görev dosyası var mı kontrol et
        if not os.path.exists(selected_mission.script_path):
            self.logger.error(f"Görev dosyası bulunamadı: {selected_mission.script_path}")
            self.gpio.error_sequence()
            return
        
        # Ses geri bildirimi
        self.gpio.buzzer_beep(0.5)
        
        try:
            # Görev sürecini başlat
            if selected_mission.mission_type == MissionType.AUTONOMOUS:
                # Otonom görev - output suppress edilir
                self.mission_process = subprocess.Popen(
                    [sys.executable, selected_mission.script_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            else:
                # Manuel görev - normal çalıştırma
                self.mission_process = subprocess.Popen(
                    [sys.executable, selected_mission.script_path]
                )
            
            self.current_mission = selected_mission
            
            # Callback çağır
            if self.mission_start_callback:
                self.mission_start_callback(selected_mission)
            
            self.logger.info(f"Görev başlatıldı - PID: {self.mission_process.pid}")
            
        except Exception as e:
            self.logger.error(f"Görev başlatma hatası: {e}")
            self.gpio.error_sequence()
    
    def _stop_current_mission(self):
        """Çalışan görevi durdur"""
        if not self.current_mission or not self.mission_process:
            return
        
        self.logger.info(f"Görev durduruluyor: {self.current_mission.name}")
        
        try:
            # Süreci sonlandır
            self.mission_process.terminate()
            
            # 5 saniye bekle
            try:
                self.mission_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Zorla öldür
                self.mission_process.kill()
                self.logger.warning("Görev zorla sonlandırıldı")
            
            # Callback çağır
            if self.mission_end_callback:
                self.mission_end_callback(self.current_mission)
            
            self.current_mission = None
            self.mission_process = None
            
            self.logger.info("Görev durduruldu")
            
        except Exception as e:
            self.logger.error(f"Görev durdurma hatası: {e}")
    
    def _check_mission_status(self):
        """Görev durumunu kontrol et"""
        if not self.current_mission or not self.mission_process:
            return
        
        # Süreç hala çalışıyor mu?
        if self.mission_process.poll() is not None:
            # Görev tamamlandı
            return_code = self.mission_process.returncode
            
            if return_code == 0:
                self.logger.info(f"Görev başarıyla tamamlandı: {self.current_mission.name}")
                self.gpio.success_sequence()
            else:
                self.logger.warning(f"Görev hatayla sonlandı: {self.current_mission.name} (kod: {return_code})")
                self.gpio.error_sequence()
            
            # Callback çağır
            if self.mission_end_callback:
                self.mission_end_callback(self.current_mission)
            
            self.current_mission = None
            self.mission_process = None
    
    def _check_system_health(self):
        """Sistem sağlığını kontrol et"""
        # GPIO durumunu kontrol et
        if not self.gpio.setup_complete:
            self.logger.warning("GPIO bağlantısı kopuk!")
            return
        
        # Diğer sistem kontrolleri burada yapılabilir
        # Örneğin: disk alanı, bellek kullanımı, vs.
    
    def _update_status_led(self):
        """Durum LED'ini güncelle"""
        if self.current_mission:
            # Görev çalışıyor - LED yanıp sönsün
            if not hasattr(self, '_led_blinking') or not self._led_blinking:
                self.gpio.led_blink(0.5, 0.5, 0)  # Sonsuz blink
                self._led_blinking = True
        else:
            # Görev çalışmıyor - LED açık
            if hasattr(self, '_led_blinking') and self._led_blinking:
                self.gpio.stop_led_blink()
                self._led_blinking = False
            self.gpio.led_on()
    
    def _indicate_mission_selection(self):
        """LED ile görev seçimini belirt"""
        # Görev numarası kadar blink
        mission_number = self.current_mission_index + 1
        self.gpio.led_blink(0.2, 0.3, mission_number)
        time.sleep(mission_number * 0.5 + 1)
    
    def _show_system_status(self):
        """Sistem durumunu göster"""
        selected_mission = self.missions[self.current_mission_index] if self.missions else None
        
        status_info = f"""
{'='*60}
BUTON SİSTEMİ DURUMU - {self.wing_type.upper()}
{'='*60}
Toplam Görev: {len(self.missions)}
Seçili Görev: {self.current_mission_index + 1}
Görev Adı: {selected_mission.name if selected_mission else 'N/A'}
Görev Türü: {selected_mission.mission_type.value if selected_mission else 'N/A'}
Çalışan Görev: {self.current_mission.name if self.current_mission else 'Yok'}
{'='*60}
KONTROLLER:
- Kısa Basış: Görev başlat/durdur
- Uzun Basış (3s): Görev seçimini değiştir
{'='*60}
        """
        
        print(status_info)
        self.logger.info("Sistem durumu güncellendi")
    
    def set_callbacks(self, start_callback: Optional[Callable] = None, 
                     end_callback: Optional[Callable] = None):
        """Callback fonksiyonlarını ayarla"""
        self.mission_start_callback = start_callback
        self.mission_end_callback = end_callback
    
    def get_system_status(self) -> Dict:
        """Sistem durumunu dict olarak al"""
        selected_mission = self.missions[self.current_mission_index] if self.missions else None
        
        return {
            "running": self.running,
            "wing_type": self.wing_type,
            "total_missions": len(self.missions),
            "selected_mission_index": self.current_mission_index,
            "selected_mission": {
                "name": selected_mission.name if selected_mission else None,
                "type": selected_mission.mission_type.value if selected_mission else None,
                "description": selected_mission.description if selected_mission else None
            },
            "current_mission": {
                "name": self.current_mission.name if self.current_mission else None,
                "running": self.current_mission is not None
            },
            "gpio_status": self.gpio.get_status()
        }
    
    def list_missions(self) -> List[Dict]:
        """Mevcut görevleri listele"""
        return [
            {
                "index": i,
                "name": mission.name,
                "type": mission.mission_type.value,
                "description": mission.description,
                "duration_estimate": mission.duration_estimate,
                "script_exists": os.path.exists(mission.script_path)
            }
            for i, mission in enumerate(self.missions)
        ]

# Test ve yardımcı fonksiyonlar
def test_button_system(wing_type: str = "x_wing"):
    """Buton sistemini test et"""
    import signal
    
    def signal_handler(sig, frame):
        print("\nSistem durduruluyor...")
        button_system.stop_system()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Callback fonksiyonları
    def mission_started(mission):
        print(f"✅ Görev başlatıldı: {mission.name}")
    
    def mission_ended(mission):
        print(f"🏁 Görev tamamlandı: {mission.name}")
    
    # Buton sistemini başlat
    button_system = ButtonSystem(wing_type)
    button_system.set_callbacks(mission_started, mission_ended)
    
    if not button_system.start_system():
        print("❌ Buton sistemi başlatılamadı!")
        return
    
    print("Buton sistemi test modu - Ctrl+C ile çıkın")
    print("Butonu test edin...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == "__main__":
    # Test - wing type'ı parametre olarak al
    wing_type = sys.argv[1] if len(sys.argv) > 1 else "x_wing"
    test_button_system(wing_type)
