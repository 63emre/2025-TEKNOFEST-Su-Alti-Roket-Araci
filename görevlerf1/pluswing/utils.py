#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UTILS - Yardımcı Fonksiyonlar
LED kontrolü, buzzer kontrolü, zamanlayıcılar ve loglama fonksiyonları
"""

import time
import threading
from gpio_wrapper import GPIO
from datetime import datetime
from config import *
from gpio_compat import GPIO

class LEDController:
    """Kırmızı LED kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_LED_RED, GPIO.OUT)
        self.led_state = False
        self.blink_thread = None
        self.blink_running = False
        
    def set_led(self, state):
        """LED'i aç/kapat"""
        self.led_state = state
        GPIO.output(GPIO_LED_RED, state)
        
    def turn_on(self):
        """LED'i aç"""
        self.set_led(True)
        
    def turn_off(self):
        """LED'i kapat"""
        self.set_led(False)
        
    def blink(self, interval=0.5, count=None):
        """LED'i yanıp söndür
        Args:
            interval: Yanıp sönme aralığı (saniye)
            count: Kaç kez yanıp söneceği (None=sürekli)
        """
        self.stop_blink()
        
        def blink_worker():
            blink_count = 0
            while self.blink_running:
                if count is not None and blink_count >= count * 2:
                    break
                    
                self.set_led(not self.led_state)
                time.sleep(interval)
                blink_count += 1
                
            self.blink_running = False
            
        self.blink_running = True
        self.blink_thread = threading.Thread(target=blink_worker)
        self.blink_thread.daemon = True
        self.blink_thread.start()
        
    def stop_blink(self):
        """Yanıp sönmeyi durdur"""
        self.blink_running = False
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)
            
    def cleanup(self):
        """Temizlik"""
        self.stop_blink()
        self.turn_off()

class BuzzerController:
    """Buzzer kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_BUZZER, GPIO.OUT)
        self.buzzer_thread = None
        self.buzzer_running = False
        
    def beep(self, duration):
        """Tek bip çıkar"""
        GPIO.output(GPIO_BUZZER, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(GPIO_BUZZER, GPIO.LOW)
        
    def beep_pattern(self, pattern):
        """Bip patterni çal
        Args:
            pattern: [süre1, süre2, süre3, ...] listesi
                    Çift indeksler HIGH, tek indeksler LOW süresi
        """
        self.stop_buzzer()
        
        def pattern_worker():
            for i, duration in enumerate(pattern):
                if not self.buzzer_running:
                    break
                    
                state = GPIO.HIGH if i % 2 == 0 else GPIO.LOW
                GPIO.output(GPIO_BUZZER, state)
                time.sleep(duration)
                
            GPIO.output(GPIO_BUZZER, GPIO.LOW)
            self.buzzer_running = False
            
        self.buzzer_running = True
        self.buzzer_thread = threading.Thread(target=pattern_worker)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
        
    def countdown_buzzer(self, total_seconds):
        """90 saniye geri sayım buzzer'ı
        90 saniye = 10 döngü x 9 saniye
        Her döngü: 9 kısa bip + 1 uzun bip
        """
        self.stop_buzzer()
        
        def countdown_worker():
            cycles = total_seconds // 9  # 90/9 = 10 döngü
            
            for cycle in range(cycles):
                if not self.buzzer_running:
                    break
                    
                # 9 kısa bip
                for short_beep in range(9):
                    if not self.buzzer_running:
                        break
                    
                    GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                    time.sleep(BUZZER_COUNTDOWN_SHORT)
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                    time.sleep(BUZZER_COUNTDOWN_PAUSE)
                    
                # 1 uzun bip (10. saniye)
                if self.buzzer_running:
                    GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                    time.sleep(BUZZER_COUNTDOWN_LONG)
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                    
                    # Son döngü değilse 8.5 saniye bekle
                    if cycle < cycles - 1:
                        time.sleep(8.5)
                        
            GPIO.output(GPIO_BUZZER, GPIO.LOW)
            self.buzzer_running = False
            
        self.buzzer_running = True
        self.buzzer_thread = threading.Thread(target=countdown_worker)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
        
    def mission_start_buzzer(self):
        """Görev başlangıcı - 5 saniyede bip"""
        self.beep_pattern(BUZZER_MISSION_START)
        
    def mission_end_buzzer(self):
        """Görev bitişi - 3 saniyede 1 bip"""
        self.beep_pattern(BUZZER_MISSION_END)
        
    def emergency_buzzer(self):
        """Acil durum buzzer'ı"""
        self.beep_pattern(BUZZER_EMERGENCY)
        
    def stop_buzzer(self):
        """Buzzer'ı durdur"""
        self.buzzer_running = False
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=1.0)
        GPIO.output(GPIO_BUZZER, GPIO.LOW)
        
    def cleanup(self):
        """Temizlik"""
        self.stop_buzzer()

class ButtonController:
    """Buton kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.last_press_time = 0
        self.debounce_time = 1.0  # 1000ms debounce (floating pin koruması)
        
    def is_pressed(self):
        """Buton EDGE tespiti - basma anını yakala"""
        current_time = time.time()
        
        # Çok sıkı debounce kontrolü
        if (current_time - self.last_press_time) < self.debounce_time:
            return False
            
        try:
            # Mevcut durum
            current_state = GPIO.input(GPIO_START_BUTTON)  # 1=basıldı, 0=serbest
            
            # Önceki durumu kontrol et
            if not hasattr(self, 'previous_state'):
                self.previous_state = 0  # İlk çalışmada serbest kabul et
            
            # EDGE tespiti: 0→1 geçişi (serbest→basıldı)
            if self.previous_state == 0 and current_state == 1:
                # Gerçek basma anı tespit edildi
                self.last_press_time = current_time
                self.previous_state = current_state
                return True
            
            # Durumu güncelle
            self.previous_state = current_state
            return False
                
        except Exception as e:
            # GPIO hatası varsa False döndür
            return False
        
    def wait_for_press(self, timeout=None):
        """Buton basılmasını bekle"""
        start_time = time.time()
        
        while True:
            if self.is_pressed():
                return True
                
            if timeout and (time.time() - start_time) > timeout:
                return False
                
            time.sleep(0.1)

class Timer:
    """Zamanlayıcı sınıfı"""
    
    def __init__(self):
        self.start_time = None
        self.pause_time = None
        self.paused_duration = 0
        
    def start(self):
        """Zamanlayıcıyı başlat"""
        self.start_time = time.time()
        self.pause_time = None
        self.paused_duration = 0
        
    def pause(self):
        """Zamanlayıcıyı duraklat"""
        if self.start_time and not self.pause_time:
            self.pause_time = time.time()
            
    def resume(self):
        """Zamanlayıcıyı devam ettir"""
        if self.pause_time:
            self.paused_duration += time.time() - self.pause_time
            self.pause_time = None
            
    def elapsed(self):
        """Geçen süreyi döndür (saniye)"""
        if not self.start_time:
            return 0
            
        current_time = time.time()
        if self.pause_time:
            # Duraklatılmış
            return (self.pause_time - self.start_time) - self.paused_duration
        else:
            # Çalışıyor
            return (current_time - self.start_time) - self.paused_duration
            
    def reset(self):
        """Zamanlayıcıyı sıfırla"""
        self.start_time = None
        self.pause_time = None
        self.paused_duration = 0
        
    def is_running(self):
        """Zamanlayıcı çalışıyor mu"""
        return self.start_time is not None and self.pause_time is None

class Logger:
    """Gelişmiş loglama sınıfı - HER ZAMAN dosyaya yaz + Raspberry Pi bağlantı durumuna göre loglama"""
    
    def __init__(self, log_file=None):
        self.log_file = log_file
        self.buffer = []  # Bağlantı yokken kritik logları buffer'a ekle
        self.max_buffer_size = 100
        self.connected = self._check_pi_connection()
        
        # KAPSAMLI LOG DOSYASI SİSTEMİ
        self.setup_comprehensive_logging()
    
    def setup_comprehensive_logging(self):
        """Kapsamlı loglama sistemi kurulumu"""
        import os
        from datetime import datetime
        
        # Log klasörü oluştur
        self.log_dir = "logs"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Tarih-saat bazlı log dosyası isimleri
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Ana log dosyası (TÜM loglar)
        self.main_log_file = os.path.join(self.log_dir, f"sara_mission_{timestamp}.log")
        
        # Kategori bazlı log dosyaları
        self.sensor_log_file = os.path.join(self.log_dir, f"sensors_{timestamp}.log")
        self.control_log_file = os.path.join(self.log_dir, f"control_{timestamp}.log")
        self.mission_log_file = os.path.join(self.log_dir, f"mission_{timestamp}.log")
        self.error_log_file = os.path.join(self.log_dir, f"errors_{timestamp}.log")
        
        # Son log dosyası linklerini oluştur (en güncel loglar için)
        self.create_latest_links()
        
        # Başlangıç logu
        self.write_startup_header()
    
    def create_latest_links(self):
        """En güncel log dosyalarına linkler oluştur"""
        import os
        
        latest_links = {
            "latest_main.log": self.main_log_file,
            "latest_sensors.log": self.sensor_log_file, 
            "latest_control.log": self.control_log_file,
            "latest_mission.log": self.mission_log_file,
            "latest_errors.log": self.error_log_file
        }
        
        for link_name, target_file in latest_links.items():
            link_path = os.path.join(self.log_dir, link_name)
            try:
                # Eski linki sil
                if os.path.exists(link_path):
                    os.remove(link_path)
                # Windows'ta copy, Linux'ta symlink
                if os.name == 'nt':
                    import shutil
                    shutil.copy2(target_file, link_path)
                else:
                    os.symlink(os.path.basename(target_file), link_path)
            except Exception as e:
                print(f"Latest link oluşturma hatası {link_name}: {e}")
    
    def write_startup_header(self):
        """Başlangıç header'ı yaz"""
        from datetime import datetime
        import platform
        import sys
        import os
        
        header = f"""
{'='*80}
SARA (Su Altı Roket Aracı) - Görev Log Dosyası
{'='*80}
Başlatılma Zamanı: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Platform: {platform.system()} {platform.release()}
Python: {sys.version.split()[0]}
Çalışma Dizini: {os.getcwd()}
Log Dizini: {self.log_dir}
{'='*80}
"""
        
        # Ana log dosyasına header yaz
        try:
            with open(self.main_log_file, "w", encoding="utf-8") as f:
                f.write(header + "\n")
        except Exception as e:
            print(f"Header yazma hatası: {e}")
        
    def _check_pi_connection(self):
        """Raspberry Pi bağlantı durumunu kontrol et"""
        try:
            # Basit network check - Pi'ye SSH bağlantısı var mı
            import socket
            import os
            # Pi üzerinde çalışıyorsak dosya sistemi kontrolü
            return os.path.exists('/proc/device-tree/model')
        except:
            return False
            
    def _add_to_buffer(self, message, level):
        """Kritik mesajları buffer'a ekle"""
        if level in ["ERROR", "WARNING", "CRITICAL"]:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.buffer.append((timestamp, level, message))
            
            # Buffer boyutu kontrolü
            if len(self.buffer) > self.max_buffer_size:
                self.buffer.pop(0)  # En eski kaydı sil
        
    def log(self, message, level="INFO", category=None):
        """Log mesajı yazdır - KAPSAMLI DOSYA YAZMA"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] [{level}] {message}"
        
        # Konsola yazdır (mevcut format korundu)
        print(log_message)
        
        # HER ZAMAN dosyaya yaz (Pi bağlantısı olmasın)
        self.write_to_log_files(log_message, level, category, timestamp)
        
        # Eski sistem (Pi bağlantısı varsa eski log dosyasına da yaz)
        if self.connected and self.log_file:
            try:
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(log_message + "\n")
                    
                # Buffer'daki bekleyen logları da yaz
                if self.buffer:
                    for buf_time, buf_level, buf_msg in self.buffer:
                        buf_log = f"[{buf_time}] [BUFFERED-{buf_level}] {buf_msg}"
                        f.write(buf_log + "\n")
                    self.buffer.clear()
                    
            except Exception as e:
                print(f"Eski log yazma hatası: {e}")
                self._add_to_buffer(message, level)
        else:
            # Bağlantı yoksa kritik mesajları buffer'a ekle
            self._add_to_buffer(message, level)
    
    def write_to_log_files(self, log_message, level, category, timestamp):
        """Tüm log dosyalarına yaz"""
        try:
            # 1. Ana log dosyasına HER ZAMAN yaz
            with open(self.main_log_file, "a", encoding="utf-8") as f:
                f.write(log_message + "\n")
            
            # 2. Kategori bazlı log dosyalarına yaz
            category_file = self.get_category_log_file(category, level)
            if category_file:
                with open(category_file, "a", encoding="utf-8") as f:
                    f.write(log_message + "\n")
            
            # 3. Error logları ayrı dosyaya
            if level in ["ERROR", "CRITICAL", "WARNING"]:
                with open(self.error_log_file, "a", encoding="utf-8") as f:
                    f.write(log_message + "\n")
            
            # 4. Latest linkleri güncelle (dosyalar büyüdükçe)
            self.update_latest_links()
            
        except Exception as e:
            print(f"Kapsamlı log yazma hatası: {e}")
            # En azından buffer'a ekle
            self._add_to_buffer(log_message, level)
    
    def get_category_log_file(self, category, level):
        """Kategori veya seviyeye göre log dosyası döndür"""
        if category:
            category_lower = category.lower()
            if "sensor" in category_lower or "d300" in category_lower or "attitude" in category_lower:
                return self.sensor_log_file
            elif "control" in category_lower or "servo" in category_lower or "motor" in category_lower or "pwm" in category_lower:
                return self.control_log_file
            elif "mission" in category_lower or "phase" in category_lower or "görev" in category_lower:
                return self.mission_log_file
        
        # Mesaj içeriğine göre kategori tahmin et
        message_lower = str(level).lower()
        if any(keyword in message_lower for keyword in ["sensor", "d300", "attitude", "derinlik", "basınç"]):
            return self.sensor_log_file
        elif any(keyword in message_lower for keyword in ["servo", "motor", "pwm", "control", "stabiliz"]):
            return self.control_log_file
        elif any(keyword in message_lower for keyword in ["mission", "phase", "görev", "faz"]):
            return self.mission_log_file
        
        return None
    
    def update_latest_links(self):
        """Latest linklerini güncelle (sadece gerektiğinde)"""
        # Bu fonksiyon çok sık çağrılmasın diye basit bir kontrol
        import time
        if not hasattr(self, '_last_link_update'):
            self._last_link_update = 0
        
        current_time = time.time()
        if current_time - self._last_link_update > 60:  # 60 saniyede bir güncelle
            try:
                self.create_latest_links()
                self._last_link_update = current_time
            except Exception as e:
                pass  # Sessizce geç, çok kritik değil
                
    def info(self, message):
        """Info seviyesi log"""
        self.log(message, "INFO")
        
    def warning(self, message):
        """Warning seviyesi log"""
        self.log(message, "WARNING")
        
    def error(self, message):
        """Error seviyesi log"""
        self.log(message, "ERROR")
        
    def debug(self, message):
        """Debug seviyesi log"""
        self.log(message, "DEBUG")
        
    def critical(self, message):
        """Critical seviyesi log"""
        self.log(message, "CRITICAL")
        
    def get_buffer_status(self):
        """Buffer durumunu döndür"""
        return {
            'connected': self.connected,
            'buffer_size': len(self.buffer),
            'max_buffer_size': self.max_buffer_size
        }
    
    # KATEGORİ BAZLI LOGLAMA FONKSİYONLARI
    def sensor_log(self, message, level="INFO"):
        """Sensör logları için"""
        self.log(message, level, category="SENSOR")
    
    def control_log(self, message, level="INFO"):
        """Kontrol sistemi logları için"""
        self.log(message, level, category="CONTROL")
    
    def mission_log(self, message, level="INFO"):
        """Görev logları için"""
        self.log(message, level, category="MISSION")
    
    def d300_log(self, message, level="INFO"):
        """D300 sensörü logları için"""
        self.log(message, level, category="D300_SENSOR")
    
    def get_log_summary(self):
        """Log dosyası özeti"""
        import os
        summary = {
            'log_directory': self.log_dir,
            'main_log': self.main_log_file,
            'files': {}
        }
        
        log_files = {
            'main': self.main_log_file,
            'sensors': self.sensor_log_file,
            'control': self.control_log_file,
            'mission': self.mission_log_file,
            'errors': self.error_log_file
        }
        
        for name, filepath in log_files.items():
            try:
                if os.path.exists(filepath):
                    size = os.path.getsize(filepath)
                    summary['files'][name] = {
                        'path': filepath,
                        'size_bytes': size,
                        'size_kb': round(size / 1024, 2),
                        'exists': True
                    }
                else:
                    summary['files'][name] = {
                        'path': filepath,
                        'exists': False
                    }
            except Exception as e:
                summary['files'][name] = {
                    'path': filepath,
                    'error': str(e)
                }
        
        return summary

class SystemStatus:
    """Sistem durumu takip sınıfı"""
    
    def __init__(self):
        self.led = LEDController()
        self.buzzer = BuzzerController()
        self.button = ButtonController()
        self.timer = Timer()
        self.logger = Logger("sara_mission.log")
        
        # Durum değişkenleri
        self.mission_phase = MissionPhase.WAITING
        self.is_emergency = False
        self.button_toggle_state = False  # Soft-kill butonu toggle durumu
        
    def set_phase(self, phase):
        """Görev fazını değiştir"""
        self.mission_phase = phase
        self.logger.info(f"Görev fazı değişti: {phase}")
        
        # Faza göre LED ve buzzer kontrolü
        if phase == MissionPhase.WAITING:
            self.led.blink(0.5)  # Yavaş yanıp sön
        elif phase == MissionPhase.CALIBRATION:
            self.led.turn_on()
        elif phase in [MissionPhase.PHASE_1, MissionPhase.PHASE_2, MissionPhase.RETURN]:
            self.led.turn_on()
        elif phase == MissionPhase.COMPLETED:
            self.led.blink(0.2, 10)  # Hızlı 10 kez yanıp sön
            self.buzzer.mission_end_buzzer()
        elif phase == MissionPhase.EMERGENCY:
            self.led.blink(0.1)  # Çok hızlı yanıp sön
            self.buzzer.emergency_buzzer()
            
    def check_start_button(self):
        """Başlatma butonunu kontrol et - İPTAL/RESTART sistemi"""
        if self.button.is_pressed():
            self.logger.info("🔘 Başlatma butonu GERÇEKten basıldı!")
            
            # Her basış restart için kullanılır
            self.logger.info("Görev başlatma/restart modu")
            return "restart"
                
        return None
        
    def emergency_stop(self):
        """Acil durum prosedürü"""
        self.is_emergency = True
        self.set_phase(MissionPhase.EMERGENCY)
        self.logger.error("ACİL DURUM AKTIF!")
        
    def cleanup(self):
        """Sistem temizliği"""
        self.logger.info("Sistem temizleniyor...")
        self.led.cleanup()
        self.buzzer.cleanup()
        GPIO.cleanup()
        self.logger.info("Sistem temizliği tamamlandı")

# Yardımcı fonksiyonlar
def wait_with_button_check(duration, button_controller, logger=None):
    """Belirtilen süre bekle ama buton kontrolü yap
    Args:
        duration: Bekleme süresi (saniye)
        button_controller: ButtonController instance
        logger: Logger instance (opsiyonel)
    Returns:
        True: Normal süre doldu
        False: Buton basıldı
    """
    start_time = time.time()
    
    while time.time() - start_time < duration:
        if button_controller.is_pressed():
            if logger:
                logger.info("Bekleme sırasında buton basıldı")
            return False
        time.sleep(0.1)
        
    return True

def establish_mavlink_connection(connection_string, retries=5, logger=None):
    """MAVLink bağlantısını güvenilir şekilde kur"""
    import sys
    sys.path.append('/usr/local/lib/python3.11/dist-packages')
    from pymavlink import mavutil
    
    for attempt in range(retries):
        try:
            if logger:
                logger.info(f"MAVLink bağlantı denemesi {attempt+1}/{retries}: {connection_string}")
                
            mavlink_connection = mavutil.mavlink_connection(connection_string, baud=57600)
            
            # Heartbeat bekle
            if logger:
                logger.info("Heartbeat bekleniyor...")
            heartbeat = mavlink_connection.wait_heartbeat(timeout=10)
            
            if heartbeat:
                if logger:
                    logger.info(f"MAVLink bağlantısı başarılı! Sistem ID: {mavlink_connection.target_system}")
                return True, mavlink_connection
            else:
                raise Exception("Heartbeat alınamadı")
                
        except Exception as e:
            if logger:
                logger.warning(f"Bağlantı denemesi {attempt+1}/{retries} başarısız: {e}")
            if attempt < retries - 1:
                time.sleep(2)
                
    if logger:
        logger.error("MAVLink bağlantısı kurulamadı!")
    return False, None

def check_sensor_connectivity(sensors, logger=None):
    """Sensör bağlantı durumunu kontrol et"""
    connectivity = {
        'depth_sensor': False,
        'attitude_sensor': False,
        'system_sensor': False,
        'overall_status': False
    }
    
    try:
        # D300 derinlik sensörü kontrolü
        if hasattr(sensors, 'depth') and sensors.depth:
            depth_data = sensors.depth.read_raw_data()
            connectivity['depth_sensor'] = depth_data is not None
            
        # Attitude sensörü kontrolü (Pixhawk içinde)
        if hasattr(sensors, 'attitude') and sensors.attitude:
            attitude_data = sensors.attitude.get_attitude(timeout=2.0)
            connectivity['attitude_sensor'] = attitude_data is not None
            
        # Sistem sensörü kontrolü
        if hasattr(sensors, 'system') and sensors.system:
            system_data = sensors.system.get_status()
            connectivity['system_sensor'] = system_data is not None
            
        # Genel durum: En azından derinlik sensörü çalışmalı
        connectivity['overall_status'] = connectivity['depth_sensor']
        
        if logger:
            logger.info(f"Sensör bağlantı durumu: {connectivity}")
            
    except Exception as e:
        if logger:
            logger.error(f"Sensör bağlantı kontrolü hatası: {e}")
            
    return connectivity

def estimate_distance(speed_pwm, elapsed_time):
    """Hız ve zamandan mesafe tahmini
    Args:
        speed_pwm: Motor PWM değeri
        elapsed_time: Geçen süre (saniye)
    Returns:
        Tahmini mesafe (metre)
    """
    if DISTANCE_CALC_MODE == "ADVANCED":
        return estimate_distance_advanced(speed_pwm, elapsed_time)
    else:
        # Eski basit yöntem (geriye uyumluluk için)
        return estimate_distance_simple(speed_pwm, elapsed_time)

def estimate_distance_simple(speed_pwm, elapsed_time):
    """Basit hız hesaplama (eski yöntem)
    Args:
        speed_pwm: Motor PWM değeri
        elapsed_time: Geçen süre (saniye)
    Returns:
        Tahmini mesafe (metre)
    """
    # PWM değerine göre hız tahmini
    if speed_pwm <= MOTOR_STOP:
        estimated_speed = 0
    elif speed_pwm <= SPEED_SLOW:
        estimated_speed = ESTIMATED_SPEED_SLOW
    elif speed_pwm <= SPEED_MEDIUM:
        estimated_speed = ESTIMATED_SPEED_MEDIUM
    else:
        estimated_speed = ESTIMATED_SPEED_FAST

    return estimated_speed * elapsed_time

class AdvancedDistanceCalculator:
    """Gelişmiş mesafe hesaplayıcı sınıfı"""

    def __init__(self):
        self.last_speed = 0.0
        self.last_pwm = MOTOR_STOP
        self.speed_history = []
        self.time_history = []
        self.max_history_size = 50  # Son 50 ölçümü tut

    def get_speed_from_pwm_curve(self, pwm_value):
        """PWM değerini hız karakteristik eğrisinden hesapla (doğrusal interpolasyon)
        Args:
            pwm_value: PWM değeri
        Returns:
            Hız (m/s)
        """
        if pwm_value <= MOTOR_STOP:
            return 0.0

        # PWM değerlerini ve karşılık gelen hızları sıralı listeye çevir
        pwm_points = sorted(MOTOR_SPEED_CURVE.keys())
        speed_points = [MOTOR_SPEED_CURVE[pwm] for pwm in pwm_points]

        # PWM değeri hangi aralıkta?
        for i in range(len(pwm_points) - 1):
            if pwm_points[i] <= pwm_value <= pwm_points[i + 1]:
                # Doğrusal interpolasyon
                pwm1, pwm2 = pwm_points[i], pwm_points[i + 1]
                speed1, speed2 = speed_points[i], speed_points[i + 1]

                # İnterpolasyon formülü: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                if pwm2 != pwm1:
                    speed = speed1 + (pwm_value - pwm1) * (speed2 - speed1) / (pwm2 - pwm1)
                else:
                    speed = speed1

                return speed

        # PWM değeri maksimumdan büyükse maksimum hızı döndür
        return max(speed_points)

    def apply_motor_dynamics(self, target_speed, elapsed_time):
        """Motor dinamiklerini uygula (hızlanma/yavaşlama)
        Args:
            target_speed: Hedef hız (m/s)
            elapsed_time: Geçen süre (saniye)
        Returns:
            Gerçekleşen hız (m/s)
        """
        # Motor başlangıç gecikmesini kontrol et
        if elapsed_time < MOTOR_STARTUP_DELAY:
            return 0.0

        # Hız farkı
        speed_diff = target_speed - self.last_speed

        if speed_diff > 0:
            # Hızlanma
            max_speed_change = elapsed_time / MOTOR_ACCELERATION_TIME * target_speed
            actual_speed_change = min(speed_diff, max_speed_change)
        else:
            # Yavaşlama
            max_speed_change = elapsed_time / MOTOR_DECELERATION_TIME * abs(target_speed)
            actual_speed_change = max(speed_diff, -max_speed_change)

        current_speed = self.last_speed + actual_speed_change

        # Hızı sınırla (negatif olamaz)
        current_speed = max(0.0, current_speed)

        return current_speed

    def apply_environmental_factors(self, speed, elapsed_time):
        """Çevresel faktörleri uygula (su direnci, sürtünme)
        Args:
            speed: Temiz hız (m/s)
            elapsed_time: Geçen süre (saniye)
        Returns:
            Düzeltilmiş hız (m/s)
        """
        if speed <= 0:
            return 0.0

        # Su direnci etkisi (kuvvet = 0.5 * ρ * v² * Cd * A)
        # Basitleştirilmiş: hız karesi ile orantılı direnç
        drag_force = WATER_DRAG_COEFFICIENT * speed * speed

        # Sürtünme etkisi
        friction_force = FRICTION_COEFFICIENT * speed

        # Toplam direnç
        total_resistance = drag_force + friction_force

        # Direnç faktörünü hesapla (basitleştirilmiş model)
        resistance_factor = max(0.1, 1.0 - total_resistance / 10.0)  # Min %10 verim

        return speed * resistance_factor

    def smooth_speed(self, new_speed):
        """Hız değerini yumuşat (noise azaltma)
        Args:
            new_speed: Yeni hız değeri
        Returns:
            Yumuşatılmış hız
        """
        if not self.speed_history:
            self.speed_history.append(new_speed)
            return new_speed

        # Üstel hareketli ortalama
        smoothed = (SPEED_SMOOTHING_FACTOR * new_speed +
                   (1 - SPEED_SMOOTHING_FACTOR) * self.speed_history[-1])

        self.speed_history.append(smoothed)

        # Geçmiş boyutu kontrolü
        if len(self.speed_history) > self.max_history_size:
            self.speed_history.pop(0)

        return smoothed

    def calculate_distance(self, pwm_value, elapsed_time):
        """Tam gelişmiş mesafe hesaplaması
        Args:
            pwm_value: Motor PWM değeri
            elapsed_time: Geçen süre (saniye)
        Returns:
            Tahmini mesafe (metre)
        """
        # 1. PWM'den hedef hızı al (karakteristik eğri)
        target_speed = self.get_speed_from_pwm_curve(pwm_value)

        # 2. Motor dinamiklerini uygula
        actual_speed = self.apply_motor_dynamics(target_speed, elapsed_time)

        # 3. Çevresel faktörleri uygula
        corrected_speed = self.apply_environmental_factors(actual_speed, elapsed_time)

        # 4. Hızı yumuşat
        smoothed_speed = self.smooth_speed(corrected_speed)

        # 5. Mesafeyi hesapla (ortalama hız * süre)
        distance = smoothed_speed * elapsed_time

        # Hızı güncelle (bir sonraki hesaplama için)
        self.last_speed = smoothed_speed
        self.last_pwm = pwm_value

        return distance

    def reset(self):
        """Hesaplayıcıyı sıfırla"""
        self.last_speed = 0.0
        self.last_pwm = MOTOR_STOP
        self.speed_history.clear()
        self.time_history.clear()

# Global gelişmiş hesaplayıcı örneği
_advanced_calculator = AdvancedDistanceCalculator()

def estimate_distance_advanced(speed_pwm, elapsed_time):
    """Gelişmiş mesafe hesaplaması
    Args:
        speed_pwm: Motor PWM değeri
        elapsed_time: Geçen süre (saniye)
    Returns:
        Tahmini mesafe (metre)
    """
    global _advanced_calculator

    # PWM değeri kontrolü
    if speed_pwm is None or elapsed_time is None or elapsed_time <= 0:
        return 0.0

    # PWM değerini sınırla
    pwm_clamped = clamp(speed_pwm, MOTOR_STOP, MOTOR_FORWARD_MAX)

    try:
        distance = _advanced_calculator.calculate_distance(pwm_clamped, elapsed_time)
        return max(0.0, distance)  # Negatif mesafe olamaz
    except Exception as e:
        # Hata durumunda basit yöntem fallback
        print(f"Gelişmiş mesafe hesaplaması hatası: {e}")
        return estimate_distance_simple(speed_pwm, elapsed_time)

def format_time(seconds):
    """Saniyeyi mm:ss formatına çevir"""
    minutes = int(seconds // 60)
    seconds = int(seconds % 60)
    return f"{minutes:02d}:{seconds:02d}"

def calculate_return_distance(phase1_distance, phase2_distance):
    """Geri dönüş mesafesini hesapla
    Args:
        phase1_distance: Faz 1'de gidilen mesafe (metre)
        phase2_distance: Faz 2'de gidilen mesafe (metre)
    Returns:
        Geri dönülecek toplam mesafe (metre)
    """
    return phase1_distance + phase2_distance

def safe_gpio_cleanup():
    """Güvenli GPIO temizleme"""
    try:
        GPIO.cleanup()
        print("✓ GPIO temizliği tamamlandı")
    except Exception as e:
        print(f"⚠️ GPIO temizleme hatası: {e}")

# Global sistem durumu (ana dosyadan kullanılabilir)
system_status = None

def init_system_status():
    """Sistem durumunu başlat"""
    global system_status
    if system_status is None:
        system_status = SystemStatus()
    return system_status

def get_system_status():
    """Sistem durumunu döndür"""
    global system_status
    if system_status is None:
        system_status = SystemStatus()
    return system_status
