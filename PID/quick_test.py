#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HIZLI PID TEST - Tek parametre optimizasyonu
Zaman kısıtlı durumlarda hızlı PID ayarlaması için
"""

import sys
import os
import time
import json
from datetime import datetime

# görevlerf1/pluswing modüllerini import et
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'görevlerf1', 'pluswing'))

from config import *
from utils import init_system_status, Logger
from sensors import SensorManager
from control import StabilizationController
from pymavlink import mavutil

class QuickPIDTest:
    """Hızlı PID Test - Tek parametre odaklı optimizasyon"""
    
    def __init__(self):
        self.logger = Logger()
        self.mavlink = None
        self.system_status = None
        
        # Test parametreleri
        self.test_depth = 0.5        # Sabit test derinliği
        self.test_duration = 10      # Kısa test süresi
        self.settle_time = 2         # Hızlı stabilizasyon
        
        # Mevcut PID değerleri (başlangıç)
        self.current_pid = {
            'kp': DEPTH_KP,
            'ki': DEPTH_KI,
            'kd': DEPTH_KD
        }
        
        self.logger.info("Quick PID Test başlatıldı")
        
    def setup_connections(self):
        """Hızlı bağlantı kurma"""
        self.logger.info("Bağlantılar kuruluyor...")
        
        # Sistem durumu
        self.system_status = init_system_status()
        
        # MAVLink - sadece ilk portu dene
        try:
            port = MAVLINK_PORTS[0] if sys.platform.startswith('linux') else MAVLINK_PORT_WIN
            self.mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
            
            msg = self.mavlink.wait_heartbeat(timeout=3)
            if msg:
                self.mavlink.target_system = msg.get_srcSystem()
                self.mavlink.target_component = msg.get_srcComponent()
                self.logger.info(f"✅ MAVLink bağlı: {port}")
                return True
            else:
                self.logger.error("❌ Heartbeat alınamadı")
                return False
                
        except Exception as e:
            self.logger.error(f"Bağlantı hatası: {e}")
            return False
            
    def quick_calibration(self):
        """Hızlı sensör kalibrasyonu"""
        try:
            sensor_manager = SensorManager(self.mavlink, self.logger)
            
            # Sadece D300'ü hızlı kalibre et
            self.logger.info("Hızlı D300 kalibrasyonu...")
            success = sensor_manager.depth.calibrate_surface(duration=3, use_water_surface=False)
            
            if success:
                self.logger.info("✅ Hızlı kalibrasyon tamam")
                return sensor_manager
            else:
                self.logger.error("❌ Kalibrasyon başarısız")
                return None
                
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return None
            
    def test_current_pid(self, sensor_manager):
        """Mevcut PID'yi test et"""
        try:
            self.logger.info("Mevcut PID test ediliyor...")
            self.logger.info(f"Kp={self.current_pid['kp']}, Ki={self.current_pid['ki']}, Kd={self.current_pid['kd']}")
            
            # Stabilizer oluştur
            stabilizer = StabilizationController(self.mavlink, sensor_manager, self.logger)
            
            # Test yap
            score = self._single_test(stabilizer, self.current_pid)
            
            self.logger.info(f"Mevcut PID skoru: {score:.3f}")
            return score
            
        except Exception as e:
            self.logger.error(f"Test hatası: {e}")
            return float('inf')
            
    def optimize_kp(self, sensor_manager, base_score):
        """Kp değerini optimize et"""
        self.logger.info("🎯 Kp optimizasyonu başlatılıyor...")
        
        # Kp test değerleri
        current_kp = self.current_pid['kp']
        test_values = [
            current_kp * 0.7,  # %30 azalt
            current_kp * 0.85, # %15 azalt
            current_kp,        # Mevcut
            current_kp * 1.15, # %15 artır
            current_kp * 1.3   # %30 artır
        ]
        
        best_kp = current_kp
        best_score = base_score
        
        for kp in test_values:
            test_pid = self.current_pid.copy()
            test_pid['kp'] = kp
            
            stabilizer = StabilizationController(self.mavlink, sensor_manager, self.logger)
            score = self._single_test(stabilizer, test_pid)
            
            self.logger.info(f"Kp={kp:.1f} → Skor: {score:.3f}")
            
            if score < best_score:
                best_score = score
                best_kp = kp
                self.logger.info(f"  ✅ Yeni en iyi Kp: {kp:.1f}")
                
            time.sleep(1)  # Kısa bekleme
            
        self.current_pid['kp'] = best_kp
        self.logger.info(f"Kp optimizasyonu tamamlandı: {best_kp:.1f} (skor: {best_score:.3f})")
        return best_score
        
    def optimize_ki(self, sensor_manager, base_score):
        """Ki değerini optimize et"""
        self.logger.info("🎯 Ki optimizasyonu başlatılıyor...")
        
        current_ki = self.current_pid['ki']
        test_values = [
            max(1.0, current_ki * 0.5),  # %50 azalt (min 1.0)
            current_ki * 0.75,           # %25 azalt
            current_ki,                  # Mevcut
            current_ki * 1.25,           # %25 artır
            current_ki * 1.5             # %50 artır
        ]
        
        best_ki = current_ki
        best_score = base_score
        
        for ki in test_values:
            test_pid = self.current_pid.copy()
            test_pid['ki'] = ki
            
            stabilizer = StabilizationController(self.mavlink, sensor_manager, self.logger)
            score = self._single_test(stabilizer, test_pid)
            
            self.logger.info(f"Ki={ki:.1f} → Skor: {score:.3f}")
            
            if score < best_score:
                best_score = score
                best_ki = ki
                self.logger.info(f"  ✅ Yeni en iyi Ki: {ki:.1f}")
                
            time.sleep(1)
            
        self.current_pid['ki'] = best_ki
        self.logger.info(f"Ki optimizasyonu tamamlandı: {best_ki:.1f} (skor: {best_score:.3f})")
        return best_score
        
    def _single_test(self, stabilizer, pid_params):
        """Tek bir PID testi yap - sabit frekanslı kontrol"""
        try:
            # PID parametrelerini ayarla
            stabilizer.depth_pid.kp = pid_params['kp']
            stabilizer.depth_pid.ki = pid_params['ki']
            stabilizer.depth_pid.kd = pid_params['kd']
            stabilizer.depth_pid.reset()
            
            # Test başlat
            stabilizer.set_target_depth(self.test_depth)
            stabilizer.enable_stabilization()
            
            # Güvenlik sınırları
            max_pitch_roll_deg = 20.0
            depth_soft_limit = self.test_depth + 0.2  # Hızlı test için dar sınır
            
            # Sabit frekanslı kontrol döngüsü
            rate_hz = 20.0
            dt = 1.0 / rate_hz
            next_t = time.monotonic()
            t0 = next_t
            
            # Stabilizasyon bekle
            time.sleep(self.settle_time)
            
            # Veri topla
            errors = []
            oscillations = []
            last_depth = None
            
            while (now := time.monotonic()) - t0 < self.test_duration:
                # Sabit cadence
                if now < next_t:
                    time.sleep(min(0.002, next_t - now))
                    continue
                next_t += dt
                
                # Stabilizasyonu güncelle
                try:
                    stabilizer.update_stabilization()
                except Exception as stab_e:
                    self.logger.warning(f"Stabilizasyon hatası: {stab_e}")
                    continue
                
                sensor_data = stabilizer.sensors.get_all_sensor_data()
                depth_data = sensor_data['depth']
                
                if depth_data['is_valid']:
                    current_depth = depth_data['depth_m']
                    
                    # Güvenlik kontrolü
                    if current_depth > depth_soft_limit:
                        self.logger.warning(f"  ⚠️ Derinlik sınırı aşıldı ({current_depth:.2f}m), test iptal")
                        stabilizer.disable_stabilization()
                        return float('inf')
                    
                    error = abs(self.test_depth - current_depth)
                    errors.append(error)
                    
                    # Oscillation
                    if last_depth is not None:
                        oscillations.append(abs(current_depth - last_depth))
                    last_depth = current_depth
                
                # Pitch/roll kontrolü
                attitude_data = sensor_data.get('attitude')
                if attitude_data:
                    try:
                        pitch = abs(math.degrees(attitude_data.get('pitch', 0)))
                        roll = abs(math.degrees(attitude_data.get('roll', 0)))
                        if pitch > max_pitch_roll_deg or roll > max_pitch_roll_deg:
                            self.logger.warning(f"  ⚠️ Aşırı pitch/roll, test iptal")
                            stabilizer.disable_stabilization()
                            return float('inf')
                    except (TypeError, AttributeError):
                        pass
                
            # Stabilizasyonu durdur
            stabilizer.disable_stabilization()
            
            # Skor hesapla
            if not errors:
                return float('inf')
                
            avg_error = sum(errors) / len(errors)
            max_error = max(errors)
            avg_osc = sum(oscillations) / len(oscillations) if oscillations else 0.0
            score = avg_error + (max_error * 0.2) + (avg_osc * 1.5)  # Hızlı test skoru
            return score
                
        except Exception as e:
            self.logger.error(f"Test hatası: {e}")
            try: 
                stabilizer.disable_stabilization()
            except: 
                pass
            return float('inf')
            
    def save_quick_results(self):
        """Hızlı sonuçları kaydet"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"PID/quick_pid_results_{timestamp}.json"
            
            results = {
                'timestamp': timestamp,
                'test_type': 'quick_optimization',
                'test_depth': self.test_depth,
                'original_pid': {
                    'kp': DEPTH_KP,
                    'ki': DEPTH_KI,
                    'kd': DEPTH_KD
                },
                'optimized_pid': self.current_pid,
                'improvement': {
                    'kp_change': self.current_pid['kp'] - DEPTH_KP,
                    'ki_change': self.current_pid['ki'] - DEPTH_KI,
                    'kd_change': self.current_pid['kd'] - DEPTH_KD
                }
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
                
            self.logger.info(f"Sonuçlar kaydedildi: {filename}")
            
            # Config güncellemesi
            config_update = f"""
# Hızlı PID Optimizasyon - {timestamp}
DEPTH_KP = {self.current_pid['kp']:.1f}  # (önceki: {DEPTH_KP})
DEPTH_KI = {self.current_pid['ki']:.1f}  # (önceki: {DEPTH_KI})
DEPTH_KD = {self.current_pid['kd']:.1f}  # (önceki: {DEPTH_KD})
"""
            
            with open("PID/quick_config.py", 'w', encoding='utf-8') as f:
                f.write(config_update)
                
        except Exception as e:
            self.logger.error(f"Kaydetme hatası: {e}")
            
    def run_quick_optimization(self):
        """Hızlı optimizasyon döngüsü"""
        try:
            self.logger.info("🚀 Hızlı PID Optimizasyonu Başlatılıyor")
            self.logger.info(f"Test Derinliği: {self.test_depth}m")
            self.logger.info(f"Test Süresi: {self.test_duration}s per test")
            
            # Bağlantılar
            if not self.setup_connections():
                return False
                
            # Hızlı kalibrasyon
            sensor_manager = self.quick_calibration()
            if not sensor_manager:
                return False
                
            self.logger.info("⏳ Test başlıyor...")
            time.sleep(3)
            
            # Mevcut PID'yi test et
            base_score = self.test_current_pid(sensor_manager)
            
            # Kp optimize et
            score_after_kp = self.optimize_kp(sensor_manager, base_score)
            
            # Ki optimize et (Kp sabitken)
            score_after_ki = self.optimize_ki(sensor_manager, score_after_kp)
            
            # Sonuçları kaydet
            self.save_quick_results()
            
            # Sonuç raporu
            improvement = ((base_score - score_after_ki) / base_score) * 100
            
            self.logger.info("🎉 Hızlı Optimizasyon Tamamlandı!")
            self.logger.info(f"Başlangıç Skoru: {base_score:.3f}")
            self.logger.info(f"Final Skoru: {score_after_ki:.3f}")
            self.logger.info(f"İyileştirme: %{improvement:.1f}")
            self.logger.info(f"Optimize PID: Kp={self.current_pid['kp']:.1f}, "
                           f"Ki={self.current_pid['ki']:.1f}, Kd={self.current_pid['kd']:.1f}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Optimizasyon hatası: {e}")
            return False
            
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Temizlik"""
        try:
            if self.mavlink:
                self.mavlink.close()
            if self.system_status:
                self.system_status.cleanup()
        except:
            pass

def main():
    """Ana fonksiyon"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Hızlı PID Test')
    parser.add_argument('--depth', type=float, default=0.5,
                       help='Test derinliği (metre)')
    parser.add_argument('--duration', type=int, default=10,
                       help='Test süresi (saniye)')
    
    args = parser.parse_args()
    
    # Quick test oluştur
    quick_test = QuickPIDTest()
    quick_test.test_depth = args.depth
    quick_test.test_duration = args.duration
    
    print("=" * 50)
    print("⚡ HIZLI PID OPTİMİZASYONU")
    print("=" * 50)
    print(f"Test Derinliği: {args.depth}m")
    print(f"Test Süresi: {args.duration}s per test")
    print("Toplam Süre: ~2-3 dakika")
    print("=" * 50)
    print()
    print("⚠️  Araç suda olmalı!")
    input("Devam etmek için Enter'a basın...")
    
    # Optimizasyonu çalıştır
    success = quick_test.run_quick_optimization()
    
    if success:
        print("\n⚡ Hızlı optimizasyon tamamlandı!")
        print("📁 Sonuçlar: quick_pid_results_*.json")
        print("📋 Config: quick_config.py")
    else:
        print("\n❌ Optimizasyon başarısız!")
        
    return success

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Test durduruldu")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Hata: {e}")
        sys.exit(1)
