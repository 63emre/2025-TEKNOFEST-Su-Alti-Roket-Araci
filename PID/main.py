#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID OPTİMİZASYON SİSTEMİ
4m çap, 1m derinlik test alanında otomatik PID değeri bulma
Iteratif daraltma algoritması ile en uygun PID parametrelerini bulur
"""

import sys
import os
import time
import json
import math
import statistics
from datetime import datetime

# görevlerf1/pluswing modüllerini import et
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'görevlerf1', 'pluswing'))

from config import *
from utils import init_system_status, Logger
from sensors import SensorManager
from control import StabilizationController, PIDController
from pymavlink import mavutil

class PIDOptimizer:
    """PID Optimizasyon Sistemi - Grid Search + Iterative Refinement"""
    
    def __init__(self):
        self.logger = Logger()
        self.mavlink = None
        self.system_status = None
        
        # Test alanı parametreleri
        self.test_area_radius = 2.0  # 4m çap = 2m yarıçap
        self.max_depth = 1.0         # 1m maksimum derinlik
        self.test_depths = [0.3, 0.5, 0.7, 1.0]  # Test derinlikleri
        
        # PID arama aralıkları
        self.pid_ranges = {
            'kp': {'min': 50.0, 'max': 400.0, 'current_best': 200.0},
            'ki': {'min': 2.0, 'max': 20.0, 'current_best': 10.0},
            'kd': {'min': 10.0, 'max': 100.0, 'current_best': 50.0}
        }
        
        # Optimizasyon parametreleri
        self.max_iterations = 5      # Maksimum iterasyon sayısı
        self.tests_per_iteration = 9 # Her iterasyonda 3x3 grid
        self.test_duration = 15      # Her test 15 saniye
        self.settle_time = 3         # Stabilizasyon için bekleme
        
        # Sonuç kayıtları
        self.test_results = []
        self.best_pid = None
        self.best_score = float('inf')
        
        self.logger.info("PID Optimizer başlatıldı")
        
    def setup_connections(self):
        """MAVLink ve sistem bağlantılarını kur"""
        self.logger.info("Bağlantılar kuruluyor...")
        
        # Sistem durumu başlat
        self.system_status = init_system_status()
        
        # MAVLink bağlantısı
        try:
            ports_to_try = MAVLINK_PORTS if sys.platform.startswith('linux') else [MAVLINK_PORT_WIN]
            
            for port in ports_to_try:
                try:
                    self.logger.info(f"MAVLink bağlantısı deneniyor: {port}")
                    self.mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
                    
                    msg = self.mavlink.wait_heartbeat(timeout=3)
                    if msg:
                        self.mavlink.target_system = msg.get_srcSystem()
                        self.mavlink.target_component = msg.get_srcComponent()
                        self.logger.info(f"✅ MAVLink bağlantısı başarılı: {port}")
                        return True
                except Exception as e:
                    self.logger.warning(f"Port {port} başarısız: {e}")
                    continue
                    
            self.logger.error("❌ MAVLink bağlantısı kurulamadı")
            return False
            
        except Exception as e:
            self.logger.error(f"Bağlantı hatası: {e}")
            return False
            
    def calibrate_sensors(self):
        """Sensör kalibrasyonu"""
        try:
            self.logger.info("Sensör kalibrasyonu başlatılıyor...")
            
            sensor_manager = SensorManager(self.mavlink, self.logger)
            results = sensor_manager.calibrate_all()
            
            if results.get('depth', False):
                self.logger.info("✅ D300 sensör kalibrasyonu başarılı")
                return sensor_manager
            else:
                self.logger.error("❌ D300 sensör kalibrasyonu başarısız")
                return None
                
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return None
            
    def create_test_grid(self, iteration):
        """Her iterasyonda bir ekseni sabit tutup diğer ikisini 3x3 tarar"""
        axes = ['kp', 'ki', 'kd']
        fixed = axes[iteration % 3]              # 0: kd sabit, 1: ki sabit, 2: kp sabit
        varying = [a for a in axes if a != fixed]

        grid_points = []

        # test değerlerini hazırla (merkez etrafında daraltma)
        for param in axes:
            range_info = self.pid_ranges[param]
            center = range_info['current_best']
            reduction_factor = 0.6 ** iteration
            range_width = (range_info['max'] - range_info['min']) * reduction_factor
            low = max(range_info['min'], center - range_width/2)
            high = min(range_info['max'], center + range_width/2)
            range_info['test_values'] = [low, center, high]

        # yalnızca iki parametreyi tara, sabit olanı current_best bırak
        p1, p2 = varying
        for v1 in self.pid_ranges[p1]['test_values']:
            for v2 in self.pid_ranges[p2]['test_values']:
                pid = {
                    p1: v1,
                    p2: v2,
                    fixed: self.pid_ranges[fixed]['current_best']
                }
                grid_points.append(pid)

        self.logger.info(f"İterasyon {iteration+1}: {len(grid_points)} test (sabit={fixed})")
        self.logger.info(f"{p1} aralığı: {self.pid_ranges[p1]['test_values']}")
        self.logger.info(f"{p2} aralığı: {self.pid_ranges[p2]['test_values']}")
        self.logger.info(f"{fixed} sabit: {self.pid_ranges[fixed]['current_best']}")
        
        return grid_points
        
    def test_pid_configuration(self, pid_params, sensor_manager):
        """Tek bir PID konfigürasyonunu test et"""
        try:
            self.logger.info(f"Test ediliyor: Kp={pid_params['kp']:.1f}, "
                           f"Ki={pid_params['ki']:.1f}, Kd={pid_params['kd']:.1f}")
            
            # Stabilizasyon kontrolcüsü oluştur
            stabilizer = StabilizationController(self.mavlink, sensor_manager, self.logger)
            
            # PID parametrelerini ayarla
            stabilizer.depth_pid.kp = pid_params['kp']
            stabilizer.depth_pid.ki = pid_params['ki']
            stabilizer.depth_pid.kd = pid_params['kd']
            stabilizer.depth_pid.reset()
            
            test_scores = []
            
            # Her test derinliği için ölçüm yap
            for target_depth in self.test_depths:
                if target_depth > self.max_depth:
                    continue
                    
                score = self._test_single_depth(stabilizer, target_depth)
                if score is not None:
                    test_scores.append(score)
                    
                time.sleep(2)  # Test arası bekleme
                
            # Ortalama skoru hesapla
            if test_scores:
                avg_score = statistics.mean(test_scores)
                self.logger.info(f"Test tamamlandı. Ortalama skor: {avg_score:.3f}")
                return avg_score
            else:
                self.logger.warning("Test başarısız - skor hesaplanamadı")
                return float('inf')
                
        except Exception as e:
            self.logger.error(f"PID test hatası: {e}")
            return float('inf')
            
    def _test_single_depth(self, stabilizer, target_depth):
        """Tek bir derinlik için test yap - sabit frekanslı kontrol döngüsü"""
        try:
            self.logger.info(f"  → {target_depth}m derinlik testi başlatılıyor...")
            
            # Hedef derinliği ayarla ve stabilizasyonu başlat
            stabilizer.set_target_depth(target_depth)
            stabilizer.enable_stabilization()
            
            # Güvenlik sınırları
            max_pitch_roll_deg = 25.0
            depth_soft_limit = min(self.max_depth, target_depth + 0.3)
            
            # Sabit frekanslı kontrol döngüsü
            rate_hz = 20.0
            dt = 1.0 / rate_hz
            next_t = time.monotonic()
            t0 = next_t
            
            # Stabilizasyon için bekle
            time.sleep(self.settle_time)
            
            # Test verilerini topla
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
                    self.logger.warning(f"Stabilizasyon güncelleme hatası: {stab_e}")
                    continue
                
                # Sensör verilerini al
                sensor_data = stabilizer.sensors.get_all_sensor_data()
                depth_data = sensor_data['depth']
                
                if depth_data['is_valid']:
                    current_depth = depth_data['depth_m']
                    
                    # Derinlik güvenliği
                    if current_depth > depth_soft_limit:
                        self.logger.warning(f"  ⚠️ Derinlik sınırı aşıldı ({current_depth:.2f}m > {depth_soft_limit:.2f}m), test iptal")
                        stabilizer.disable_stabilization()
                        return float('inf')
                    
                    error = abs(target_depth - current_depth)
                    errors.append(error)
                    
                    # Oscillation hesapla
                    if last_depth is not None:
                        oscillation = abs(current_depth - last_depth)
                        oscillations.append(oscillation)
                        
                    last_depth = current_depth
                
                # Basit pitch/roll güvenliği (varsa attitude sensörü)
                attitude_data = sensor_data.get('attitude')
                if attitude_data:
                    try:
                        pitch = abs(math.degrees(attitude_data.get('pitch', 0)))
                        roll = abs(math.degrees(attitude_data.get('roll', 0)))
                        if pitch > max_pitch_roll_deg or roll > max_pitch_roll_deg:
                            self.logger.warning(f"  ⚠️ Aşırı pitch/roll ({pitch:.1f}°/{roll:.1f}°), test iptal")
                            stabilizer.disable_stabilization()
                            return float('inf')
                    except (TypeError, AttributeError):
                        pass  # Attitude verisi eksik/hatalı, devam et
                        
            # Stabilizasyonu durdur
            stabilizer.disable_stabilization()
            
            # Skor hesapla
            if not errors:
                return float('inf')
                
            avg_error = statistics.mean(errors)
            max_error = max(errors)
            avg_oscillation = statistics.mean(oscillations) if oscillations else 0.0
            
            # Composite skor: hata + oscillation + overshoot cezası
            score = avg_error + (max_error * 0.3) + (avg_oscillation * 2.0)
            
            self.logger.info(f"    AvgErr={avg_error:.3f}m  MaxErr={max_error:.3f}m  Osc={avg_oscillation:.3f}m  Skor={score:.3f}")
            return score
                
        except Exception as e:
            self.logger.error(f"Derinlik testi hatası: {e}")
            try: 
                stabilizer.disable_stabilization()
            except: 
                pass
            return float('inf')
            
    def update_best_parameters(self, test_results):
        """En iyi parametreleri güncelle"""
        if not test_results:
            return
            
        # En iyi skoru bul
        best_result = min(test_results, key=lambda x: x['score'])
        
        if best_result['score'] < self.best_score:
            self.best_score = best_result['score']
            self.best_pid = best_result['params'].copy()
            
            # Arama merkezlerini güncelle
            for param in ['kp', 'ki', 'kd']:
                self.pid_ranges[param]['current_best'] = best_result['params'][param]
                
            self.logger.info(f"🎯 Yeni en iyi PID bulundu!")
            self.logger.info(f"   Kp: {self.best_pid['kp']:.1f}")
            self.logger.info(f"   Ki: {self.best_pid['ki']:.1f}")
            self.logger.info(f"   Kd: {self.best_pid['kd']:.1f}")
            self.logger.info(f"   Skor: {self.best_score:.3f}")
            
    def save_results(self):
        """Sonuçları dosyaya kaydet"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"PID/pid_optimization_results_{timestamp}.json"
            
            results_data = {
                'timestamp': timestamp,
                'test_area': {
                    'radius': self.test_area_radius,
                    'max_depth': self.max_depth,
                    'test_depths': self.test_depths
                },
                'optimization_params': {
                    'max_iterations': self.max_iterations,
                    'tests_per_iteration': self.tests_per_iteration,
                    'test_duration': self.test_duration
                },
                'best_pid': self.best_pid,
                'best_score': self.best_score,
                'all_results': self.test_results
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(results_data, f, indent=2, ensure_ascii=False)
                
            self.logger.info(f"Sonuçlar kaydedildi: {filename}")
            
        except Exception as e:
            self.logger.error(f"Sonuç kaydetme hatası: {e}")
            
    def generate_config_update(self):
        """config.py güncellemesi için kod üret"""
        if not self.best_pid:
            return
            
        config_update = f"""
# PID Optimizasyon Sonuçları - {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
# Test Alanı: {self.test_area_radius*2}m çap, {self.max_depth}m derinlik
# En İyi Skor: {self.best_score:.3f}

# Derinlik Kontrolü (PID) - OPTİMİZE EDİLMİŞ
DEPTH_KP = {self.best_pid['kp']:.1f}        # P kontrolcü katsayısı (önceki: {DEPTH_KP})
DEPTH_KI = {self.best_pid['ki']:.1f}         # I kontrolcü katsayısı (önceki: {DEPTH_KI})
DEPTH_KD = {self.best_pid['kd']:.1f}        # D kontrolcü katsayısı (önceki: {DEPTH_KD})
"""
        
        config_file = "PID/optimized_config.py"
        with open(config_file, 'w', encoding='utf-8') as f:
            f.write(config_update)
            
        self.logger.info(f"Config güncellemesi oluşturuldu: {config_file}")
        self.logger.info("Bu değerleri görevlerf1/pluswing/config.py'ye kopyalayın!")
        
    def run_optimization(self):
        """Ana optimizasyon döngüsü"""
        try:
            self.logger.info("🚀 PID Optimizasyon Başlatılıyor")
            self.logger.info(f"Test Alanı: {self.test_area_radius*2}m çap, {self.max_depth}m derinlik")
            self.logger.info(f"Test Derinlikleri: {self.test_depths}")
            
            # Bağlantıları kur
            if not self.setup_connections():
                return False
                
            # Sensörleri kalibre et
            sensor_manager = self.calibrate_sensors()
            if not sensor_manager:
                return False
                
            self.logger.info("⏳ Optimizasyon başlıyor - ARACIN SUDA OLDUĞUNDAN EMİN OLUN!")
            time.sleep(5)  # Hazırlık zamanı
            
            # Iteratif optimizasyon
            for iteration in range(self.max_iterations):
                self.logger.info(f"🔄 İTERASYON {iteration + 1}/{self.max_iterations}")
                
                # Test grid'i oluştur
                test_points = self.create_test_grid(iteration)
                iteration_results = []
                
                # Her test noktasını dene
                for i, pid_params in enumerate(test_points):
                    self.logger.info(f"Test {i+1}/{len(test_points)}")
                    
                    score = self.test_pid_configuration(pid_params, sensor_manager)
                    
                    result = {
                        'iteration': iteration + 1,
                        'test_number': i + 1,
                        'params': pid_params.copy(),
                        'score': score,
                        'timestamp': time.time()
                    }
                    
                    iteration_results.append(result)
                    self.test_results.append(result)
                    
                    # Güvenlik molası
                    time.sleep(3)
                    
                # En iyi parametreleri güncelle
                self.update_best_parameters(iteration_results)
                
                # Eğer hâlâ hiç geçerli skor yoksa erken uyarı
                if iteration == 0 and self.best_pid is None:
                    self.logger.warning("İlk iterasyonda geçerli skor bulunamadı. Sensör/bağlantı kontrol edin.")
                
                self.logger.info(f"İterasyon {iteration+1} tamamlandı")
                time.sleep(5)  # İterasyon arası mola
                
            # Sonuçları kaydet
            self.save_results()
            self.generate_config_update()
            
            # Final kontrol - None koruması
            if not self.best_pid:
                self.logger.error("Geçerli PID bulunamadı. Logları inceleyin.")
                return False
            
            self.logger.info("🎉 PID Optimizasyon Tamamlandı!")
            self.logger.info(f"En İyi PID: Kp={self.best_pid['kp']:.1f}, "
                           f"Ki={self.best_pid['ki']:.1f}, Kd={self.best_pid['kd']:.1f}")
            self.logger.info(f"En İyi Skor: {self.best_score:.3f}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Optimizasyon hatası: {e}")
            return False
            
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Sistem temizliği"""
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
    
    parser = argparse.ArgumentParser(description='PID Optimizasyon Sistemi')
    parser.add_argument('--test-area-radius', type=float, default=2.0,
                       help='Test alanı yarıçapı (metre)')
    parser.add_argument('--max-depth', type=float, default=1.0,
                       help='Maksimum test derinliği (metre)')
    parser.add_argument('--iterations', type=int, default=5,
                       help='Maksimum iterasyon sayısı')
    parser.add_argument('--test-duration', type=int, default=15,
                       help='Her test süresi (saniye)')
    
    args = parser.parse_args()
    
    # PID optimizer oluştur
    optimizer = PIDOptimizer()
    optimizer.test_area_radius = args.test_area_radius
    optimizer.max_depth = args.max_depth
    optimizer.max_iterations = args.iterations
    optimizer.test_duration = args.test_duration
    
    # Test derinliklerini güncelle
    optimizer.test_depths = [d for d in [0.3, 0.5, 0.7, args.max_depth] if d <= args.max_depth]
    
    print("=" * 60)
    print("🤖 SARA PID OPTİMİZASYON SİSTEMİ")
    print("=" * 60)
    print(f"Test Alanı: {args.test_area_radius*2}m çap")
    print(f"Maksimum Derinlik: {args.max_depth}m")
    print(f"İterasyon Sayısı: {args.iterations}")
    print(f"Test Süresi: {args.test_duration}s")
    print("=" * 60)
    print()
    print("⚠️  UYARI: Araç suda olmalı ve güvenli bir test alanında!")
    print("⚠️  Test sırasında araca müdahale etmeyin!")
    print()
    input("Devam etmek için Enter'a basın...")
    
    # Optimizasyonu çalıştır
    success = optimizer.run_optimization()
    
    if success:
        print("\n🎉 Optimizasyon başarıyla tamamlandı!")
        print("📁 Sonuçlar PID/ klasöründe kaydedildi")
        print("📋 optimized_config.py dosyasındaki değerleri config.py'ye kopyalayın")
    else:
        print("\n❌ Optimizasyon başarısız!")
        
    return success

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Optimizasyon kullanıcı tarafından durduruldu")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Kritik hata: {e}")
        sys.exit(1)
