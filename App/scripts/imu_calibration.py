#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - IMU Kalibrasyonu
Gerçek IMU Sensör Kalibrasyonu
"""

import sys
import os
import time
import json
import math
import numpy as np
from collections import deque

# Parent directory import
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from mavlink_handler import MAVLinkHandler

class IMUCalibration:
    def __init__(self):
        """IMU kalibrasyon sistemi"""
        self.mavlink = MAVLinkHandler()
        self.samples_required = 100
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        self.mag_bias = [0.0, 0.0, 0.0]
        
    def connect_system(self):
        """Sisteme bağlan"""
        print("🔌 MAVLink bağlantısı kuruluyor...")
        
        if not self.mavlink.connect():
            print("❌ BAĞLANTI HATASI: Pixhawk'a bağlanılamadı!")
            print("   • Pixhawk açık mı?")
            print("   • IMU sensörleri çalışıyor mu?")
            return False
        
        print("✅ MAVLink bağlantısı başarılı")
        
        # IMU veri kontrolü
        print("🔍 IMU sensörleri kontrol ediliyor...")
        imu_data = self.mavlink.get_imu_data()
        if not imu_data:
            print("❌ IMU VERİSİ ALINAMIYOR!")
            print("   • RAW_IMU mesajı geliyor mu?")
            print("   • Pixhawk IMU'su çalışıyor mu?")
            return False
        
        print("✅ IMU verileri alınıyor")
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
        print(f"📊 Örnek IMU verisi:")
        print(f"   Accel: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s²")
        print(f"   Gyro:  X={gyro_x:.3f}, Y={gyro_y:.3f}, Z={gyro_z:.3f} rad/s")
        
        return True
    
    def collect_imu_samples(self, duration_seconds=10):
        """IMU örnekleri topla"""
        print(f"📡 {duration_seconds} saniye boyunca IMU verileri toplanacak...")
        print("⚠️ UYARI: Araç sabit tutulmalı (titreşim olmamalı)!")
        
        countdown = 5
        for i in range(countdown, 0, -1):
            print(f"🕐 Başlatma: {i}...")
            time.sleep(1)
        
        print("🚀 Veri toplama başladı!")
        
        accel_samples = {'x': [], 'y': [], 'z': []}
        gyro_samples = {'x': [], 'y': [], 'z': []}
        
        start_time = time.time()
        sample_count = 0
        
        while time.time() - start_time < duration_seconds:
            imu_data = self.mavlink.get_imu_data()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                
                accel_samples['x'].append(accel_x)
                accel_samples['y'].append(accel_y)
                accel_samples['z'].append(accel_z)
                
                gyro_samples['x'].append(gyro_x)
                gyro_samples['y'].append(gyro_y)
                gyro_samples['z'].append(gyro_z)
                
                sample_count += 1
                
                # Progress göster
                progress = (time.time() - start_time) / duration_seconds * 100
                print(f"\r📊 Progress: {progress:5.1f}% | Samples: {sample_count}", end='')
            
            time.sleep(0.05)  # 20Hz sampling
        
        print(f"\n✅ Toplam {sample_count} örnek toplandı")
        
        if sample_count < 50:
            print("❌ Yetersiz örnek! En az 50 örnek gerekli")
            return None, None
        
        return accel_samples, gyro_samples
    
    def calculate_gyro_bias(self, gyro_samples):
        """Gyroscope bias hesapla"""
        print("\n🔄 Gyroscope bias hesaplanıyor...")
        
        if not gyro_samples['x']:
            print("❌ Gyro örnekleri boş!")
            return False
        
        # Ortalama hesapla (bias)
        bias_x = np.mean(gyro_samples['x'])
        bias_y = np.mean(gyro_samples['y'])
        bias_z = np.mean(gyro_samples['z'])
        
        # Standart sapma (noise level)
        noise_x = np.std(gyro_samples['x'])
        noise_y = np.std(gyro_samples['y'])
        noise_z = np.std(gyro_samples['z'])
        
        self.gyro_bias = [bias_x, bias_y, bias_z]
        
        print("📊 Gyroscope Kalibrasyon Sonucu:")
        print(f"   Bias X: {bias_x:+8.5f} rad/s ({math.degrees(bias_x):+6.2f}°/s)")
        print(f"   Bias Y: {bias_y:+8.5f} rad/s ({math.degrees(bias_y):+6.2f}°/s)")
        print(f"   Bias Z: {bias_z:+8.5f} rad/s ({math.degrees(bias_z):+6.2f}°/s)")
        print(f"   Noise X: {noise_x:.5f} rad/s")
        print(f"   Noise Y: {noise_y:.5f} rad/s")
        print(f"   Noise Z: {noise_z:.5f} rad/s")
        
        # Kalite değerlendirmesi
        max_bias = max(abs(bias_x), abs(bias_y), abs(bias_z))
        max_noise = max(noise_x, noise_y, noise_z)
        
        if max_bias > 0.1:  # 5.7°/s
            print("⚠️ UYARI: Yüksek gyro bias - IMU ısınması gerekebilir")
        elif max_bias > 0.05:  # 2.9°/s
            print("⚠️ Orta seviye gyro bias - kabul edilebilir")
        else:
            print("✅ Düşük gyro bias - mükemmel!")
        
        if max_noise > 0.01:
            print("⚠️ UYARI: Yüksek gyro noise - titreşim var mı?")
        else:
            print("✅ Düşük gyro noise - iyi!")
        
        return True
    
    def calculate_accel_bias(self, accel_samples):
        """Accelerometer bias ve gravity hesapla"""
        print("\n🔄 Accelerometer bias hesaplanıyor...")
        
        if not accel_samples['x']:
            print("❌ Accel örnekleri boş!")
            return False
        
        # Ortalama hesapla
        mean_x = np.mean(accel_samples['x'])
        mean_y = np.mean(accel_samples['y'])
        mean_z = np.mean(accel_samples['z'])
        
        # Standart sapma
        noise_x = np.std(accel_samples['x'])
        noise_y = np.std(accel_samples['y'])
        noise_z = np.std(accel_samples['z'])
        
        # Gravity magnitude
        gravity_magnitude = math.sqrt(mean_x**2 + mean_y**2 + mean_z**2)
        
        # Z ekseni gravity olmalı (downward), X ve Y bias olmalı
        self.accel_bias = [mean_x, mean_y, mean_z - 9.81]
        
        print("📊 Accelerometer Kalibrasyon Sonucu:")
        print(f"   Mean X: {mean_x:+7.3f} m/s² (bias: {self.accel_bias[0]:+7.3f})")
        print(f"   Mean Y: {mean_y:+7.3f} m/s² (bias: {self.accel_bias[1]:+7.3f})")
        print(f"   Mean Z: {mean_z:+7.3f} m/s² (bias: {self.accel_bias[2]:+7.3f})")
        print(f"   Gravity: {gravity_magnitude:.3f} m/s² (ideal: 9.81)")
        print(f"   Noise X: {noise_x:.4f} m/s²")
        print(f"   Noise Y: {noise_y:.4f} m/s²")
        print(f"   Noise Z: {noise_z:.4f} m/s²")
        
        # Kalite değerlendirmesi
        gravity_error = abs(gravity_magnitude - 9.81)
        
        if gravity_error > 1.0:
            print("❌ UYARI: Gravity çok yanlış - accelerometer arızalı olabilir!")
        elif gravity_error > 0.5:
            print("⚠️ UYARI: Gravity hatası büyük - kalibrasyon gerekli")
        else:
            print("✅ Gravity değeri doğru!")
        
        # Araç pozisyonu kontrolü
        if abs(mean_z - 9.81) < 1.0 and abs(mean_x) < 2.0 and abs(mean_y) < 2.0:
            print("✅ Araç düz pozisyonda (Z aşağı bakıyor)")
        else:
            print("⚠️ UYARI: Araç eğik - kalibrasyon doğruluğu etkilenebilir")
        
        return True
    
    def test_imu_stability(self, duration=30):
        """IMU stabilitesi test et"""
        print(f"\n🔍 IMU STABİLİTE TESTİ ({duration} saniye)")
        print("-" * 40)
        print("⚠️ Araç sabit tutulmalı!")
        
        countdown = 3
        for i in range(countdown, 0, -1):
            print(f"🕐 Başlatma: {i}...")
            time.sleep(1)
        
        print("🚀 Stabilite testi başladı!")
        
        # Rolling buffers
        accel_buffer = deque(maxlen=50)  # 2.5 saniye @20Hz
        gyro_buffer = deque(maxlen=50)
        
        start_time = time.time()
        max_accel_variation = 0.0
        max_gyro_variation = 0.0
        
        while time.time() - start_time < duration:
            imu_data = self.mavlink.get_imu_data()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                
                # Magnitude hesapla
                accel_mag = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                gyro_mag = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                
                accel_buffer.append(accel_mag)
                gyro_buffer.append(gyro_mag)
                
                # Variation hesapla
                if len(accel_buffer) >= 10:
                    accel_var = np.std(list(accel_buffer))
                    gyro_var = np.std(list(gyro_buffer))
                    
                    max_accel_variation = max(max_accel_variation, accel_var)
                    max_gyro_variation = max(max_gyro_variation, gyro_var)
                    
                    # Progress
                    progress = (time.time() - start_time) / duration * 100
                    print(f"\r📊 Progress: {progress:5.1f}% | Accel Var: {accel_var:.4f} | Gyro Var: {gyro_var:.6f}", end='')
            
            time.sleep(0.05)
        
        print(f"\n📊 Stabilite Test Sonucu:")
        print(f"   Max Accel Variation: {max_accel_variation:.4f} m/s²")
        print(f"   Max Gyro Variation:  {max_gyro_variation:.6f} rad/s")
        
        # Sonuç değerlendirmesi
        if max_accel_variation < 0.1 and max_gyro_variation < 0.01:
            print("✅ MÜKEMMEL: IMU çok stabil!")
        elif max_accel_variation < 0.3 and max_gyro_variation < 0.03:
            print("✅ İYİ: IMU yeterince stabil")
        elif max_accel_variation < 0.5 and max_gyro_variation < 0.05:
            print("⚠️ ORTA: Hafif titreşim var")
        else:
            print("❌ KÖTÜ: Yüksek titreşim/noise!")
        
        return max_accel_variation, max_gyro_variation
    
    def save_calibration_data(self):
        """Kalibrasyon verilerini kaydet"""
        calibration_data = {
            "timestamp": time.time(),
            "gyro_bias": {
                "x": self.gyro_bias[0],
                "y": self.gyro_bias[1], 
                "z": self.gyro_bias[2]
            },
            "accel_bias": {
                "x": self.accel_bias[0],
                "y": self.accel_bias[1],
                "z": self.accel_bias[2]
            },
            "calibration_notes": {
                "gyro_bias_deg_per_sec": [math.degrees(b) for b in self.gyro_bias],
                "calibration_quality": "auto_generated"
            }
        }
        
        try:
            filename = f"../config/imu_calibration_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            print(f"✅ Kalibrasyon verisi kaydedildi: {filename}")
            return True
            
        except Exception as e:
            print(f"❌ Kalibrasyon kaydetme hatası: {e}")
            return False
    
    def full_calibration(self):
        """Tam IMU kalibrasyonu"""
        print("🧭 IMU TAM KALİBRASYON SİSTEMİ")
        print("=" * 50)
        
        if not self.connect_system():
            return False
        
        try:
            # 1. Stabilite testi
            print("\n1️⃣ ADIM 1: Stabilite Testi")
            accel_var, gyro_var = self.test_imu_stability(duration=15)
            
            if accel_var > 1.0 or gyro_var > 0.1:
                print("❌ UYARI: Çok yüksek titreşim!")
                response = input("Devam etmek istiyor musunuz? (y/N): ")
                if response.lower() != 'y':
                    return False
            
            input("\n⏸️ Kalibrasyon için ENTER'a basın...")
            
            # 2. Veri toplama
            print("\n2️⃣ ADIM 2: Kalibrasyon Verisi Toplama")
            accel_samples, gyro_samples = self.collect_imu_samples(duration=15)
            
            if not accel_samples or not gyro_samples:
                print("❌ Veri toplama başarısız!")
                return False
            
            # 3. Gyro bias hesaplama
            print("\n3️⃣ ADIM 3: Gyroscope Kalibrasyonu")
            if not self.calculate_gyro_bias(gyro_samples):
                return False
            
            # 4. Accel bias hesaplama
            print("\n4️⃣ ADIM 4: Accelerometer Kalibrasyonu")
            if not self.calculate_accel_bias(accel_samples):
                return False
            
            # 5. Sonuç kaydetme
            print("\n5️⃣ ADIM 5: Kalibrasyon Kaydetme")
            self.save_calibration_data()
            
            print("\n🎉 IMU KALİBRASYONU BAŞARILI!")
            print("=" * 50)
            print("📋 Kalibrasyon Özeti:")
            print(f"   Gyro Bias: X={math.degrees(self.gyro_bias[0]):+.2f}°/s, Y={math.degrees(self.gyro_bias[1]):+.2f}°/s, Z={math.degrees(self.gyro_bias[2]):+.2f}°/s")
            print(f"   Accel Bias: X={self.accel_bias[0]:+.3f}m/s², Y={self.accel_bias[1]:+.3f}m/s², Z={self.accel_bias[2]:+.3f}m/s²")
            
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Kalibrasyon kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Kalibrasyon hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        print("\n🧹 IMU kalibrasyon temizliği...")
        self.mavlink.disconnect()
        print("✅ Temizlik tamamlandı")

def main():
    """Ana fonksiyon"""
    imu_cal = IMUCalibration()
    
    print("🧭 TEKNOFEST IMU KALİBRASYON SİSTEMİ")
    print("=" * 60)
    print("1. Tam kalibrasyon (Önerilen)")
    print("2. Sadece stabilite testi")
    print("3. Hızlı bias testi")
    print("4. Çıkış")
    
    try:
        choice = input("\nSeçiminiz (1-4): ").strip()
        
        if choice == '1':
            imu_cal.full_calibration()
        elif choice == '2':
            if imu_cal.connect_system():
                imu_cal.test_imu_stability(duration=30)
                imu_cal.cleanup()
        elif choice == '3':
            if imu_cal.connect_system():
                accel_samples, gyro_samples = imu_cal.collect_imu_samples(duration=5)
                if accel_samples and gyro_samples:
                    imu_cal.calculate_gyro_bias(gyro_samples)
                    imu_cal.calculate_accel_bias(accel_samples)
                imu_cal.cleanup()
        elif choice == '4':
            print("👋 Çıkış yapılıyor...")
        else:
            print("❌ Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
    except Exception as e:
        print(f"\n❌ Program hatası: {e}")
    finally:
        print("Program bitti.")

if __name__ == "__main__":
    main() 