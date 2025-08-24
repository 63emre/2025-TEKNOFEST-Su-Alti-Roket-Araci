#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COMPASS/MAGNETOMETER CALIBRATION - Pusula/Manyetometre Kalibrasyonu  
Yaw açısı ve 180° dönüş kontrolü için kalibrasyon

KULLANIM:
1. Açık alanda, manyetik girişimden uzak yerde yapın
2. Kart yatay pozisyonda 360° tam dönüş yapılacak
3. Yavaş ve düzgün dönüş yapın (yaklaşık 2 dakika)
4. Manyetik hard iron ve soft iron etkilerini kompanse eder
"""

import sys
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

# MAVLink bağlantı ayarları
MAVLINK_PORT = "/dev/ttyACM0"  # Linux
MAVLINK_PORT_WIN = "COM5"      # Windows
MAVLINK_BAUD = 115200

class CompassCalibrator:
    """Pusula/Manyetometre Kalibrasyon Sınıfı"""
    
    def __init__(self):
        self.mavlink = None
        self.calibration_data = {
            'raw_samples': [],
            'hard_iron_offset': {'x': 0, 'y': 0, 'z': 0},
            'soft_iron_matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            'declination': 0.0,  # Manyetik sapma
            'timestamp': None,
            'quality_score': 0.0
        }
        
    def connect_mavlink(self):
        """MAVLink bağlantısı kur"""
        print("MAVLink bağlantısı kuruluyor...")
        
        try:
            # Port seçimi
            port = MAVLINK_PORT
            if sys.platform.startswith('win'):
                port = MAVLINK_PORT_WIN
                
            self.mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
            
            # Heartbeat bekle
            print("Heartbeat bekleniyor...")
            if not self.mavlink.wait_heartbeat(timeout=15):
                print("❌ Heartbeat alınamadı!")
                return False
                
            print(f"✅ MAVLink bağlantısı başarılı - Sistem ID: {self.mavlink.target_system}")
            
            # Manyetometre veri akışını iste
            self.mavlink.mav.request_data_stream_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                10,  # 10 Hz
                1    # Başlat
            )
            
            return True
            
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            return False
            
    def collect_magnetometer_data(self, duration=120):
        """360° dönüş sırasında manyetometre verisi topla"""
        print(f"\n📊 {duration} saniye boyunca 360° dönüş verisi toplama")
        print("⚠️ YAVAŞ VE DÜZGüN DÖNÜŞ YAPIN!")
        print("Kart yatay pozisyonda tutarak tam bir daire çizin.")
        
        # 5 saniye hazırlık
        for i in range(5, 0, -1):
            print(f"Başlıyor... {i}")
            time.sleep(1)
            
        print("🔄 DÖNÜŞ BAŞLAYIN!")
        
        samples = []
        start_time = time.time()
        last_progress = 0
        
        while time.time() - start_time < duration:
            # RAW_IMU mesajından manyetometre verisi al
            msg = self.mavlink.recv_match(type='RAW_IMU', blocking=False, timeout=0.1)
            if msg:
                # mGauss cinsinden -> Gauss cinsine çevir
                mag_x = msg.xmag / 1000.0
                mag_y = msg.ymag / 1000.0
                mag_z = msg.zmag / 1000.0
                
                samples.append([mag_x, mag_y, mag_z])
                
            # İlerleme göster
            elapsed = time.time() - start_time
            progress = int((elapsed / duration) * 100)
            if progress > last_progress + 10:
                print(f"İlerleme: %{progress} - {len(samples)} örnek")
                last_progress = progress
                
            time.sleep(0.05)  # 20 Hz örnekleme
            
        print(f"✅ {len(samples)} manyetometre örneği toplandı")
        
        if len(samples) < 500:
            print(f"⚠️ Yetersiz veri! En az 500 örnek gerekli, {len(samples)} toplandı")
            return None
            
        return np.array(samples)
        
    def calculate_hard_iron_compensation(self, samples):
        """Hard Iron kompanzasyonu hesapla (manyetik offsetler)"""
        print("\n🧮 Hard Iron kompanzasyonu hesaplanıyor...")
        
        # En basit yöntem: Min/Max değerlerden merkez hesaplama
        min_x, max_x = np.min(samples[:, 0]), np.max(samples[:, 0])
        min_y, max_y = np.min(samples[:, 1]), np.max(samples[:, 1])
        min_z, max_z = np.min(samples[:, 2]), np.max(samples[:, 2])
        
        # Merkez nokta (offset)
        offset_x = (min_x + max_x) / 2.0
        offset_y = (min_y + max_y) / 2.0
        offset_z = (min_z + max_z) / 2.0
        
        # Yarıçap (scale kontrolü için)
        radius_x = (max_x - min_x) / 2.0
        radius_y = (max_y - min_y) / 2.0
        radius_z = (max_z - min_z) / 2.0
        
        print(f"Offset: X={offset_x:.3f}, Y={offset_y:.3f}, Z={offset_z:.3f} Gauss")
        print(f"Yarıçap: X={radius_x:.3f}, Y={radius_y:.3f}, Z={radius_z:.3f} Gauss")
        
        return offset_x, offset_y, offset_z
        
    def calculate_soft_iron_compensation(self, samples, offset):
        """Soft Iron kompanzasyonu hesapla (şekil bozulması düzeltme)"""
        print("\n🧮 Soft Iron kompanzasyonu hesaplanıyor...")
        
        # Offseti çıkar
        corrected = samples - np.array(offset)
        
        # Kovaryans matrisini hesapla
        cov_matrix = np.cov(corrected.T)
        
        # Eigenvalue decomposition
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # Scale faktörleri hesapla
        scales = np.sqrt(eigenvalues)
        avg_scale = np.mean(scales)
        
        # Normalleştirme matrisi
        scale_matrix = np.diag(avg_scale / scales)
        
        # Soft iron matrisi
        soft_iron = eigenvectors @ scale_matrix @ eigenvectors.T
        
        print(f"Soft Iron Matrisi:")
        for i, row in enumerate(soft_iron):
            print(f"  [{row[0]:6.3f}, {row[1]:6.3f}, {row[2]:6.3f}]")
            
        return soft_iron
        
    def evaluate_calibration_quality(self, samples, hard_iron, soft_iron):
        """Kalibrasyon kalitesini değerlendir"""
        print("\n📊 Kalibrasyon kalitesi değerlendiriliyor...")
        
        # Düzeltilmiş verileri hesapla
        corrected = samples - np.array(hard_iron)
        corrected = corrected @ soft_iron.T
        
        # Her noktanın orijine uzaklığını hesapla
        distances = np.linalg.norm(corrected, axis=1)
        
        # İstatistikler
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        min_distance = np.min(distances)
        max_distance = np.max(distances)
        
        # Kalite skoru (0-100)
        # Düşük standart sapma = yüksek kalite
        relative_std = std_distance / mean_distance
        quality_score = max(0, 100 * (1 - relative_std * 2))
        
        print(f"Ortalama manyetik alan: {mean_distance:.3f} Gauss")
        print(f"Standart sapma: {std_distance:.3f} Gauss")
        print(f"Min/Max: {min_distance:.3f} / {max_distance:.3f} Gauss")
        print(f"Kalite Skoru: {quality_score:.1f}/100")
        
        return quality_score
        
    def save_calibration(self, filename="compass_calibration.json"):
        """Kalibrasyon verilerini dosyaya kaydet"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
            print(f"✅ Kalibrasyon dosyaya kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Kaydetme hatası: {e}")
            
    def plot_calibration_results(self, samples, hard_iron, soft_iron):
        """Kalibrasyon sonuçlarını görselleştir"""
        try:
            # Düzeltilmiş verileri hesapla
            corrected = samples - np.array(hard_iron)
            corrected = corrected @ soft_iron.T
            
            # 2D plot (X-Y düzlemi)
            plt.figure(figsize=(12, 5))
            
            plt.subplot(1, 2, 1)
            plt.scatter(samples[:, 0], samples[:, 1], alpha=0.6, label='Ham Veri')
            plt.scatter(hard_iron[0], hard_iron[1], color='red', s=100, marker='x', label='Hard Iron Offset')
            plt.xlabel('Manyetometre X (Gauss)')
            plt.ylabel('Manyetometre Y (Gauss)')
            plt.title('Ham Manyetometre Verisi')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            
            plt.subplot(1, 2, 2)
            plt.scatter(corrected[:, 0], corrected[:, 1], alpha=0.6, label='Düzeltilmiş Veri', color='green')
            plt.scatter(0, 0, color='red', s=100, marker='x', label='Merkez')
            plt.xlabel('Düzeltilmiş X (Gauss)')
            plt.ylabel('Düzeltilmiş Y (Gauss)')
            plt.title('Kalibre Edilmiş Manyetometre Verisi')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            
            plt.tight_layout()
            plt.savefig('compass_calibration_plot.png', dpi=150)
            print("📊 Kalibrasyon grafiği kaydedildi: compass_calibration_plot.png")
            
        except Exception as e:
            print(f"⚠️ Grafik oluşturma hatası: {e}")
            
    def run_calibration(self):
        """Ana kalibrasyon prosedürü"""
        print("=" * 60)
        print("PUSULA/MANYETOMETRe KALİBRASYONU")
        print("Yaw açısı ve 180° dönüş kontrolü için kalibrasyon")
        print("=" * 60)
        
        # MAVLink bağlantısını kur
        if not self.connect_mavlink():
            return False
            
        print("\n📍 KALİBRASYON TALİMATLARI:")
        print("1. Açık alanda, manyetik girişimden uzak yerde olduğunuzdan emin olun")
        print("2. Kart yatay pozisyonda tutacaksınız")
        print("3. 2 dakika boyunca yavaş ve düzgün 360° dönüş yapacaksınız")
        print("4. Birden fazla tam dönüş yapabilirsiniz")
        print("\nHazır olduğunuzda ENTER'a basın...")
        input()
        
        # Manyetometre verisi topla
        samples = self.collect_magnetometer_data(duration=120)
        if samples is None:
            print("❌ Veri toplama başarısız!")
            return False
            
        self.calibration_data['raw_samples'] = samples.tolist()
        
        # Hard Iron kompanzasyonu
        hard_iron = self.calculate_hard_iron_compensation(samples)
        self.calibration_data['hard_iron_offset'] = {
            'x': hard_iron[0], 'y': hard_iron[1], 'z': hard_iron[2]
        }
        
        # Soft Iron kompanzasyonu
        soft_iron = self.calculate_soft_iron_compensation(samples, hard_iron)
        self.calibration_data['soft_iron_matrix'] = soft_iron.tolist()
        
        # Kalite değerlendirmesi
        quality = self.evaluate_calibration_quality(samples, hard_iron, soft_iron)
        self.calibration_data['quality_score'] = quality
        self.calibration_data['timestamp'] = time.time()
        
        # Sonuçları kaydet
        self.save_calibration()
        
        # Grafik çiz
        self.plot_calibration_results(samples, hard_iron, soft_iron)
        
        print("\n🎉 PUSULA KALİBRASYONU TAMAMLANDI!")
        
        if quality > 80:
            print("✅ Mükemmel kalibrasyon kalitesi!")
        elif quality > 60:
            print("✅ İyi kalibrasyon kalitesi")
        elif quality > 40:
            print("⚠️ Orta kalibrasyon kalitesi - Tekrar deneyebilirsiniz")
        else:
            print("❌ Düşük kalibrasyon kalitesi - Tekrar yapın!")
            
        print("Bu kalibrasyon verileri Yaw açısı ve 180° dönüş kontrolü için kullanılacak.")
        
        return True

def main():
    """Ana fonksiyon"""
    calibrator = CompassCalibrator()
    success = calibrator.run_calibration()
    
    if success:
        print("\n✅ Kalibrasyon başarılı!")
    else:
        print("\n❌ Kalibrasyon başarısız!")
        
    return success

if __name__ == "__main__":
    main()
