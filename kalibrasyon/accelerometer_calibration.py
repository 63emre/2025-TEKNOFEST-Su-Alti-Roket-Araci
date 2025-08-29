#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ACCELEROMETER CALIBRATION - İvmeölçer Kalibrasyonu
Roll ve Pitch açısı hesaplaması için 6 yönlü kalibrasyon

KULLANIM:
1. Kart düz yatay pozisyonda başlatın
2. Kart 6 farklı pozisyona çevirmeniz istenecek:
   - Üst (Z+): Kart yüzü yukarı
   - Alt (Z-): Kart yüzü aşağı  
   - Sağ (Y+): Kart sağa çevrilmiş
   - Sol (Y-): Kart sola çevrilmiş
   - İleri (X+): Kart önü yukarı
   - Geri (X-): Kart arkası yukarı
3. Her pozisyonda kart sabit tutulmalı
"""

import sys
import time
import json
import numpy as np
from pymavlink import mavutil

# MAVLink bağlantı ayarları
MAVLINK_PORT = "/dev/ttyACM0"  # Linux
MAVLINK_PORT_WIN = "COM4"     # Windows
MAVLINK_BAUD = 115200

class AccelerometerCalibrator:
    """İvmeölçer Kalibrasyon Sınıfı"""
    
    def __init__(self):
        self.mavlink = None
        self.calibration_data = {
            'samples': {},
            'offsets': {'x': 0, 'y': 0, 'z': 0},
            'scales': {'x': 1, 'y': 1, 'z': 1},
            'timestamp': None
        }
        
        # 6 yön tanımları
        self.positions = [
            {'name': 'ÜST (Z+)', 'code': 'z_pos', 'desc': 'Kart yüzü yukarı, düz yatır'},
            {'name': 'ALT (Z-)', 'code': 'z_neg', 'desc': 'Kart yüzü aşağı, ters çevir'},
            {'name': 'SAĞ (Y+)', 'code': 'y_pos', 'desc': 'Kart sağa 90° çevir'},
            {'name': 'SOL (Y-)', 'code': 'y_neg', 'desc': 'Kart sola 90° çevir'},
            {'name': 'İLERİ (X+)', 'code': 'x_pos', 'desc': 'Kart önü yukarı, 90° çevir'},
            {'name': 'GERİ (X-)', 'code': 'x_neg', 'desc': 'Kart arkası yukarı, 90° çevir'}
        ]
        
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
            
            # IMU veri akışını iste
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
            
    def collect_samples(self, position_code, duration=10):
        """Belirli pozisyon için ivmeölçer örnekleri topla"""
        print(f"📊 {duration} saniye boyunca veri toplama başlıyor...")
        print("⚠️ KART HAREKETSİZ TUTULACAK!")
        
        # 3 saniye hazırlık
        for i in range(3, 0, -1):
            print(f"Başlıyor... {i}")
            time.sleep(1)
            
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # RAW_IMU mesajından ivmeölçer verisi al
            msg = self.mavlink.recv_match(type='RAW_IMU', blocking=False, timeout=0.1)
            if msg:
                # mG cinsinden -> G cinsine çevir (1000 mG = 1 G)
                acc_x = msg.xacc / 1000.0
                acc_y = msg.yacc / 1000.0  
                acc_z = msg.zacc / 1000.0
                
                samples.append([acc_x, acc_y, acc_z])
                
            time.sleep(0.05)  # 20 Hz örnekleme
            
        if len(samples) < 50:
            print(f"⚠️ Yetersiz veri! Sadece {len(samples)} örnek toplandı")
            return None
            
        # Ortalama hesapla
        avg_sample = np.mean(samples, axis=0)
        print(f"✅ {len(samples)} örnek toplandı - Ortalama: [{avg_sample[0]:.3f}, {avg_sample[1]:.3f}, {avg_sample[2]:.3f}] G")
        
        return avg_sample.tolist()
        
    def calculate_calibration(self):
        """6 yön verisinden kalibrasyon parametrelerini hesapla"""
        print("\n🧮 Kalibrasyon parametreleri hesaplanıyor...")
        
        samples = self.calibration_data['samples']
        
        # Her eksen için pozitif ve negatif değerleri al
        x_pos = np.array(samples['x_pos'])
        x_neg = np.array(samples['x_neg'])
        y_pos = np.array(samples['y_pos'])
        y_neg = np.array(samples['y_neg'])
        z_pos = np.array(samples['z_pos'])
        z_neg = np.array(samples['z_neg'])
        
        # Offset hesaplama (bias)
        offset_x = (x_pos[0] + x_neg[0]) / 2.0
        offset_y = (y_pos[1] + y_neg[1]) / 2.0
        offset_z = (z_pos[2] + z_neg[2]) / 2.0
        
        # Scale hesaplama (1G normalleştirme)
        scale_x = 2.0 / (x_pos[0] - x_neg[0]) if (x_pos[0] - x_neg[0]) != 0 else 1.0
        scale_y = 2.0 / (y_pos[1] - y_neg[1]) if (y_pos[1] - y_neg[1]) != 0 else 1.0
        scale_z = 2.0 / (z_pos[2] - z_neg[2]) if (z_pos[2] - z_neg[2]) != 0 else 1.0
        
        self.calibration_data['offsets'] = {'x': offset_x, 'y': offset_y, 'z': offset_z}
        self.calibration_data['scales'] = {'x': scale_x, 'y': scale_y, 'z': scale_z}
        self.calibration_data['timestamp'] = time.time()
        
        print("✅ Kalibrasyon parametreleri:")
        print(f"   Offset - X: {offset_x:.4f}, Y: {offset_y:.4f}, Z: {offset_z:.4f}")
        print(f"   Scale  - X: {scale_x:.4f}, Y: {scale_y:.4f}, Z: {scale_z:.4f}")
        
    def save_calibration(self, filename="accelerometer_calibration.json"):
        """Kalibrasyon verilerini dosyaya kaydet"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
            print(f"✅ Kalibrasyon dosyaya kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Kaydetme hatası: {e}")
            
    def run_calibration(self):
        """Ana kalibrasyon prosedürü"""
        print("=" * 60)
        print("İVMEÖLÇER KALİBRASYONU")
        print("Roll ve Pitch açısı hesaplaması için 6 yönlü kalibrasyon")
        print("=" * 60)
        
        # MAVLink bağlantısını kur
        if not self.connect_mavlink():
            return False
            
        print("\n📍 KALIBRASYON PROSEDÜRÜ:")
        print("Kart 6 farklı pozisyona çevrilecek.")
        print("Her pozisyonda kart 10 saniye hareketsiz tutulacak.")
        print("Hazır olduğunuzda ENTER'a basın...\n")
        input()
        
        # Her pozisyon için veri topla
        for i, position in enumerate(self.positions):
            print(f"\n📍 POZİSYON {i+1}/6: {position['name']}")
            print(f"👁️ {position['desc']}")
            print("Pozisyonu ayarlayın ve ENTER'a basın...")
            input()
            
            samples = self.collect_samples(position['code'])
            if samples is None:
                print("❌ Veri toplama başarısız! Kalibrasyon iptal ediliyor.")
                return False
                
            self.calibration_data['samples'][position['code']] = samples
            
        # Kalibrasyon hesapla
        self.calculate_calibration()
        
        # Kaydet
        self.save_calibration()
        
        print("\n🎉 İVMEÖLÇER KALİBRASYONU TAMAMLANDI!")
        print("Bu kalibrasyon verileri Roll ve Pitch açısı hesaplama için kullanılacak.")
        
        return True

def main():
    """Ana fonksiyon"""
    calibrator = AccelerometerCalibrator()
    success = calibrator.run_calibration()
    
    if success:
        print("\n✅ Kalibrasyon başarılı!")
    else:
        print("\n❌ Kalibrasyon başarısız!")
        
    return success

if __name__ == "__main__":
    main()
