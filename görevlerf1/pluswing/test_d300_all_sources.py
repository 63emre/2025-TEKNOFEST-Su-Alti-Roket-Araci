#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D300 TÜM KAYNAK TARAMASI
Ne kadar D300 yayını varsa hepsini yakalar!
"""

import sys
import time
from pymavlink import mavutil
from config import *
from sensors import SensorManager, DepthSensor
from utils import Logger

def scan_all_mavlink_pressure_sources():
    """MAVLink üzerindeki TÜM basınç kaynaklarını tara"""
    logger = Logger()
    
    logger.info("🔍 MAVLink TÜM BASINCA KAYNAKLARI TARAMASI BAŞLIYOR...")
    
    try:
        # MAVLink bağlantısı
        logger.info("MAVLink bağlantısı kuruluyor...")
        m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        
        logger.info("Heartbeat bekleniyor...")
        if not m.wait_heartbeat(timeout=10):
            logger.error("Heartbeat yok — bağlantıyı/portu kontrol edin.")
            return False
        
        logger.info("✅ Heartbeat alındı")
        
        # TÜM olası basınç kaynaklarını tanımla
        all_pressure_sources = {
            0: {'name': 'SCALED_PRESSURE', 'msg_id': 29},
            1: {'name': 'SCALED_PRESSURE1', 'msg_id': 136},
            2: {'name': 'SCALED_PRESSURE2', 'msg_id': 137},
            3: {'name': 'SCALED_PRESSURE3', 'msg_id': 142},
            4: {'name': 'SCALED_PRESSURE4', 'msg_id': 143},
            5: {'name': 'SCALED_PRESSURE5', 'msg_id': 144},
            6: {'name': 'SCALED_PRESSURE6', 'msg_id': 145},
            7: {'name': 'SCALED_PRESSURE7', 'msg_id': 146},
        }
        
        # TÜM kaynaklardan veri akışı iste
        logger.info("📡 TÜM kaynaklardan veri akışı isteniyor...")
        for src_id, info in all_pressure_sources.items():
            try:
                interval_us = int(1_000_000 / 5)  # 5 Hz
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    info['msg_id'], interval_us, 0, 0, 0, 0, 0
                )
                logger.info(f"📡 Kaynak {src_id} ({info['name']}) veri akışı istendi")
                time.sleep(0.1)  # Kısa bekle
            except Exception as e:
                logger.warning(f"Kaynak {src_id} veri akışı isteği hatası: {e}")
        
        # 3 saniye bekle veri akışının başlaması için
        logger.info("⏳ 3 saniye bekleniyor veri akışının başlaması için...")
        time.sleep(3.0)
        
        # Her kaynağı test et
        active_sources = []
        logger.info("🔍 Kaynak taraması başlıyor...")
        
        for src_id, info in all_pressure_sources.items():
            logger.info(f"\n--- Kaynak {src_id} ({info['name']}) Test Ediliyor ---")
            
            found_data = False
            for attempt in range(5):  # 5 deneme
                try:
                    msg = m.recv_match(type=info['name'], blocking=False, timeout=1.0)
                    if msg:
                        pressure_hpa = float(getattr(msg, "press_abs", 0.0))
                        temp_raw = float(getattr(msg, "temperature", 0))
                        temp_c = temp_raw / 100.0
                        
                        # Geçerli veri kontrolü (700-1200 mbar)
                        if 700.0 <= pressure_hpa <= 1200.0:
                            # Derinlik hesapla
                            pa = pressure_hpa * 100.0
                            dp = max(0.0, pa - D300_SEA_LEVEL_PRESSURE_PA)
                            depth_m = dp / (D300_FRESHWATER_DENSITY * D300_GRAVITY)
                            
                            logger.info(f"✅ Kaynak {src_id} BULUNDU!")
                            logger.info(f"   Basınç: {pressure_hpa:.2f} hPa")
                            logger.info(f"   Sıcaklık: {temp_c:.2f} °C")
                            logger.info(f"   Derinlik: {depth_m:.3f} m")
                            
                            active_sources.append({
                                'id': src_id,
                                'name': info['name'],
                                'pressure': pressure_hpa,
                                'temperature': temp_c,
                                'depth': depth_m
                            })
                            found_data = True
                            break
                        else:
                            logger.warning(f"   Deneme {attempt+1}: Geçersiz basınç {pressure_hpa:.1f} hPa")
                    else:
                        logger.info(f"   Deneme {attempt+1}: Mesaj yok")
                        
                    time.sleep(0.3)
                    
                except Exception as e:
                    logger.warning(f"   Deneme {attempt+1}: Hata - {e}")
            
            if not found_data:
                logger.info(f"❌ Kaynak {src_id} ({info['name']}): Veri bulunamadı")
        
        # Sonuçları özetle
        logger.info("\n" + "="*60)
        logger.info("🎯 D300 KAYNAK TARAMASI SONUÇLARI:")
        logger.info("="*60)
        
        if active_sources:
            logger.info(f"✅ TOPLAM {len(active_sources)} AKTİF KAYNAK BULUNDU:")
            for src in active_sources:
                logger.info(f"   • Kaynak {src['id']} ({src['name']}): "
                          f"Derinlik={src['depth']:.3f}m, "
                          f"Basınç={src['pressure']:.1f}hPa, "
                          f"Sıcaklık={src['temperature']:.1f}°C")
            
            # En iyi kaynağı öner
            recommended = None
            for pref_id in [2, 3, 1, 0, 4, 5, 6, 7]:  # Öncelik sırası
                for src in active_sources:
                    if src['id'] == pref_id:
                        recommended = src
                        break
                if recommended:
                    break
            
            if recommended:
                logger.info(f"\n🏆 ÖNERİLEN KAYNAK: {recommended['id']} ({recommended['name']})")
                logger.info("Bu kaynağı config.py'de D300_SOURCE olarak ayarlayın!")
        else:
            logger.error("❌ HİÇBİR AKTİF D300 KAYNAĞI BULUNAMADI!")
            logger.info("Kontrol edilecekler:")
            logger.info("  • D300 sensörü bağlı mı?")
            logger.info("  • I2C bağlantıları doğru mu?")
            logger.info("  • Pixhawk D300'ü algılıyor mu?")
        
        logger.info("="*60)
        return len(active_sources) > 0
        
    except Exception as e:
        logger.error(f"D300 tarama hatası: {e}")
        return False

def test_sensor_manager_scan():
    """SensorManager ile D300 tarama testi"""
    logger = Logger()
    
    logger.info("\n🧪 SENSORMANAGER İLE D300 TARAMA TESTİ")
    
    try:
        # MAVLink bağlantısı
        m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        
        if not m.wait_heartbeat(timeout=10):
            logger.error("Heartbeat yok")
            return False
        
        # SensorManager oluştur (otomatik tarama yapacak)
        sensor_manager = SensorManager(m, logger)
        
        # 5 saniye bekle taramanın tamamlanması için
        time.sleep(5.0)
        
        # Test oku
        for i in range(10):
            depth = sensor_manager.depth.get_depth_no_calibration()
            if depth is not None:
                logger.info(f"SensorManager #{i+1}: Derinlik = {depth:.3f}m")
            else:
                logger.warning(f"SensorManager #{i+1}: Derinlik okunamadı")
            time.sleep(0.5)
        
        return True
        
    except Exception as e:
        logger.error(f"SensorManager test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🔍 D300 TÜM KAYNAK TARAMASI")
    print("Bu test ne kadar D300 yayını varsa hepsini yakalar!")
    print("="*60)
    
    # 1. Kapsamlı tarama
    success1 = scan_all_mavlink_pressure_sources()
    
    print("\n" + "="*60)
    
    # 2. SensorManager testi
    success2 = test_sensor_manager_scan()
    
    if success1 or success2:
        print("\n✅ Test başarılı - En az bir D300 kaynağı bulundu!")
    else:
        print("\n❌ Test başarısız - Hiç D300 kaynağı bulunamadı!")
    
    return success1 or success2

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
