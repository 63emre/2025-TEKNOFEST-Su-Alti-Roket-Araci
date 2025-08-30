#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D300 Test - Çalışan Kod Versiyonu
Verdiğiniz çalışan kodun pluswing projesine entegre edilmiş hali
"""

import sys
import time
from pymavlink import mavutil
from config import *
from sensors import SensorManager
from utils import Logger

def test_d300_direct():
    """D300'ü direkt test et - çalışan kodun mantığı"""
    logger = Logger()
    
    logger.info("D300 direkt test başlıyor (kalibrasyon YOK)...")
    
    try:
        # MAVLink bağlantısı
        logger.info("MAVLink bağlantısı kuruluyor...")
        m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        
        logger.info("Heartbeat bekleniyor...")
        if not m.wait_heartbeat(timeout=10):
            logger.error("Heartbeat yok — bağlantıyı/portu kontrol edin.")
            return False
        
        logger.info("✅ Heartbeat alındı")
        
        # Veri akışı iste (çalışan kodun exact mantığı)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
            int(1e6/5), 0, 0, 0, 0, 0  # 5 Hz
        )
        
        logger.info("SCALED_PRESSURE2 veri akışı istendi")
        
        # SensorManager ile test
        sensor_manager = SensorManager(m, logger)
        
        logger.info("10 saniye boyunca D300 verilerini okuyacağız...")
        
        for i in range(50):  # 10 saniye * 5 Hz = 50 okuma
            try:
                # Direkt çalışan kod mantığı
                msg = m.recv_match(type="SCALED_PRESSURE2", blocking=True, timeout=2)
                
                if not msg:
                    logger.warning("Mesaj boş veya gelmiyor")
                    continue
                
                # Çalışan kodun exact veri çıkarma
                press_hpa = float(getattr(msg, "press_abs", 0.0))
                temp_c = float(getattr(msg, "temperature", 0)) / 100.0
                
                # Çalışan kodun exact derinlik hesaplama
                pa = press_hpa * 100.0
                dp = max(0.0, pa - D300_SEA_LEVEL_PRESSURE_PA)
                depth_m = dp / (D300_FRESHWATER_DENSITY * D300_GRAVITY)
                
                logger.info(f"D300 #{i+1:02d}: Derinlik={depth_m:.3f}m | Basınç={press_hpa:.1f}hPa | Sıcaklık={temp_c:.1f}°C")
                
                # Sensor manager ile de test
                depth_sm = sensor_manager.depth.get_depth_no_calibration()
                if depth_sm is not None:
                    logger.info(f"SensorManager: Derinlik={depth_sm:.3f}m")
                
                time.sleep(0.2)  # 5 Hz için
                
            except Exception as e:
                logger.error(f"Okuma hatası: {e}")
                continue
        
        logger.info("✅ D300 test tamamlandı!")
        return True
        
    except Exception as e:
        logger.error(f"D300 test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("=== D300 ÇALIŞAN KOD TESTİ ===")
    print("Bu test çalışan kodunuzun pluswing projesine entegre edilmiş halini test eder")
    print("Kalibrasyon YOK - Direkt derinlik okuma")
    print()
    
    success = test_d300_direct()
    
    if success:
        print("✅ Test başarılı!")
    else:
        print("❌ Test başarısız!")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
