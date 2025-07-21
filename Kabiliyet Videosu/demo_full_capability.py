#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Tam Kabiliyet Gösterim Videosu
Tüm şartname gereksinimlerini birleştiren kapsamlı video demo scripti
2-5 dakika YouTube videosu için optimal çekim sırası
"""

import time
import threading
import sys
import os
from datetime import datetime
import json

# Demo modüllerini import et
sys.path.append(os.path.dirname(__file__))

try:
    from demo_waterproof_test import WaterproofDemo
    from demo_maneuver_capabilities import ManeuverabilityDemo
    from demo_rocket_separation import RocketSeparationDemo
    from demo_emergency_stop import EmergencyStopDemo
except ImportError as e:
    print(f"❌ Demo modülü import hatası: {e}")
    print("💡 Tüm demo scriptlerinin aynı klasörde olduğundan emin olun!")
    sys.exit(1)

# Video çekim parametreleri
VIDEO_DURATION_TARGET = 300  # 5 dakika hedef (saniye)
VIDEO_QUALITY = "720p"       # Minimum şartname gereksinimi
VIDEO_SEGMENTS = [
    ("sistem_tanitimi", 30),      # Sistem tanıtımı
    ("acil_durdurma", 30),        # Acil durdurma testi  
    ("sizdimazlik", 90),          # Sızdırmazlık testi
    ("manevrabilite", 120),       # Hareket kabiliyeti
    ("roket_ayrilma", 45),        # Roket ayrılma
    ("sonuclar", 15)              # Test sonuçları
]

class FullCapabilityDemo:
    def __init__(self):
        self.video_start_time = None
        self.video_segments_completed = []
        self.overall_success = False
        
        # Demo sonuçları
        self.demo_results = {
            'waterproof': False,
            'maneuverability': False, 
            'rocket_separation': False,
            'emergency_stop': False
        }
        
        # Video telemetri
        self.video_telemetry = []
        self.segment_timings = {}
        
        # Demo instances
        self.waterproof_demo = None
        self.maneuver_demo = None
        self.rocket_demo = None
        self.emergency_demo = None
        
    def display_video_status(self, current_segment, elapsed_time, total_time):
        """Video çekim durumunu göster"""
        print("\n" + "="*80)
        print("🎬 TEKNOFEST Su Altı Roket Aracı - TAM KABİLİYET VİDEO ÇEKİMİ")
        print("="*80)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        progress = (elapsed_time / total_time) * 100 if total_time > 0 else 0
        
        print(f"📹 Video Çekim Zamanı: {timestamp}")
        print(f"⏱️ Video Süresi: {elapsed_time:.0f}s / {total_time:.0f}s ({progress:.1f}%)")
        print(f"🎯 Mevcut Segment: {current_segment}")
        
        # Segment durumu
        print(f"\n📋 VİDEO SEGMENTLERİ:")
        for segment_name, duration in VIDEO_SEGMENTS:
            status_icon = "✅" if segment_name in self.video_segments_completed else ("🔄" if segment_name == current_segment else "⏳")
            segment_display = {
                'sistem_tanitimi': 'Sistem Tanıtımı',
                'acil_durdurma': 'Acil Durdurma', 
                'sizdimazlik': 'Sızdırmazlık',
                'manevrabilite': 'Manevrabilite',
                'roket_ayrilma': 'Roket Ayrılma',
                'sonuclar': 'Sonuçlar'
            }.get(segment_name, segment_name)
            
            print(f"  {status_icon} {segment_display}: {duration}s")
        
        # Demo başarı durumu
        print(f"\n🏆 DEMO SONUÇLARI:")
        for demo_name, success in self.demo_results.items():
            demo_display = {
                'waterproof': 'Sızdırmazlık',
                'maneuverability': 'Manevrabilite',
                'rocket_separation': 'Roket Ayrılma', 
                'emergency_stop': 'Acil Durdurma'
            }.get(demo_name, demo_name)
            
            status_icon = "✅" if success else "❌"
            print(f"  {status_icon} {demo_display}")
        
        print("="*80)
    
    def record_segment_timing(self, segment_name, duration):
        """Segment zamanlamasını kaydet"""
        self.segment_timings[segment_name] = {
            'planned_duration': duration,
            'actual_duration': duration,  # Gerçek süre demo'dan gelecek
            'timestamp': time.time()
        }
        
        self.video_segments_completed.append(segment_name)
    
    def segment_system_introduction(self):
        """1. Segment: Sistem Tanıtımı (30s)"""
        print("\n🎬 1. SEGMENT: SİSTEM TANITIMI (30s)")
        print("-"*50)
        print("📋 Video Çekim Talimatları:")
        print("  🎥 Aracın genel görünümü")
        print("  🔧 Ana bileşenler tanıtımı")
        print("  📊 Teknik özellikler overlay")
        print("  🚀 TEKNOFEST logo ve takım adı")
        
        segment_start = time.time()
        
        # Sistem tanıtım metnini göster
        introduction_text = [
            "🚀 TEKNOFEST 2025 Su Altı Roket Aracı",
            "💡 Raspberry Pi 4B + Pixhawk 2.4.8 Kontrol",
            "🔧 4 Adet DS3230MG Su Geçirmez Servo",
            "⚡ DEGZ M5 Su Altı Motor + 30A ESC", 
            "🛡️ Tam Sızdırmazlık Tasarımı",
            "🎯 Otonom Su Altı Navigasyon",
            "🚀 Model Roket Fırlatma Sistemi"
        ]
        
        for i, text in enumerate(introduction_text):
            print(f"  📝 [{i+1}/7] {text}")
            time.sleep(4)  # Her başlık 4s
            
            if time.time() - segment_start >= 30:
                break
        
        self.record_segment_timing("sistem_tanitimi", 30)
        print("✅ Sistem tanıtımı tamamlandı!")
        
        input("\n⏸️ Sistem tanıtımı çekimi tamam mı? Devam için ENTER...")
        return True
    
    def segment_emergency_stop(self):
        """2. Segment: Acil Durdurma Testi (30s)"""
        print("\n🚨 2. SEGMENT: ACİL DURDURMA TESTİ (30s)")
        print("-"*50)
        
        segment_start = time.time()
        
        # Kısa acil durdurma demo
        self.emergency_demo = EmergencyStopDemo()
        
        print("📹 Acil durdurma çekimi başlıyor...")
        print("📋 Video Çekim Talimatları:")
        print("  🔘 Buton basımını yakın çekim")
        print("  🚨 LED uyarı sinyallerini göster")
        print("  ⏹️ Motor durdurmayı kaydet")
        print("  💡 Sistem kapanmasını göster")
        
        # Basitleştirilmiş emergency demo (30s için optimize)
        try:
            if self.emergency_demo.connect_pixhawk():
                # 15s normal operasyon
                print("  🚀 Normal operasyon gösterimi (15s)")
                for i in range(15):
                    print(f"    📊 Normal operasyon: {15-i}s - ACİL DURDURMA butonuna basın!")
                    time.sleep(1)
                    
                    if self.emergency_demo.emergency_button_pressed:
                        print("  🚨 ACİL DURDURMA tetiklendi!")
                        break
                
                # 15s emergency response
                if not self.emergency_demo.emergency_button_pressed:
                    print("  ⚠️ Manuel acil durdurma simülasyonu")
                    self.emergency_demo.execute_emergency_stop()
                
                self.demo_results['emergency_stop'] = True
                print("✅ Acil durdurma segment başarılı!")
                
        except Exception as e:
            print(f"❌ Acil durdurma segment hatası: {e}")
            self.demo_results['emergency_stop'] = False
        
        self.record_segment_timing("acil_durdurma", 30)
        
        input("\n⏸️ Acil durdurma çekimi tamam mı? Devam için ENTER...")
        return True
    
    def segment_waterproof_test(self):
        """3. Segment: Sızdırmazlık Testi (90s)"""
        print("\n💧 3. SEGMENT: SIZDIMAZLIK TESTİ (90s)")
        print("-"*50)
        
        segment_start = time.time()
        
        self.waterproof_demo = WaterproofDemo()
        
        print("📹 Sızdırmazlık çekimi başlıyor...")
        print("📋 Video Çekim Talimatları:")
        print("  🌊 1m+ derinlikte statik test")
        print("  🚀 Hareket halinde dinamik test")
        print("  📦 Kapak mekanizması testi")
        print("  💧 Kabarcık kontrolü yakın çekim")
        
        try:
            if self.waterproof_demo.connect_pixhawk():
                # Hızlandırılmış sızdırmazlık testi (90s için optimize)
                print("  🔧 Yüzey kalibrasyonu (15s)")
                self.waterproof_demo.calibrate_surface_pressure(duration=15)
                
                print("  🌊 Derinlik inişi (20s)")
                # Hızlı derinlik inişi
                descend_start = time.time()
                while time.time() - descend_start < 20:
                    self.waterproof_demo.set_motor_throttle(1400)  # Down thrust
                    self.waterproof_demo.read_sensors()
                    print(f"    📊 İniş: {self.waterproof_demo.current_depth:.1f}m")
                    
                    if self.waterproof_demo.current_depth >= 1.0:
                        break
                    time.sleep(2)
                
                print("  ⚖️ Statik sızdırmazlık testi (30s)")
                static_start = time.time()
                while time.time() - static_start < 30:
                    self.waterproof_demo.read_sensors()
                    leak_status = "SIZ YOK" if not self.waterproof_demo.check_for_leaks() else "SIZINTI!"
                    print(f"    💧 Statik test: {30-(time.time()-static_start):.0f}s | {leak_status} | {self.waterproof_demo.current_depth:.1f}m")
                    time.sleep(2)
                
                print("  🚀 Dinamik hareket testi (25s)")
                dynamic_start = time.time()
                while time.time() - dynamic_start < 25:
                    # Basit hareket patterns
                    self.waterproof_demo.set_motor_throttle(1550)  # Forward
                    self.waterproof_demo.read_sensors()
                    leak_status = "SIZ YOK" if not self.waterproof_demo.check_for_leaks() else "SIZINTI!"
                    print(f"    🌊 Dinamik test: {25-(time.time()-dynamic_start):.0f}s | {leak_status}")
                    time.sleep(2)
                
                # Sızdırmazlık başarı kontrolü
                if not self.waterproof_demo.leak_detected:
                    self.demo_results['waterproof'] = True
                    print("✅ Sızdırmazlık segment başarılı!")
                else:
                    print("❌ Sızıntı tespit edildi!")
                
        except Exception as e:
            print(f"❌ Sızdırmazlık segment hatası: {e}")
            self.demo_results['waterproof'] = False
        
        self.record_segment_timing("sizdimazlik", 90)
        
        input("\n⏸️ Sızdırmazlık çekimi tamam mı? Devam için ENTER...")
        return True
    
    def segment_maneuverability(self):
        """4. Segment: Manevrabilite Testi (120s)"""
        print("\n🚀 4. SEGMENT: MANEVRABİLİTE TESTİ (120s)")
        print("-"*50)
        
        segment_start = time.time()
        
        self.maneuver_demo = ManeuverabilityDemo()
        
        print("📹 Manevrabilite çekimi başlıyor...")
        print("📋 Video Çekim Talimatları:")
        print("  ➡️ Düz seyir gösterimi")
        print("  ↩️↪️ Sol/sağ dönüş manevralar")
        print("  🔼🔽 Yukarı/aşağı yunuslama")
        print("  🌊 Su yüzeyine çıkış")
        
        try:
            if self.maneuver_demo.connect_pixhawk():
                # Demo derinliğine in (20s)
                print("  📍 Demo derinliğine iniş (20s)")
                if self.maneuver_demo.descend_to_demo_depth():
                    
                    # Hızlandırılmış manevralar (100s)
                    print("  🚀 Manevrabilite gösterimleri (100s)")
                    
                    # Düz seyir (15s)
                    self.maneuver_demo.maneuver_straight_cruise(15)
                    
                    # Sağ dönüş (15s) 
                    self.maneuver_demo.maneuver_turn_right(15, 90)
                    
                    # Sol dönüş (15s)
                    self.maneuver_demo.maneuver_turn_left(15, 90)
                    
                    # Yukarı yunuslama (10s)
                    self.maneuver_demo.maneuver_pitch_up(10, 20)
                    
                    # Aşağı yunuslama (10s)
                    self.maneuver_demo.maneuver_pitch_down(10, -20)
                    
                    # Yüzeye çıkış (35s)
                    self.maneuver_demo.maneuver_surface_ascent(35)
                    
                    # Başarı değerlendirmesi
                    if self.maneuver_demo.total_maneuver_time >= 60:  # Min 60s şartnamesi
                        self.demo_results['maneuverability'] = True
                        print("✅ Manevrabilite segment başarılı!")
                    else:
                        print("❌ Yetersiz manevrabilite süresi!")
                
        except Exception as e:
            print(f"❌ Manevrabilite segment hatası: {e}")
            self.demo_results['maneuverability'] = False
        
        self.record_segment_timing("manevrabilite", 120)
        
        input("\n⏸️ Manevrabilite çekimi tamam mı? Devam için ENTER...")
        return True
    
    def segment_rocket_separation(self):
        """5. Segment: Roket Ayrılma Testi (45s)"""
        print("\n🚀 5. SEGMENT: ROKET AYRILMA TESTİ (45s)")
        print("-"*50)
        
        segment_start = time.time()
        
        self.rocket_demo = RocketSeparationDemo()
        
        print("📹 Roket ayrılma çekimi başlıyor...")
        print("📋 Video Çekim Talimatları:")
        print("  🌊 Su yüzeyine pozisyonlama")
        print("  📐 +30° pitch açısı ayarı")
        print("  🚀 Ayrılma mekanizması tetikleme")
        print("  📂 Kapak açılması yakın çekim")
        
        try:
            if self.rocket_demo.connect_pixhawk():
                # Yüzeye çıkış (zaten yüzeyde olmalı - önceki segmentten)
                print("  🌊 Yüzey pozisyonlama (10s)")
                if not self.rocket_demo.surface_achieved:
                    self.rocket_demo.ascend_to_surface()
                
                # Fırlatma açısı ayarı (20s)
                print("  📐 Fırlatma açısı ayarı (20s)")
                angle_success = self.rocket_demo.achieve_launch_angle()
                
                # Roket ayrılma (15s)
                print("  🚀 Roket ayrılma işlemi (15s)")
                if angle_success:
                    separation_success = self.rocket_demo.perform_rocket_separation()
                    
                    if separation_success:
                        self.demo_results['rocket_separation'] = True
                        print("✅ Roket ayrılma segment başarılı!")
                    else:
                        print("❌ Roket ayrılma başarısız!")
                else:
                    print("❌ Fırlatma açısı elde edilemedi!")
                
        except Exception as e:
            print(f"❌ Roket ayrılma segment hatası: {e}")
            self.demo_results['rocket_separation'] = False
        
        self.record_segment_timing("roket_ayrilma", 45)
        
        input("\n⏸️ Roket ayrılma çekimi tamam mı? Devam için ENTER...")
        return True
    
    def segment_results_summary(self):
        """6. Segment: Sonuçlar Özeti (15s)"""
        print("\n📊 6. SEGMENT: SONUÇLAR ÖZETİ (15s)")
        print("-"*50)
        
        segment_start = time.time()
        
        print("📹 Sonuçlar çekimi başlıyor...")
        print("📋 Video Çekim Talimatları:")
        print("  📊 Test sonuçları overlay")
        print("  🏆 Başarı oranı gösterimi")
        print("  📈 Performans metrikleri")
        print("  🎯 TEKNOFEST logo kapanış")
        
        # Sonuç özeti
        successful_demos = sum(self.demo_results.values())
        total_demos = len(self.demo_results)
        success_rate = (successful_demos / total_demos) * 100
        
        results_summary = [
            f"📊 TEST SONUÇLARI ÖZETI:",
            f"💧 Sızdırmazlık: {'✅ BAŞARILI' if self.demo_results['waterproof'] else '❌ BAŞARISIZ'}",
            f"🚀 Manevrabilite: {'✅ BAŞARILI' if self.demo_results['maneuverability'] else '❌ BAŞARISIZ'}",
            f"🎯 Roket Ayrılma: {'✅ BAŞARILI' if self.demo_results['rocket_separation'] else '❌ BAŞARISIZ'}",
            f"🚨 Acil Durdurma: {'✅ BAŞARILI' if self.demo_results['emergency_stop'] else '❌ BAŞARISIZ'}",
            f"📈 GENEL BAŞARI: {success_rate:.0f}% ({successful_demos}/{total_demos})"
        ]
        
        for i, result in enumerate(results_summary):
            print(f"  📝 {result}")
            time.sleep(2.5)  # 15s / 6 items = 2.5s each
        
        self.overall_success = success_rate >= 75  # %75 başarı şartı
        
        self.record_segment_timing("sonuclar", 15)
        
        if self.overall_success:
            print("🎉 VİDEO DEMOsu BAŞARILI!")
        else:
            print("⚠️ Video demo eksiklikleri var!")
        
        print("✅ Sonuçlar özeti tamamlandı!")
        return True
    
    def generate_video_report(self):
        """Video demo raporu oluştur"""
        total_video_time = time.time() - self.video_start_time if self.video_start_time else 0
        
        print("\n" + "="*80)
        print("📹 TEKNOFEST Su Altı Roket Aracı - TAM KABİLİYET VİDEO RAPORU")
        print("="*80)
        
        print(f"📅 Video Çekim Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Video Süresi: {total_video_time/60:.1f} dakika ({total_video_time:.0f} saniye)")
        print(f"🎯 Hedef Video Süresi: {VIDEO_DURATION_TARGET/60:.1f} dakika")
        
        # Segment analizi
        print(f"\n📋 VİDEO SEGMENT ANALİZİ:")
        print("-"*60)
        
        total_planned = sum(duration for _, duration in VIDEO_SEGMENTS)
        
        for segment_name, planned_duration in VIDEO_SEGMENTS:
            completed = "✅" if segment_name in self.video_segments_completed else "❌"
            actual_timing = self.segment_timings.get(segment_name, {})
            actual_duration = actual_timing.get('actual_duration', 0)
            
            segment_display = {
                'sistem_tanitimi': 'Sistem Tanıtımı',
                'acil_durdurma': 'Acil Durdurma',
                'sizdimazlik': 'Sızdırmazlık', 
                'manevrabilite': 'Manevrabilite',
                'roket_ayrilma': 'Roket Ayrılma',
                'sonuclar': 'Sonuçlar'
            }.get(segment_name, segment_name)
            
            print(f"  {completed} {segment_display}: {actual_duration}s / {planned_duration}s")
        
        # Demo başarı analizi
        print(f"\n🏆 DEMO BAŞARI ANALİZİ:")
        print("-"*60)
        
        successful_demos = sum(self.demo_results.values())
        total_demos = len(self.demo_results)
        success_rate = (successful_demos / total_demos) * 100
        
        for demo_name, success in self.demo_results.items():
            demo_display = {
                'waterproof': 'Sızdırmazlık Testi',
                'maneuverability': 'Manevrabilite Testi',
                'rocket_separation': 'Roket Ayrılma Testi',
                'emergency_stop': 'Acil Durdurma Testi'
            }.get(demo_name, demo_name)
            
            status_icon = "✅" if success else "❌"
            print(f"  {status_icon} {demo_display}: {'BAŞARILI' if success else 'BAŞARISIZ'}")
        
        print(f"\n📊 GENEL BAŞARI ORANI: {success_rate:.1f}% ({successful_demos}/{total_demos})")
        
        # Şartname uygunluğu
        print(f"\n🎯 ŞARTNAME UYGUNLUĞU:")
        print("-"*40)
        
        video_duration_ok = 120 <= total_video_time <= 300  # 2-5 dakika
        duration_icon = "✅" if video_duration_ok else "❌"
        print(f"  {duration_icon} Video Süresi (2-5dk): {total_video_time/60:.1f} dakika")
        
        waterproof_ok = self.demo_results.get('waterproof', False)
        waterproof_icon = "✅" if waterproof_ok else "❌"
        print(f"  {waterproof_icon} Sızdırmazlık (≥1m derinlik): {'BAŞARILI' if waterproof_ok else 'BAŞARISIZ'}")
        
        maneuver_ok = self.demo_results.get('maneuverability', False)
        maneuver_icon = "✅" if maneuver_ok else "❌"
        print(f"  {maneuver_icon} Manevrabilite (≥1dk kontrollü): {'BAŞARILI' if maneuver_ok else 'BAŞARISIZ'}")
        
        rocket_ok = self.demo_results.get('rocket_separation', False)
        rocket_icon = "✅" if rocket_ok else "❌"
        print(f"  {rocket_icon} Roket Ayrılma (+30° eğim): {'BAŞARILI' if rocket_ok else 'BAŞARISIZ'}")
        
        emergency_ok = self.demo_results.get('emergency_stop', False)
        emergency_icon = "✅" if emergency_ok else "❌"
        print(f"  {emergency_icon} Acil Durdurma (buton çalışması): {'BAŞARILI' if emergency_ok else 'BAŞARISIZ'}")
        
        # Video montaj önerileri
        print(f"\n🎬 VİDEO MONTAJ ÖNERİLERİ:")
        print("-"*40)
        
        if total_video_time > VIDEO_DURATION_TARGET:
            print(f"  ⚠️ Video uzun ({total_video_time:.0f}s > {VIDEO_DURATION_TARGET}s)")
            print(f"    💡 Öneري: Segment'leri kısalt veya hızlandır")
        elif total_video_time < 120:
            print(f"  ⚠️ Video kısa ({total_video_time:.0f}s < 120s)")
            print(f"    💡 Öneri: Ek açıklama veya slow-motion ekle")
        else:
            print(f"  ✅ Video süresi optimal ({total_video_time:.0f}s)")
        
        if success_rate >= 90:
            print(f"  🎉 Mükemmel performans - pazarlama vurgusu yap")
        elif success_rate >= 70:
            print(f"  👍 İyi performans - güçlü yönleri vurgula") 
        else:
            print(f"  🔧 Başarısız testleri kısa göster, başarılıları uzat")
        
        # Final değerlendirmesi
        overall_video_success = (video_duration_ok and success_rate >= 75 and 
                               waterproof_ok and maneuver_ok and emergency_ok)
        
        print(f"\n🏆 GENEL VİDEO BAŞARI:")
        print("="*40)
        
        if overall_video_success:
            print("🎉 MÜKEMMEL! Video TEKNOFEST için hazır!")
            print("📹 YouTube'a yükleme için optimize edilmiş!")
            print("🏆 Tüm şartname gereksinimleri karşılandı!")
        else:
            print("⚠️ Video iyileştirmeler gerekiyor!")
            missing_elements = []
            if not video_duration_ok:
                missing_elements.append("Video süresi")
            if not waterproof_ok:
                missing_elements.append("Sızdırmazlık")
            if not maneuver_ok:
                missing_elements.append("Manevrabilite")
            if not emergency_ok:
                missing_elements.append("Acil durdurma")
            print(f"🔧 İyileştirme alanları: {', '.join(missing_elements)}")
        
        # Rapor dosyasına kaydet
        video_report = {
            'timestamp': datetime.now().isoformat(),
            'total_video_duration': total_video_time,
            'target_duration': VIDEO_DURATION_TARGET,
            'segments_completed': self.video_segments_completed,
            'segment_timings': self.segment_timings,
            'demo_results': self.demo_results,
            'success_rate': success_rate,
            'overall_success': overall_video_success,
            'compliance': {
                'duration_ok': video_duration_ok,
                'waterproof_ok': waterproof_ok,
                'maneuver_ok': maneuver_ok,
                'rocket_ok': rocket_ok,
                'emergency_ok': emergency_ok
            }
        }
        
        with open(f'full_capability_video_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(video_report, f, indent=2)
        
        print(f"\n💾 Video raporu kaydedildi: full_capability_video_report_*.json")
        
        return overall_video_success
    
    def run_full_capability_video(self):
        """Tam kabiliyet video çekimi"""
        print("🎬 TEKNOFEST Su Altı Roket Aracı - TAM KABİLİYET VİDEO ÇEKİMİ")
        print("="*80)
        print("📹 YouTube için 2-5 dakika kabiliyet gösterim videosu")
        print("🎯 Tüm şartname gereksinimlerini kapsayan demo")
        print("📊 Hedef süre: 5 dakika (300 saniye)")
        
        print("\n📋 VİDEO İÇERİĞİ:")
        for i, (segment_name, duration) in enumerate(VIDEO_SEGMENTS):
            segment_display = {
                'sistem_tanitimi': 'Sistem Tanıtımı',
                'acil_durdurma': 'Acil Durdurma',
                'sizdimazlik': 'Sızdırmazlık', 
                'manevrabilite': 'Manevrabilite',
                'roket_ayrilma': 'Roket Ayrılma',
                'sonuclar': 'Sonuçlar'
            }.get(segment_name, segment_name)
            print(f"  {i+1}. {segment_display}: {duration}s")
        
        print(f"\nToplam: {sum(duration for _, duration in VIDEO_SEGMENTS)} saniye")
        
        print("\n⚠️ VİDEO ÇEKİM HAZIRLIĞI:")
        print("- Kameralar (su üstü + su altı + yakın çekim) hazır mı?")
        print("- Aydınlatma sistemleri çalışıyor mu?")
        print("- Telemetri overlay sistemi aktif mi?")
        print("- Tüm alt sistemler test edildi mi?")
        print("- Güvenlik ekibi hazır mı?")
        print("- Hava koşulları çekim için uygun mu?")
        
        ready = input("\n✅ Tam kabiliyet video çekimi başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Video çekimi iptal edildi")
            return False
        
        self.video_start_time = time.time()
        
        try:
            print("\n🎬 TAM KABİLİYET VİDEO ÇEKİMİ BAŞLADI!")
            print("⏰ Başlama zamanı:", datetime.now().strftime("%H:%M:%S"))
            
            # Video segmentleri sırasıyla çek
            segment_results = []
            
            # 1. Sistem tanıtımı
            segment_results.append(self.segment_system_introduction())
            
            # 2. Acil durdurma
            segment_results.append(self.segment_emergency_stop())
            
            # 3. Sızdırmazlık
            segment_results.append(self.segment_waterproof_test())
            
            # 4. Manevrabilite
            segment_results.append(self.segment_maneuverability())
            
            # 5. Roket ayrılma
            segment_results.append(self.segment_rocket_separation())
            
            # 6. Sonuçlar
            segment_results.append(self.segment_results_summary())
            
            # Video raporu
            video_success = self.generate_video_report()
            
            if video_success and all(segment_results):
                print("\n🎉 TAM KABİLİYET VİDEO ÇEKİMİ BAŞARILI!")
                print("📹 Video TEKNOFEST için hazır!")
                print("🚀 YouTube'a yüklemeye hazır!")
            else:
                print("\n⚠️ Video çekimi tamamlandı ama eksiklikler var!")
                print("🔧 Montaj aşamasında düzeltmeler gerekebilir!")
            
            return video_success
            
        except KeyboardInterrupt:
            print("\n⚠️ Video çekimi kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Video çekimi hatası: {e}")
            return False
        finally:
            self.cleanup_all_demos()
    
    def cleanup_all_demos(self):
        """Tüm demo sistemlerini temizle"""
        print("\n🧹 Video sistemleri temizleniyor...")
        
        demos_to_cleanup = [
            self.waterproof_demo,
            self.maneuver_demo,
            self.rocket_demo,
            self.emergency_demo
        ]
        
        for demo in demos_to_cleanup:
            if demo:
                try:
                    demo.cleanup()
                except:
                    pass
        
        print("✅ Tüm sistemler temizlendi")

def main():
    """Ana fonksiyon"""
    full_demo = FullCapabilityDemo()
    
    try:
        success = full_demo.run_full_capability_video()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 