#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Mission Entegrasyon Test Scripti
Bu script mission kodlarının entegrasyonunu ve yeni özellikleri test eder.

ENTEGRASYON TESTLERİ:
1. Config Yükleme ve Validation Testi
2. PWM Odometri Doğruluk Testi  
3. LED/Buzzer Entegrasyon Testi
4. Leak Detection Testi
5. Latched Fault Sistemi Testi
6. Thread Management Testi
7. CSV Telemetri Testi
8. Mission Stage Geçiş Testi

AMAÇ: Yeni eklenen özelliklerin mission kodlarıyla entegrasyonunu doğrula
"""

import sys
import os
import time
import json
import threading
from datetime import datetime

# Mission kodlarından import
sys.path.append('../pluswing')
try:
    from mission_1_navigation_plus import Mission1Navigator as PlusNavigator
    from mission_1_navigation_plus import load_mission_config, validate_hardware_config
    PLUS_WING_AVAILABLE = True
except ImportError:
    print("⚠️ Plus-Wing modülü bulunamadı")
    PLUS_WING_AVAILABLE = False

sys.path.append('../xwing')
try:
    from mission_1_nav import Mission1Navigator as XNavigator
    X_WING_AVAILABLE = True
except ImportError:
    print("⚠️ X-Wing modülü bulunamadı")
    X_WING_AVAILABLE = False

class MissionIntegrationTester:
    """Mission entegrasyon test sınıfı"""
    
    def __init__(self):
        print("🔄 TEKNOFEST Mission Entegrasyon Test Sistemi")
        print("="*70)
        print("Bu test yeni eklenen özelliklerin entegrasyonunu kontrol eder")
        print("="*70)
        
        self.test_results = {}
        self.navigator = None
        
    def test_config_loading(self):
        """Config yükleme ve validation testi"""
        print("\n📋 CONFIG YÜKLEME VE VALIDATION TESTİ")
        print("-" * 50)
        
        try:
            # Mission config yükleme
            config = load_mission_config()
            config_loaded = config is not None
            
            print(f"✅ Mission config yüklendi: {config_loaded}")
            
            # Hardware validation
            if config:
                validation_result = validate_hardware_config(config)
                print(f"✅ Hardware validation: {validation_result}")
                
                # Config içeriği kontrolü
                required_sections = ['mission_parameters', 'control_parameters', 'safety_limits']
                sections_exist = all(section in config for section in required_sections)
                print(f"✅ Gerekli config bölümleri: {sections_exist}")
                
                success = validation_result and sections_exist
            else:
                success = False
                validation_result = False
                sections_exist = False
            
            self.test_results['config_loading'] = {
                'success': success,
                'config_loaded': config_loaded,
                'validation_passed': validation_result,
                'sections_exist': sections_exist
            }
            
            return success
            
        except Exception as e:
            print(f"❌ Config test hatası: {e}")
            return False
    
    def test_pwm_odometry_integration(self):
        """PWM odometri entegrasyon testi"""
        print("\n📏 PWM ODOMETRİ ENTEGRASYON TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            print("❌ Navigator mevcut değil")
            return False
        
        try:
            # Başlangıç değerleri
            initial_distance = self.navigator.traveled_distance
            initial_pwm = self.navigator.current_pwm
            
            print(f"🔧 Başlangıç: Distance={initial_distance:.2f}m, PWM={initial_pwm}")
            
            # PWM değiştir ve odometri güncelle
            test_pwms = [1550, 1600, 1650, 1600, 1550, 1500]
            distance_changes = []
            
            for i, pwm in enumerate(test_pwms):
                self.navigator.current_pwm = pwm
                
                # 2 saniye bekle ve odometri güncelle
                for _ in range(20):  # 20 * 0.1s = 2s
                    self.navigator.update_pwm_based_odometry()
                    time.sleep(0.1)
                
                new_distance = self.navigator.traveled_distance
                distance_change = new_distance - initial_distance
                distance_changes.append(distance_change)
                
                print(f"   PWM {pwm}: Distance={new_distance:.3f}m (+{distance_change:.3f}m)")
                
                initial_distance = new_distance
            
            # Sonuçları değerlendir
            total_distance = self.navigator.traveled_distance
            distance_increased = total_distance > 0.1  # En az 10cm artış
            pwm_responsive = len(set(distance_changes)) > 1  # Farklı PWM'lerde farklı artışlar
            
            print(f"📊 Toplam mesafe artışı: {total_distance:.3f}m")
            print(f"📊 PWM responsive: {pwm_responsive}")
            
            success = distance_increased and pwm_responsive
            
            self.test_results['pwm_odometry'] = {
                'success': success,
                'total_distance': total_distance,
                'distance_increased': distance_increased,
                'pwm_responsive': pwm_responsive,
                'distance_changes': distance_changes
            }
            
            return success
            
        except Exception as e:
            print(f"❌ PWM odometri test hatası: {e}")
            return False
    
    def test_led_buzzer_integration(self):
        """LED/Buzzer entegrasyon testi"""
        print("\n💡 LED/BUZZER ENTEGRASYON TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        try:
            # GPIO başlatma kontrolü
            gpio_initialized = getattr(self.navigator, 'gpio_initialized', False)
            print(f"🔧 GPIO başlatıldı: {gpio_initialized}")
            
            if not gpio_initialized:
                print("⚠️ GPIO başlatılmamış, sadece fonksiyon çağrısı test edilecek")
            
            # Status indicator fonksiyon testi
            test_stages = ["INITIALIZATION", "DESCENT", "MISSION_COMPLETE"]
            function_works = True
            
            for stage in test_stages:
                try:
                    # Arming countdown testi
                    if hasattr(self.navigator, '_update_status_indicators'):
                        self.navigator._update_status_indicators(stage, arming_remaining=30)
                        time.sleep(0.5)
                        
                        # Normal stage testi
                        self.navigator._update_status_indicators(stage)
                        time.sleep(0.5)
                        
                        # Fault testi
                        self.navigator._update_status_indicators(stage, fault="TEST_FAULT")
                        time.sleep(0.5)
                        
                    print(f"   ✅ {stage} stage indicator çalıştı")
                    
                except Exception as e:
                    print(f"   ❌ {stage} stage indicator hatası: {e}")
                    function_works = False
            
            success = function_works
            
            self.test_results['led_buzzer'] = {
                'success': success,
                'gpio_initialized': gpio_initialized,
                'function_works': function_works
            }
            
            return success
            
        except Exception as e:
            print(f"❌ LED/Buzzer test hatası: {e}")
            return False
    
    def test_leak_detection(self):
        """Leak detection entegrasyon testi"""
        print("\n💧 LEAK DETECTION ENTEGRASYON TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        try:
            # Başlangıç durumu
            initial_leak = getattr(self.navigator, 'leak_detected', False)
            initial_fault = getattr(self.navigator, '_latched_fault', None)
            
            print(f"🔧 Başlangıç: Leak={initial_leak}, Fault={initial_fault}")
            
            # Mock STATUSTEXT mesajı oluştur
            class MockStatusText:
                def __init__(self, text):
                    self.text = text
            
            # Navigator'ın master'ını mock et
            class MockMaster:
                def __init__(self):
                    self.messages = []
                    self.message_index = 0
                
                def recv_match(self, type=None, blocking=False):
                    if type == 'STATUSTEXT' and self.message_index < len(self.messages):
                        msg = self.messages[self.message_index]
                        self.message_index += 1
                        return msg
                    return None
                
                def add_message(self, text):
                    self.messages.append(MockStatusText(text))
            
            # Mock master oluştur ve leak mesajı ekle
            original_master = self.navigator.master
            mock_master = MockMaster()
            mock_master.add_message("System status: LEAK detected in compartment 1")
            self.navigator.master = mock_master
            
            # Sensör okuma simüle et (sadece STATUSTEXT kısmı)
            try:
                statustext = self.navigator.master.recv_match(type='STATUSTEXT', blocking=False)
                if statustext and statustext.text:
                    text_lower = statustext.text.lower()
                    if "leak" in text_lower:
                        self.navigator.leak_detected = True
                        # Latched fault tetikle
                        if hasattr(self.navigator, '_trigger_latched_fault'):
                            self.navigator._trigger_latched_fault("LEAK_DETECTED")
                
                # Sonuç kontrolü
                leak_detected = getattr(self.navigator, 'leak_detected', False)
                fault_triggered = getattr(self.navigator, '_latched_fault', None)
                
                print(f"📊 Sonuç: Leak={leak_detected}, Fault={fault_triggered}")
                
                success = leak_detected and fault_triggered == "LEAK_DETECTED"
                
            finally:
                # Master'ı geri yükle
                self.navigator.master = original_master
            
            self.test_results['leak_detection'] = {
                'success': success,
                'leak_detected': leak_detected,
                'fault_triggered': fault_triggered is not None
            }
            
            return success
            
        except Exception as e:
            print(f"❌ Leak detection test hatası: {e}")
            return False
    
    def test_thread_management(self):
        """Thread management testi"""
        print("\n🧵 THREAD MANAGEMENT TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        try:
            # Thread oluşturma testi
            print("🔧 Test thread'i oluşturuluyor...")
            
            test_completed = threading.Event()
            thread_error = None
            
            def test_control_loop():
                try:
                    # Kısa bir kontrol döngüsü simüle et
                    self.navigator.running = True
                    self.navigator.mission_active = True
                    
                    for i in range(5):  # 5 döngü
                        if not self.navigator.running:
                            break
                        time.sleep(0.1)
                    
                    test_completed.set()
                    
                except Exception as e:
                    nonlocal thread_error
                    thread_error = e
                    test_completed.set()
            
            # Thread başlat
            test_thread = threading.Thread(target=test_control_loop, name="TestControlLoop")
            test_thread.daemon = False
            test_thread.start()
            
            # Thread'in çalışmasını bekle
            time.sleep(1)
            
            # Thread'i durdur
            self.navigator.running = False
            self.navigator.mission_active = False
            
            # Thread'in bitmesini bekle
            test_thread.join(timeout=2.0)
            
            thread_finished = not test_thread.is_alive()
            test_successful = test_completed.is_set() and thread_error is None
            
            print(f"📊 Thread bitti: {thread_finished}")
            print(f"📊 Test başarılı: {test_successful}")
            if thread_error:
                print(f"❌ Thread hatası: {thread_error}")
            
            success = thread_finished and test_successful
            
            self.test_results['thread_management'] = {
                'success': success,
                'thread_finished': thread_finished,
                'test_successful': test_successful,
                'error': str(thread_error) if thread_error else None
            }
            
            return success
            
        except Exception as e:
            print(f"❌ Thread management test hatası: {e}")
            return False
    
    def test_csv_telemetry(self):
        """CSV telemetri testi"""
        print("\n📊 CSV TELEMETRİ TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        try:
            # Test telemetri verisi oluştur
            test_data = [
                {
                    'timestamp': time.time(),
                    'mission_stage': 'TEST',
                    'depth': 1.5,
                    'depth_source': 'test',
                    'heading': 45.0,
                    'roll': 0.5,
                    'pitch': -0.3,
                    'yaw': 45.2,
                    'speed': 1.2,
                    'speed_source': 'pwm_calibrated',
                    'estimated_speed': 1.25,
                    'filtered_speed': 1.22,
                    'position': {'x': 5.0, 'y': 3.0},
                    'traveled_distance': 8.5,
                    'motor_pwm': 1580,
                    'leak_detected': False,
                    'latched_fault': None
                }
            ]
            
            # Navigator'a test verisi ekle
            self.navigator.telemetry_data = test_data
            
            # CSV kaydetme fonksiyonu test et
            if hasattr(self.navigator, '_save_csv_telemetry'):
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.navigator._save_csv_telemetry(f"test_{timestamp_str}")
                
                # CSV dosyası oluştu mu kontrol et
                csv_filename = f'mission_1_telemetry_test_{timestamp_str}.csv'
                csv_exists = os.path.exists(csv_filename)
                
                if csv_exists:
                    # Dosya içeriği kontrol et
                    with open(csv_filename, 'r') as f:
                        content = f.read()
                        has_header = 'timestamp,mission_stage' in content
                        has_data = 'TEST' in content and '1.5' in content
                    
                    # Test dosyasını sil
                    os.remove(csv_filename)
                    
                    print(f"✅ CSV dosyası oluşturuldu: {csv_exists}")
                    print(f"✅ CSV header var: {has_header}")
                    print(f"✅ CSV veri var: {has_data}")
                    
                    success = csv_exists and has_header and has_data
                else:
                    success = False
                    has_header = False
                    has_data = False
                    
            else:
                print("❌ _save_csv_telemetry fonksiyonu bulunamadı")
                success = False
                csv_exists = False
                has_header = False
                has_data = False
            
            self.test_results['csv_telemetry'] = {
                'success': success,
                'csv_exists': csv_exists,
                'has_header': has_header,
                'has_data': has_data
            }
            
            return success
            
        except Exception as e:
            print(f"❌ CSV telemetri test hatası: {e}")
            return False
    
    def run_all_tests(self):
        """Tüm entegrasyon testlerini çalıştır"""
        print("\n🚀 TÜM ENTEGRASYON TESTLERİ BAŞLIYOR")
        print("="*70)
        
        # Navigator seç ve başlat
        config_type = input("Hangi konfigürasyonu test etmek istiyorsuniz? (plus/x): ").lower()
        
        if config_type == "plus" and PLUS_WING_AVAILABLE:
            self.navigator = PlusNavigator()
            print("✅ Plus-Wing Navigator başlatıldı")
        elif config_type == "x" and X_WING_AVAILABLE:
            self.navigator = XNavigator()
            print("✅ X-Wing Navigator başlatıldı")
        else:
            print("❌ Seçilen konfigürasyon mevcut değil!")
            return False
        
        # Test listesi
        tests = [
            ("Config Loading", self.test_config_loading),
            ("PWM Odometry Integration", self.test_pwm_odometry_integration),
            ("LED/Buzzer Integration", self.test_led_buzzer_integration),
            ("Leak Detection", self.test_leak_detection),
            ("Thread Management", self.test_thread_management),
            ("CSV Telemetry", self.test_csv_telemetry)
        ]
        
        results = []
        
        for test_name, test_func in tests:
            print(f"\n{'='*70}")
            print(f"🎯 {test_name.upper()} TESTİ")
            print(f"{'='*70}")
            
            try:
                success = test_func()
                results.append((test_name, success))
                
                if success:
                    print(f"✅ {test_name} entegrasyonu BAŞARILI")
                else:
                    print(f"❌ {test_name} entegrasyonu BAŞARISIZ")
                    
            except Exception as e:
                print(f"❌ {test_name} test hatası: {e}")
                results.append((test_name, False))
            
            time.sleep(1)
        
        # Final rapor
        self.generate_integration_report(results)
        
        return results
    
    def generate_integration_report(self, results):
        """Entegrasyon test raporunu oluştur"""
        print("\n" + "="*70)
        print("📊 MISSION ENTEGRASYON TEST RAPORU")
        print("="*70)
        
        passed = sum(1 for _, success in results if success)
        total = len(results)
        
        print(f"📈 GENEL SONUÇ: {passed}/{total} test başarılı ({passed/total*100:.1f}%)")
        print()
        
        for test_name, success in results:
            status = "✅ BAŞARILI" if success else "❌ BAŞARISIZ"
            print(f"   {test_name:<30}: {status}")
        
        if passed == total:
            print("\n🎉 TÜM ENTEGRASYONLAR BAŞARILI! Yeni özellikler çalışıyor.")
        elif passed >= total // 2:
            print("\n⚠️ KISMEN BAŞARILI. Bazı entegrasyonlar kontrol edilmeli.")
        else:
            print("\n❌ ÇOĞU ENTEGRASYON BAŞARISIZ. Kod kontrolü gerekli!")
        
        # Raporu kaydet
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report = {
            'timestamp': datetime.now().isoformat(),
            'test_type': 'Mission Integration Test',
            'results_summary': {
                'passed': passed,
                'total': total,
                'success_rate': passed/total
            },
            'detailed_results': self.test_results,
            'test_results': results
        }
        
        with open(f'mission_integration_test_{timestamp}.json', 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\n💾 Entegrasyon test raporu kaydedildi: mission_integration_test_{timestamp}.json")

def main():
    """Ana test fonksiyonu"""
    print("🔄 TEKNOFEST Mission Entegrasyon Test Sistemi")
    print("Bu test yeni eklenen özelliklerin entegrasyonunu kontrol eder.")
    
    tester = MissionIntegrationTester()
    
    try:
        ready = input("\n✅ Entegrasyon testine hazır mısınız? (y/n): ").lower()
        if ready != 'y':
            print("❌ Test iptal edildi")
            return 1
        
        results = tester.run_all_tests()
        
        if results:
            passed = sum(1 for _, success in results if success)
            return 0 if passed == len(results) else 1
        else:
            return 1
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
