#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Kanat Konfigürasyonu Karşılaştırma Test Scripti
Bu script X-Wing ve Plus-Wing konfigürasyonlarını karşılaştırır.

Karşılaştırma Kriterleri:
1. Stabilizasyon Tepki Hızı
2. Hassasiyet ve Doğruluk
3. Enerji Verimliliği
4. Kontrol Otoritesi
5. Genel Performans

Hardware: AUX 1→Motor, AUX 3,4,5,6→Kanatlar
"""

import sys
import os
import time
import math
import json
from datetime import datetime

# Test modüllerini import et
from test_x_wing_stabilization import XWingStabilizationTester
from test_plus_wing_stabilization import PlusWingStabilizationTester

class WingConfigurationComparison:
    """Kanat konfigürasyonu karşılaştırma sınıfı"""
    
    def __init__(self):
        print("🚀 TEKNOFEST Kanat Konfigürasyonu Karşılaştırma Sistemi")
        print("="*70)
        
        self.x_wing_tester = None
        self.plus_wing_tester = None
        self.comparison_results = {}
        
    def run_x_wing_tests(self):
        """X-Wing testlerini çalıştır"""
        print("\n" + "="*70)
        print("🔥 X-WING KONFİGÜRASYONU TESTLERİ")
        print("="*70)
        
        self.x_wing_tester = XWingStabilizationTester()
        
        if not self.x_wing_tester.connect_and_initialize():
            print("❌ X-Wing sistemi başlatılamadı!")
            return False
        
        print("✅ X-Wing sistemi hazır. Testler başlıyor...")
        
        # X-Wing testlerini çalıştır
        x_wing_start = time.time()
        
        try:
            # Test sırası
            tests = [
                ("Roll", self.x_wing_tester.test_roll_stabilization),
                ("Pitch", self.x_wing_tester.test_pitch_stabilization),
                ("Yaw", self.x_wing_tester.test_yaw_stabilization),
                ("Odometry", self.x_wing_tester.test_distance_odometry)
            ]
            
            x_wing_results = {}
            
            for test_name, test_func in tests:
                print(f"\n🎯 X-Wing {test_name} Testi...")
                
                # Kanatları nötr yap
                self.x_wing_tester.navigator.set_control_surfaces(0, 0, 0)
                time.sleep(1.0)
                
                # Testi çalıştır
                success = test_func()
                x_wing_results[test_name.lower()] = success
                
                if success:
                    print(f"✅ X-Wing {test_name} testi başarılı")
                else:
                    print(f"❌ X-Wing {test_name} testi başarısız")
                
                time.sleep(2.0)  # Testler arası ara
            
            x_wing_duration = time.time() - x_wing_start
            
            print(f"\n📊 X-Wing Test Özeti:")
            print(f"   Toplam süre: {x_wing_duration:.1f}s")
            print(f"   Başarı oranı: {sum(x_wing_results.values())}/{len(x_wing_results)}")
            
            self.comparison_results['x_wing'] = {
                'results': x_wing_results,
                'duration': x_wing_duration,
                'success_rate': sum(x_wing_results.values()) / len(x_wing_results),
                'test_data': self.x_wing_tester.test_results
            }
            
            return True
            
        except Exception as e:
            print(f"❌ X-Wing test hatası: {e}")
            return False
        finally:
            self.x_wing_tester.cleanup()
    
    def run_plus_wing_tests(self):
        """Plus-Wing testlerini çalıştır"""
        print("\n" + "="*70)
        print("➕ PLUS-WING KONFİGÜRASYONU TESTLERİ")
        print("="*70)
        
        self.plus_wing_tester = PlusWingStabilizationTester()
        
        if not self.plus_wing_tester.connect_and_initialize():
            print("❌ Plus-Wing sistemi başlatılamadı!")
            return False
        
        print("✅ Plus-Wing sistemi hazır. Testler başlıyor...")
        
        # Plus-Wing testlerini çalıştır
        plus_wing_start = time.time()
        
        try:
            # Test sırası
            tests = [
                ("Roll", self.plus_wing_tester.test_roll_stabilization),
                ("Pitch", self.plus_wing_tester.test_pitch_stabilization),
                ("Yaw", self.plus_wing_tester.test_yaw_stabilization),
                ("Odometry", self.plus_wing_tester.test_distance_odometry)
            ]
            
            plus_wing_results = {}
            
            for test_name, test_func in tests:
                print(f"\n🎯 Plus-Wing {test_name} Testi...")
                
                # Kanatları nötr yap
                self.plus_wing_tester.navigator.set_control_surfaces(0, 0, 0)
                time.sleep(1.0)
                
                # Testi çalıştır
                success = test_func()
                plus_wing_results[test_name.lower()] = success
                
                if success:
                    print(f"✅ Plus-Wing {test_name} testi başarılı")
                else:
                    print(f"❌ Plus-Wing {test_name} testi başarısız")
                
                time.sleep(2.0)  # Testler arası ara
            
            plus_wing_duration = time.time() - plus_wing_start
            
            print(f"\n📊 Plus-Wing Test Özeti:")
            print(f"   Toplam süre: {plus_wing_duration:.1f}s")
            print(f"   Başarı oranı: {sum(plus_wing_results.values())}/{len(plus_wing_results)}")
            
            self.comparison_results['plus_wing'] = {
                'results': plus_wing_results,
                'duration': plus_wing_duration,
                'success_rate': sum(plus_wing_results.values()) / len(plus_wing_results),
                'test_data': self.plus_wing_tester.test_results
            }
            
            return True
            
        except Exception as e:
            print(f"❌ Plus-Wing test hatası: {e}")
            return False
        finally:
            self.plus_wing_tester.cleanup()
    
    def analyze_comparison(self):
        """Karşılaştırma analizini yap"""
        print("\n" + "="*70)
        print("📊 KANAT KONFİGÜRASYONU KARŞILAŞTIRMA ANALİZİ")
        print("="*70)
        
        if 'x_wing' not in self.comparison_results or 'plus_wing' not in self.comparison_results:
            print("❌ Karşılaştırma için yeterli veri yok!")
            return
        
        x_data = self.comparison_results['x_wing']
        plus_data = self.comparison_results['plus_wing']
        
        # Genel karşılaştırma
        print("🔍 GENEL PERFORMANS KARŞILAŞTIRMASI")
        print("-" * 50)
        
        print(f"{'Konfigürasyon':<15} {'Başarı Oranı':<12} {'Test Süresi':<12} {'Genel Puan'}")
        print("-" * 50)
        
        x_score = x_data['success_rate'] * 100
        plus_score = plus_data['success_rate'] * 100
        
        print(f"{'X-Wing':<15} {x_data['success_rate']:.1%} {x_data['duration']:.1f}s        {x_score:.1f}/100")
        print(f"{'Plus-Wing':<15} {plus_data['success_rate']:.1%} {plus_data['duration']:.1f}s        {plus_score:.1f}/100")
        
        # Detaylı test karşılaştırması
        print(f"\n🔬 DETAYLI TEST KARŞILAŞTIRMASI")
        print("-" * 50)
        
        test_names = ['roll', 'pitch', 'yaw', 'odometry']
        
        for test in test_names:
            x_success = x_data['results'].get(test, False)
            plus_success = plus_data['results'].get(test, False)
            
            x_status = "✅" if x_success else "❌"
            plus_status = "✅" if plus_success else "❌"
            
            print(f"{test.capitalize():<12}: X-Wing {x_status}  |  Plus-Wing {plus_status}")
        
        # Öneriler
        print(f"\n💡 ÖNERİLER VE DEĞERLENDİRME")
        print("-" * 50)
        
        if x_score > plus_score:
            winner = "X-Wing"
            difference = x_score - plus_score
        elif plus_score > x_score:
            winner = "Plus-Wing"
            difference = plus_score - x_score
        else:
            winner = "Berabere"
            difference = 0
        
        if winner != "Berabere":
            print(f"🏆 Kazanan: {winner} ({difference:.1f} puan farkla)")
        else:
            print(f"🤝 Sonuç: Berabere")
        
        # Konfigürasyon önerileri
        print(f"\n🎯 GÖREV ÖNERİLERİ:")
        
        if x_data['results'].get('roll', False) and x_data['results'].get('pitch', False):
            print("   • X-Wing: Roll/Pitch kontrolü güçlü → Hızlı manevra gereken görevler")
        
        if plus_data['results'].get('yaw', False):
            print("   • Plus-Wing: Yaw kontrolü güçlü → Hassas yön kontrolü gereken görevler")
        
        if x_data['success_rate'] > 0.8:
            print("   • X-Wing: Genel olarak güvenilir → Ana görevler için öncelikli")
        
        if plus_data['success_rate'] > 0.8:
            print("   • Plus-Wing: Genel olarak güvenilir → Ana görevler için öncelikli")
        
        # Sonuçları kaydet
        self.save_comparison_report()
    
    def save_comparison_report(self):
        """Karşılaştırma raporunu kaydet"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"wing_comparison_report_{timestamp}.json"
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'test_type': 'Wing Configuration Comparison',
            'comparison_results': self.comparison_results,
            'summary': {
                'x_wing_success_rate': self.comparison_results.get('x_wing', {}).get('success_rate', 0),
                'plus_wing_success_rate': self.comparison_results.get('plus_wing', {}).get('success_rate', 0),
                'winner': self._determine_winner()
            }
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\n💾 Karşılaştırma raporu kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Rapor kayıt hatası: {e}")
    
    def _determine_winner(self):
        """Kazananı belirle"""
        if 'x_wing' not in self.comparison_results or 'plus_wing' not in self.comparison_results:
            return "Unknown"
        
        x_score = self.comparison_results['x_wing']['success_rate']
        plus_score = self.comparison_results['plus_wing']['success_rate']
        
        if x_score > plus_score:
            return "X-Wing"
        elif plus_score > x_score:
            return "Plus-Wing"
        else:
            return "Tie"
    
    def run_full_comparison(self):
        """Tam karşılaştırma testini çalıştır"""
        print("🚀 TEKNOFEST Kanat Konfigürasyonu Karşılaştırma Testi Başlıyor...")
        print("Bu test her iki konfigürasyonu da test ederek karşılaştırma yapar.")
        
        try:
            # Kullanıcı onayı
            ready = input("\n✅ Karşılaştırma testine hazır mısınız? (y/n): ").lower()
            if ready != 'y':
                print("❌ Test iptal edildi")
                return 1
            
            print("\n⚠️ NOT: Test sırasında her iki konfigürasyon da test edilecek.")
            print("        Pixhawk'ı manuel hareket ettirmeyi unutmayın!")
            
            input("\nDevam etmek için Enter'a basın...")
            
            # X-Wing testleri
            if not self.run_x_wing_tests():
                print("❌ X-Wing testleri başarısız!")
                return 1
            
            print("\n⏱️ X-Wing testleri tamamlandı. Plus-Wing testlerine geçiliyor...")
            time.sleep(5.0)  # Konfigürasyon değişimi için bekle
            
            # Plus-Wing testleri
            if not self.run_plus_wing_tests():
                print("❌ Plus-Wing testleri başarısız!")
                return 1
            
            # Karşılaştırma analizi
            self.analyze_comparison()
            
            return 0
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return 1
        except Exception as e:
            print(f"❌ Test hatası: {e}")
            return 1

def main():
    """Ana fonksiyon"""
    comparison = WingConfigurationComparison()
    return comparison.run_full_comparison()

if __name__ == "__main__":
    import sys
    sys.exit(main())
