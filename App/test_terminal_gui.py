#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Terminal GUI Test
Tuş kontrol test scripti
"""

def test_keyboard_mappings():
    """Tuş eşlemelerini test et"""
    print("🎮 TEKNOFEST Terminal GUI - Tuş Test")
    print("=" * 50)
    
    # Test tuşları
    test_keys = {
        'W/S': 'Pitch kontrol',
        'A/D': 'Roll kontrol', 
        'Q/E': 'Yaw kontrol',
        'O/L': 'Motor kontrol',
        'X': 'Servo sıfırlama',
        'V': 'Vibration menü',
        'ESC/P': 'Çıkış'
    }
    
    print("🔤 TUŞ EŞLEMELERİ:")
    for keys, function in test_keys.items():
        print(f"   {keys:8} → {function}")
    
    print("\n✅ TUŞ SİSTEMİ:")
    print("   • WASD servo kontrol - ANLıK hareket")
    print("   • O/L motor kontrol - debug loglu") 
    print("   • V vibration menü - debug loglu")
    print("   • X servo sıfırlama - anlık reset")
    print("   • Otomatik servo sıfırlama aktif")
    
    print("\n🚀 TESİ İÇİN:")
    print("   python3 terminal_gui.py")
    print("   \n   Tuş bastıktan sonra log mesajlarını kontrol et!")

if __name__ == "__main__":
    test_keyboard_mappings() 