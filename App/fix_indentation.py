#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Indentasyon Hatası Düzeltme Scripti
main_gui.py'deki line 1120-1122 indentasyon hatasını düzeltir
"""

import re

def fix_indentation():
    """main_gui.py'deki indentasyon hatasını düzelt"""
    file_path = "main_gui.py"
    
    try:
        # Dosyayı oku
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # Hatayı bul ve düzelt
        fixed_lines = []
        in_reset_method = False
        
        for i, line in enumerate(lines):
            line_num = i + 1
            
            # reset_to_defaults metodundayız
            if 'def reset_to_defaults(self):' in line:
                in_reset_method = True
                fixed_lines.append(line)
                continue
            
            # Metoddan çıktık mı kontrol et
            if in_reset_method and line.strip() and not line.startswith('    ') and not line.startswith('#'):
                if 'class ' in line or 'def ' in line:
                    in_reset_method = False
            
            # Problematik satırları düzelt
            if line_num >= 1120 and line_num <= 1122:
                # MAVLink defaults kısmı
                if 'self.connection_string.setText' in line or 'self.heartbeat_timeout.setValue' in line or 'self.command_timeout.setValue' in line:
                    # Indentasyonu düzelt
                    if not line.startswith('            '):
                        fixed_line = '            ' + line.lstrip()
                        fixed_lines.append(fixed_line)
                        print(f"✅ Line {line_num} düzeltildi: {line.strip()} -> doğru indentasyon")
                        continue
            
            # Line 1115 civarındaki fazla indentasyon
            if 'self.pin_widgets["raspberry_pi_gpio_warning_led"]' in line:
                if line.startswith('                        '):  # Fazla indentasyon
                    fixed_line = '            ' + line.lstrip()
                    fixed_lines.append(fixed_line)
                    print(f"✅ Line {line_num} düzeltildi: Fazla indentasyon kaldırıldı")
                    continue
            
            # Normal satırları ekle
            fixed_lines.append(line)
        
        # Dosyayı yaz
        with open(file_path, 'w', encoding='utf-8') as f:
            f.writelines(fixed_lines)
        
        print(f"🎉 {file_path} başarıyla düzeltildi!")
        return True
        
    except FileNotFoundError:
        print(f"❌ Hata: {file_path} dosyası bulunamadı!")
        return False
    except Exception as e:
        print(f"❌ Hata: {e}")
        return False

def validate_fix():
    """Düzeltmenin başarılı olduğunu doğrula"""
    file_path = "main_gui.py"
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Syntax hatası kontrolü
        try:
            compile(content, file_path, 'exec')
            print("✅ Syntax kontrolü: BAŞARILI")
            return True
        except SyntaxError as e:
            print(f"❌ Hala syntax hatası var: {e}")
            return False
            
    except Exception as e:
        print(f"❌ Doğrulama hatası: {e}")
        return False

if __name__ == "__main__":
    print("🔧 TEKNOFEST ROV - Indentasyon Düzeltme Scripti")
    print("=" * 50)
    
    # Düzeltmeyi yap
    if fix_indentation():
        # Doğrulamayı yap
        if validate_fix():
            print("\n🎉 Tüm düzeltmeler başarılı!")
            print("Artık main_gui.py'yi çalıştırabilirsiniz:")
            print("python3 main_gui.py")
        else:
            print("\n⚠️ Düzeltme yapıldı ama hala hata var.")
            print("Manuel kontrol gerekebilir.")
    else:
        print("\n❌ Düzeltme başarısız!")
        
    print("\n" + "=" * 50) 