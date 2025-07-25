#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Control Module
Tkinter Real-time Kontrol Sistemi
"""

import tkinter as tk
from tkinter import messagebox, simpledialog
import threading
import time
import json

class ControlModule:
    def __init__(self, mavlink_handler, navigation_engine, main_gui_callback=None):
        """Real-time kontrol modülü"""
        self.mavlink = mavlink_handler
        self.navigation = navigation_engine
        self.gui_callback = main_gui_callback
        
        # Tkinter window
        self.control_window = None
        self.window_active = False
        
        # Keyboard state
        self.active_keys = set()
        self.key_bindings = {}
        self.setup_key_bindings()
        
        # Real-time control
        self.realtime_active = False
        self.control_thread = None
        self.control_lock = threading.Lock()
        
        # Control values
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_depth = 0.0
        self.current_motor = 0.0
        
        # Settings
        self.control_sensitivity = 1.0
        self.max_control_value = 45.0
        
    def setup_key_bindings(self):
        """Klavye bağlamalarını kur"""
        # Real-time controls (basılı tutma)
        self.key_bindings = {
            # Servo controls
            'w': {'type': 'realtime', 'action': 'pitch_up'},
            's': {'type': 'realtime', 'action': 'pitch_down'},
            'a': {'type': 'realtime', 'action': 'roll_left'},
            'd': {'type': 'realtime', 'action': 'roll_right'},
            'q': {'type': 'realtime', 'action': 'yaw_left'},
            'e': {'type': 'realtime', 'action': 'yaw_right'},
            
            # Depth control
            'Prior': {'type': 'realtime', 'action': 'depth_up'},     # Page Up
            'Next': {'type': 'realtime', 'action': 'depth_down'},   # Page Down
            
            # Emergency
            'space': {'type': 'instant', 'action': 'emergency_stop'},
            'Escape': {'type': 'instant', 'action': 'surface_immediately'},
            
            # Movement commands (parameter input)
            't': {'type': 'command', 'action': 'forward_command'},
            'y': {'type': 'command', 'action': 'yaw_command'},
            'u': {'type': 'command', 'action': 'ascend_command'},
            'g': {'type': 'command', 'action': 'strafe_left_command'},
            'h': {'type': 'command', 'action': 'strafe_right_command'},
            
            # System controls
            'F1': {'type': 'instant', 'action': 'toggle_control_mode'},
            'F2': {'type': 'instant', 'action': 'toggle_navigation_mode'},
            'F12': {'type': 'instant', 'action': 'system_status'}
        }
    
    def create_control_window(self):
        """Kontrol penceresi oluştur"""
        if self.control_window:
            return
        
        self.control_window = tk.Toplevel()
        self.control_window.title("TEKNOFEST ROV - Real-time Control")
        self.control_window.geometry("600x400")
        self.control_window.protocol("WM_DELETE_WINDOW", self.close_control_window)
        
        # Window'u her zaman üstte tut
        self.control_window.attributes("-topmost", True)
        
        # Focus'u al
        self.control_window.focus_set()
        
        # Keyboard events
        self.control_window.bind('<KeyPress>', self.on_key_press)
        self.control_window.bind('<KeyRelease>', self.on_key_release) 
        self.control_window.bind('<FocusIn>', self.on_focus_in)
        self.control_window.bind('<FocusOut>', self.on_focus_out)
        
        # GUI elements
        self.setup_control_gui()
        
        self.window_active = True
        print("🎮 Real-time control window açıldı")
    
    def setup_control_gui(self):
        """Kontrol GUI elemanları"""
        # Main frame
        main_frame = tk.Frame(self.control_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title = tk.Label(main_frame, text="TEKNOFEST ROV KONTROL", 
                        font=("Arial", 16, "bold"))
        title.pack(pady=5)
        
        # Status frame
        status_frame = tk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)
        
        # Connection status
        self.connection_label = tk.Label(status_frame, text="MAVLink: ❌", 
                                       fg="red", font=("Arial", 10))
        self.connection_label.pack(side=tk.LEFT)
        
        # Armed status
        self.armed_label = tk.Label(status_frame, text="DISARMED", 
                                  fg="red", font=("Arial", 10, "bold"))
        self.armed_label.pack(side=tk.RIGHT)
        
        # Control values frame
        values_frame = tk.Frame(main_frame)
        values_frame.pack(fill=tk.X, pady=10)
        
        # Current control values
        tk.Label(values_frame, text="Mevcut Kontrol Değerleri:", 
                font=("Arial", 12, "bold")).pack()
        
        self.values_text = tk.Text(values_frame, height=4, width=50)
        self.values_text.pack(pady=5)
        
        # Controls help frame
        help_frame = tk.Frame(main_frame)
        help_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        tk.Label(help_frame, text="Kontrol Tuşları:", 
                font=("Arial", 12, "bold")).pack()
        
        help_text = """
REAL-TIME KONTROLLER (Basılı Tut):
  W,A,S,D  → Servo kontrolü (Pitch/Roll)
  Q,E      → Yaw (sola/sağa dönme)
  Page↑/↓  → Derinlik kontrolü
  
HAREKET KOMUTLARI (Parametre Gir):
  T → İleri git (metre gir)
  Y → Yaw dönme (derece gir)  
  U → Yukarı çık (metre gir)
  G → Sol git (metre gir)
  H → Sağ git (metre gir)
  
ACİL KONTROLLER:
  Space → Acil durum!
  Esc   → Yüzeye çık!
  
SİSTEM:
  F1  → Kontrol modu değiştir
  F2  → Navigation modu değiştir
  F12 → Sistem durumu
        """
        
        help_display = tk.Text(help_frame, height=18, width=70)
        help_display.insert(tk.END, help_text)
        help_display.config(state=tk.DISABLED)
        help_display.pack(fill=tk.BOTH, expand=True)
        
        # Control sensitivity
        sens_frame = tk.Frame(main_frame)
        sens_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(sens_frame, text="Hassasiyet:").pack(side=tk.LEFT)
        self.sensitivity_scale = tk.Scale(sens_frame, from_=0.1, to=2.0, 
                                         resolution=0.1, orient=tk.HORIZONTAL,
                                         command=self.on_sensitivity_change)
        self.sensitivity_scale.set(1.0)
        self.sensitivity_scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Start monitoring
        self.start_status_monitoring()
    
    def close_control_window(self):
        """Kontrol penceresini kapat"""
        self.window_active = False
        self.stop_realtime_control()
        
        if self.control_window:
            self.control_window.destroy()
            self.control_window = None
        
        print("🎮 Real-time control window kapatıldı")
    
    def on_focus_in(self, event):
        """Window focus aldığında"""
        print("🎮 Control window focus aldı")
    
    def on_focus_out(self, event):
        """Window focus kaybettiğinde"""
        # Tüm tuşları bırak
        self.active_keys.clear()
        self.stop_realtime_control()
        print("🎮 Control window focus kaybetti - kontroller temizlendi")
    
    def on_key_press(self, event):
        """Tuş basıldığında"""
        key = event.keysym.lower()
        
        if key not in self.key_bindings:
            return
        
        binding = self.key_bindings[key]
        
        if binding['type'] == 'realtime':
            # Real-time kontrol (basılı tutma)
            if key not in self.active_keys:
                self.active_keys.add(key)
                self.start_realtime_control()
                
        elif binding['type'] == 'instant':
            # Anlık komut
            self.execute_instant_command(binding['action'])
            
        elif binding['type'] == 'command':
            # Parametreli komut
            self.execute_parameter_command(binding['action'])
    
    def on_key_release(self, event):
        """Tuş bırakıldığında"""
        key = event.keysym.lower()
        
        if key in self.active_keys:
            self.active_keys.remove(key)
            
        # Hiç tuş kalmadıysa real-time kontrolü durdur
        if not self.active_keys:
            self.stop_realtime_control()
    
    def on_sensitivity_change(self, value):
        """Hassasiyet değiştiğinde"""
        self.control_sensitivity = float(value)
        print(f"🎛️ Kontrol hassasiyeti: {self.control_sensitivity:.1f}")
    
    def start_realtime_control(self):
        """Real-time kontrolü başlat"""
        if self.realtime_active:
            return
        
        with self.control_lock:
            self.realtime_active = True
            
        if self.control_thread is None or not self.control_thread.is_alive():
            self.control_thread = threading.Thread(target=self._realtime_control_loop, daemon=True)
            self.control_thread.start()
    
    def stop_realtime_control(self):
        """Real-time kontrolü durdur"""
        with self.control_lock:
            self.realtime_active = False
            
        # Kontrol değerlerini sıfırla
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Neutral komut gönder
        if self.mavlink.connected and self.mavlink.armed:
            if self.mavlink.control_mode == "raw":
                self.mavlink.control_servos_raw(0, 0, 0)
            else:
                self.mavlink.control_servos_pid(0, 0, 0)
    
    def _realtime_control_loop(self):
        """Real-time kontrol döngüsü"""
        control_rate = 50  # Hz
        dt = 1.0 / control_rate
        
        while self.realtime_active:
            start_time = time.time()
            
            # Aktif tuşları kontrol değerlerine çevir
            self.calculate_control_values()
            
            # Servo komutlarını gönder
            if self.mavlink.connected and self.mavlink.armed:
                if self.mavlink.control_mode == "raw":
                    self.mavlink.control_servos_raw(
                        self.current_roll, self.current_pitch, self.current_yaw
                    )
                else:
                    self.mavlink.control_servos_pid(
                        self.current_roll, self.current_pitch, self.current_yaw
                    )
            
            # Timing
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def calculate_control_values(self):
        """Basılı tuşlardan kontrol değerlerini hesapla"""
        # Reset values
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        
        # Her aktif tuş için değer ekle
        for key in self.active_keys:
            if key not in self.key_bindings:
                continue
                
            action = self.key_bindings[key]['action']
            control_value = self.max_control_value * self.control_sensitivity
            
            if action == 'pitch_up':
                pitch += control_value
            elif action == 'pitch_down':
                pitch -= control_value
            elif action == 'roll_left':
                roll -= control_value
            elif action == 'roll_right':
                roll += control_value
            elif action == 'yaw_left':
                yaw -= control_value
            elif action == 'yaw_right':
                yaw += control_value
            elif action == 'depth_up':
                self.current_depth += control_value
            elif action == 'depth_down':
                self.current_depth -= control_value
        
        # Limitleri uygula
        self.current_roll = max(-self.max_control_value, min(self.max_control_value, roll))
        self.current_pitch = max(-self.max_control_value, min(self.max_control_value, pitch))
        self.current_yaw = max(-self.max_control_value, min(self.max_control_value, yaw))
    
    def execute_instant_command(self, action):
        """Anlık komutları çalıştır"""
        print(f"⚡ Anlık komut: {action}")
        
        if action == 'emergency_stop':
            self.mavlink.emergency_stop()
            messagebox.showwarning("ACİL DURUM", "Tüm kontroller durduruldu!")
            
        elif action == 'surface_immediately':
            messagebox.showwarning("YÜZEYE ÇIK", "Acil yüzey çıkış protokolü!")
            # TODO: Emergency surface protocol
            
        elif action == 'toggle_control_mode':
            current_mode = self.mavlink.control_mode
            new_mode = "pid" if current_mode == "raw" else "raw"
            self.mavlink.set_control_mode(new_mode)
            messagebox.showinfo("Kontrol Modu", f"Kontrol modu: {new_mode.upper()}")
            
        elif action == 'toggle_navigation_mode':
            current_nav = self.navigation.current_mode
            nav_modes = ["gps_only", "imu_only", "hybrid"]
            current_idx = nav_modes.index(current_nav) if current_nav in nav_modes else 0
            new_idx = (current_idx + 1) % len(nav_modes)
            new_nav = nav_modes[new_idx]
            
            self.navigation.set_navigation_mode(new_nav)
            messagebox.showinfo("Navigation Modu", f"Navigation: {new_nav}")
            
        elif action == 'system_status':
            self.show_system_status()
    
    def execute_parameter_command(self, action):
        """Parametreli komutları çalıştır"""
        print(f"📝 Parametreli komut: {action}")
        
        try:
            if action == 'forward_command':
                distance = simpledialog.askfloat("İleri Git", 
                                                "Mesafe (metre):", 
                                                initialvalue=5.0, 
                                                minvalue=0.1, 
                                                maxvalue=50.0)
                if distance:
                    self.navigation.start_movement_mission('FORWARD', distance, self.mavlink.control_mode)
                    
            elif action == 'yaw_command':
                angle = simpledialog.askfloat("Yaw Dönme", 
                                            "Açı (derece):", 
                                            initialvalue=90.0, 
                                            minvalue=-180.0, 
                                            maxvalue=180.0)
                if angle:
                    self.navigation.start_movement_mission('YAW_ROTATION', angle, self.mavlink.control_mode)
                    
            elif action == 'ascend_command':
                distance = simpledialog.askfloat("Yukarı Çık", 
                                                "Mesafe (metre):", 
                                                initialvalue=2.0, 
                                                minvalue=0.1, 
                                                maxvalue=10.0)
                if distance:
                    self.navigation.start_movement_mission('ASCEND', distance, self.mavlink.control_mode)
                    
            elif action == 'strafe_left_command':
                distance = simpledialog.askfloat("Sol Git", 
                                                "Mesafe (metre):", 
                                                initialvalue=3.0, 
                                                minvalue=0.1, 
                                                maxvalue=20.0)
                if distance:
                    self.navigation.start_movement_mission('STRAFE_LEFT', distance, self.mavlink.control_mode)
                    
            elif action == 'strafe_right_command':
                distance = simpledialog.askfloat("Sağ Git", 
                                                "Mesafe (metre):", 
                                                initialvalue=3.0, 
                                                minvalue=0.1, 
                                                maxvalue=20.0)
                if distance:
                    self.navigation.start_movement_mission('STRAFE_RIGHT', distance, self.mavlink.control_mode)
                    
        except Exception as e:
            messagebox.showerror("Komut Hatası", f"Komut çalıştırılamadı: {e}")
    
    def show_system_status(self):
        """Sistem durum penceresini göster"""
        status_window = tk.Toplevel(self.control_window)
        status_window.title("Sistem Durumu")
        status_window.geometry("400x300")
        
        # Status text
        status_text = tk.Text(status_window, wrap=tk.WORD)
        status_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Sistem bilgilerini topla
        status_info = f"""
TEKNOFEST ROV SİSTEM DURUMU
================================

MAVLink Bağlantı: {'✅ Bağlı' if self.mavlink.connected else '❌ Bağlantısız'}
Sistem Durumu: {'🔒 ARMED' if self.mavlink.armed else '🔓 DISARMED'}

Kontrol Modu: {self.mavlink.control_mode.upper()}
Navigation Modu: {self.navigation.current_mode}

Aktif Görev: {'✅ Var' if self.navigation.mission_active else '❌ Yok'}

Mevcut Kontrol Değerleri:
  Roll:  {self.current_roll:+6.1f}°
  Pitch: {self.current_pitch:+6.1f}°
  Yaw:   {self.current_yaw:+6.1f}°

Hassasiyet: {self.control_sensitivity:.1f}x
Aktif Tuşlar: {', '.join(self.active_keys) if self.active_keys else 'Yok'}

Real-time Kontrol: {'🟢 Aktif' if self.realtime_active else '🔴 Pasif'}
        """
        
        status_text.insert(tk.END, status_info)
        status_text.config(state=tk.DISABLED)
    
    def start_status_monitoring(self):
        """Durum monitörünü başlat"""
        def update_status():
            if not self.window_active:
                return
            
            # Connection status
            if self.mavlink.connected:
                self.connection_label.config(text="MAVLink: ✅", fg="green")
            else:
                self.connection_label.config(text="MAVLink: ❌", fg="red")
            
            # Armed status
            if self.mavlink.armed:
                self.armed_label.config(text="ARMED", fg="green")
            else:
                self.armed_label.config(text="DISARMED", fg="red")
            
            # Control values
            values_info = f"""Roll: {self.current_roll:+6.1f}°   Pitch: {self.current_pitch:+6.1f}°   Yaw: {self.current_yaw:+6.1f}°
Aktif Tuşlar: {', '.join(self.active_keys) if self.active_keys else 'Yok'}
Kontrol Modu: {self.mavlink.control_mode.upper()}   Navigation: {self.navigation.current_mode}
Real-time: {'🟢 Aktif' if self.realtime_active else '🔴 Pasif'}"""
            
            self.values_text.delete(1.0, tk.END)
            self.values_text.insert(tk.END, values_info)
            
            # 100ms sonra tekrar güncelle
            if self.window_active:
                self.control_window.after(100, update_status)
        
        # İlk güncelleme
        if self.window_active:
            self.control_window.after(100, update_status)
    
    def show_control_window(self):
        """Kontrol penceresini göster"""
        if not self.control_window:
            self.create_control_window()
        else:
            self.control_window.deiconify()
            self.control_window.focus_set()
    
    def hide_control_window(self):
        """Kontrol penceresini gizle"""
        if self.control_window:
            self.control_window.withdraw()

if __name__ == "__main__":
    # Test
    import sys
    sys.path.append('.')
    
    from mavlink_handler import MAVLinkHandler
    from navigation_engine import NavigationEngine
    
    # Test root window
    root = tk.Tk()
    root.title("Control Module Test")
    
    # MAVLink handler
    mavlink = MAVLinkHandler()
    
    # Navigation engine  
    navigation = NavigationEngine(mavlink)
    
    # Control module
    control = ControlModule(mavlink, navigation)
    
    # Test button
    def show_control():
        control.show_control_window()
    
    test_btn = tk.Button(root, text="Kontrol Penceresini Aç", command=show_control)
    test_btn.pack(pady=20)
    
    root.mainloop() 