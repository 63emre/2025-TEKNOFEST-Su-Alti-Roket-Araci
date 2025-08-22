"""
TEKNOFEST 2025 Su Altı Roket Aracı
PID Controller Modülü

Bu modül roll, pitch, yaw, derinlik ve hız kontrolü için PID kontrolcülerini sağlar.
"""

import time
import logging
import threading
import math
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from collections import deque

@dataclass
class PIDState:
    """PID kontrolcü durumu"""
    kp: float
    ki: float  
    kd: float
    max_output: float
    integral_limit: float
    setpoint: float
    
    # İç durumlar
    previous_error: float = 0.0
    integral: float = 0.0
    last_time: float = 0.0
    output: float = 0.0

class PIDController:
    """Tek eksenli PID kontrolcü sınıfı"""
    
    def __init__(self, kp: float, ki: float, kd: float, max_output: float, 
                 integral_limit: float, setpoint: float = 0.0):
        """
        PID kontrolcüyü başlat
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_output: Maksimum çıkış değeri
            integral_limit: Integral windup limiti
            setpoint: Hedef değer
        """
        self.state = PIDState(kp, ki, kd, max_output, integral_limit, setpoint)
        self.logger = logging.getLogger(__name__)
        
        # Geçmiş hata değerleri (filtreleme için)
        self.error_history = deque(maxlen=5)
        
        # İstatistikler
        self.stats = {
            "update_count": 0,
            "max_error": 0.0,
            "avg_error": 0.0,
            "total_error": 0.0
        }
    
    def update(self, current_value: float, dt: Optional[float] = None) -> float:
        """
        PID çıkışını hesapla
        
        Args:
            current_value: Güncel değer
            dt: Delta time (saniye), None ise otomatik hesaplama
            
        Returns:
            float: PID kontrolcü çıkışı
        """
        current_time = time.time()
        
        # Delta time hesapla
        if dt is None:
            if self.state.last_time == 0:
                dt = 0.02  # İlk çalıştırmada varsayılan 20ms
            else:
                dt = current_time - self.state.last_time
        
        self.state.last_time = current_time
        
        # Hata hesapla
        error = self.state.setpoint - current_value
        
        # Hata geçmişini güncelle
        self.error_history.append(error)
        
        # P term
        p_term = self.state.kp * error
        
        # I term (integral windup'a dikkat et)
        self.state.integral += error * dt
        
        # Integral limitleri uygula
        if self.state.integral_limit > 0:
            self.state.integral = max(-self.state.integral_limit,
                                    min(self.state.integral_limit,
                                        self.state.integral))
        
        i_term = self.state.ki * self.state.integral
        
        # D term (filtrelenmiş hata değişimi kullan)
        if dt > 0:
            error_derivative = (error - self.state.previous_error) / dt
        else:
            error_derivative = 0.0
            
        d_term = self.state.kd * error_derivative
        
        # Toplam çıkış
        raw_output = p_term + i_term + d_term
        
        # Çıkış limitleri uygula
        self.state.output = max(-self.state.max_output,
                               min(self.state.max_output, raw_output))
        
        # Geçmiş hatayı güncelle
        self.state.previous_error = error
        
        # İstatistikleri güncelle
        self._update_stats(error)
        
        self.logger.debug(f"PID Update - Error: {error:.3f}, "
                         f"P: {p_term:.2f}, I: {i_term:.2f}, D: {d_term:.2f}, "
                         f"Output: {self.state.output:.2f}")
        
        return self.state.output
    
    def set_setpoint(self, setpoint: float):
        """Hedef değeri ayarla"""
        self.state.setpoint = setpoint
        self.logger.info(f"Setpoint değiştirildi: {setpoint}")
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """PID gainlerini ayarla"""
        self.state.kp = kp
        self.state.ki = ki
        self.state.kd = kd
        self.logger.info(f"PID gains güncellendi - Kp:{kp}, Ki:{ki}, Kd:{kd}")
    
    def reset(self):
        """PID durumunu sıfırla"""
        self.state.integral = 0.0
        self.state.previous_error = 0.0
        self.state.last_time = 0.0
        self.state.output = 0.0
        self.error_history.clear()
        
        # İstatistikleri sıfırla
        self.stats = {
            "update_count": 0,
            "max_error": 0.0,
            "avg_error": 0.0,
            "total_error": 0.0
        }
        
        self.logger.info("PID controller sıfırlandı")
    
    def _update_stats(self, error: float):
        """İstatistikleri güncelle"""
        self.stats["update_count"] += 1
        self.stats["total_error"] += abs(error)
        self.stats["avg_error"] = self.stats["total_error"] / self.stats["update_count"]
        self.stats["max_error"] = max(self.stats["max_error"], abs(error))
    
    def get_stats(self) -> Dict:
        """PID istatistiklerini al"""
        return {
            **self.stats,
            "setpoint": self.state.setpoint,
            "last_output": self.state.output,
            "integral": self.state.integral,
            "kp": self.state.kp,
            "ki": self.state.ki,
            "kd": self.state.kd
        }

class MultiAxisPIDController:
    """Çok eksenli PID kontrolcü sistemi"""
    
    def __init__(self, pid_configs: Dict[str, Dict]):
        """
        Çok eksenli PID kontrolcüyü başlat
        
        Args:
            pid_configs: Her eksen için PID konfigürasyonu
                        {"roll": {"kp": 1.0, "ki": 0.1, ...}, ...}
        """
        self.controllers = {}
        self.logger = logging.getLogger(__name__)
        
        # Her eksen için PID kontrolcü oluştur
        for axis_name, config in pid_configs.items():
            self.controllers[axis_name] = PIDController(**config)
            
        self.logger.info(f"Multi-axis PID Controller oluşturuldu: {list(self.controllers.keys())}")
    
    def update_all(self, current_values: Dict[str, float], 
                   dt: Optional[float] = None) -> Dict[str, float]:
        """
        Tüm eksenlerin PID çıkışını hesapla
        
        Args:
            current_values: Her eksen için güncel değerler
            dt: Delta time
            
        Returns:
            Dict[str, float]: Her eksen için PID çıkışları
        """
        outputs = {}
        
        for axis_name, controller in self.controllers.items():
            if axis_name in current_values:
                outputs[axis_name] = controller.update(current_values[axis_name], dt)
            else:
                self.logger.warning(f"{axis_name} için güncel değer bulunamadı")
                outputs[axis_name] = 0.0
                
        return outputs
    
    def set_setpoints(self, setpoints: Dict[str, float]):
        """Tüm eksenler için hedef değerleri ayarla"""
        for axis_name, setpoint in setpoints.items():
            if axis_name in self.controllers:
                self.controllers[axis_name].set_setpoint(setpoint)
    
    def reset_all(self):
        """Tüm PID kontrolcüleri sıfırla"""
        for controller in self.controllers.values():
            controller.reset()
        self.logger.info("Tüm PID kontrolcüleri sıfırlandı")
    
    def get_all_stats(self) -> Dict[str, Dict]:
        """Tüm eksenlerin istatistiklerini al"""
        return {axis_name: controller.get_stats() 
                for axis_name, controller in self.controllers.items()}
    
    def update_gains(self, axis_name: str, kp: float, ki: float, kd: float):
        """Belirli eksen için PID gainlerini güncelle"""
        if axis_name in self.controllers:
            self.controllers[axis_name].set_gains(kp, ki, kd)
        else:
            self.logger.error(f"Bilinmeyen eksen: {axis_name}")

class SubmarineStabilizer:
    """Su altı aracı stabilizasyon sistemi"""
    
    def __init__(self, pid_configs: Dict, mixing_matrix: Dict, 
                 fin_effectiveness: Dict):
        """
        Stabilizasyon sistemini başlat
        
        Args:
            pid_configs: PID konfigürasyonları
            mixing_matrix: Fin karıştırma matrisi
            fin_effectiveness: Fin etkinlik katsayıları
        """
        self.pid_controller = MultiAxisPIDController(pid_configs)
        self.mixing_matrix = mixing_matrix
        self.fin_effectiveness = fin_effectiveness
        
        self.enabled = False
        self.mode = 0  # Stabilizasyon modu
        
        # Thread kontrolü
        self._stop_control = threading.Event()
        self._control_thread = None
        self.control_frequency = pid_configs.get("roll", {}).get("update_frequency", 50)
        
        self.logger = logging.getLogger(__name__)
        
        # Callback fonksiyonları
        self.servo_output_callback = None
        self.motor_output_callback = None
        
    def set_callbacks(self, servo_callback, motor_callback):
        """Servo ve motor kontrol callback'lerini ayarla"""
        self.servo_output_callback = servo_callback
        self.motor_output_callback = motor_callback
    
    def enable_stabilization(self, mode: int = 1):
        """Stabilizasyonu etkinleştir"""
        self.enabled = True
        self.mode = mode
        self.logger.info(f"Stabilizasyon etkinleştirildi - Mod: {mode}")
    
    def disable_stabilization(self):
        """Stabilizasyonu devre dışı bırak"""
        self.enabled = False
        self.logger.info("Stabilizasyon devre dışı bırakıldı")
    
    def update_stabilization(self, sensor_data: Dict[str, float]) -> Dict[str, float]:
        """
        Stabilizasyon güncellemesi yap
        
        Args:
            sensor_data: Sensör verileri (roll, pitch, yaw, depth, etc.)
            
        Returns:
            Dict[str, float]: Fin PWM çıkışları
        """
        if not self.enabled:
            return self._get_neutral_outputs()
        
        # PID çıkışlarını hesapla
        pid_outputs = self.pid_controller.update_all(sensor_data)
        
        # Fin mixing uygula
        fin_outputs = self._apply_mixing(pid_outputs)
        
        # Servo callback'i çağır
        if self.servo_output_callback:
            self.servo_output_callback(fin_outputs)
        
        return fin_outputs
    
    def _apply_mixing(self, pid_outputs: Dict[str, float]) -> Dict[str, float]:
        """Fin mixing matrisini uygula"""
        fin_outputs = {}
        
        # Her fin için çıkışları hesapla
        for fin_name in self.fin_effectiveness.keys():
            total_output = 1500  # Neutral PWM
            
            # Her PID çıkışı için bu fin'in katkısını hesapla
            for axis, pid_output in pid_outputs.items():
                if axis in self.mixing_matrix:
                    if fin_name in self.mixing_matrix[axis]:
                        contribution = (pid_output * 
                                      self.mixing_matrix[axis][fin_name] * 
                                      self.fin_effectiveness[fin_name])
                        total_output += contribution
            
            # PWM limitlerini uygula (1000-2000)
            fin_outputs[fin_name] = max(1000, min(2000, int(total_output)))
        
        return fin_outputs
    
    def _get_neutral_outputs(self) -> Dict[str, float]:
        """Nötr fin çıkışlarını döndür"""
        return {fin_name: 1500 for fin_name in self.fin_effectiveness.keys()}
    
    def auto_tune_pid(self, axis: str, test_duration: float = 30.0):
        """
        Otomatik PID ayarlama (Ziegler-Nichols benzeri)
        
        Args:
            axis: Ayarlanacak eksen
            test_duration: Test süresi (saniye)
        """
        self.logger.info(f"{axis} ekseni için otomatik PID ayarlama başlıyor...")
        
        if axis not in self.pid_controller.controllers:
            self.logger.error(f"Bilinmeyen eksen: {axis}")
            return
        
        # Mevcut PID değerlerini kaydet
        controller = self.pid_controller.controllers[axis]
        original_gains = (controller.state.kp, controller.state.ki, controller.state.kd)
        
        try:
            # İlk olarak sadece P gain ile test yap
            controller.set_gains(1.0, 0.0, 0.0)
            controller.reset()
            
            # Test verilerini topla
            test_data = []
            start_time = time.time()
            
            while time.time() - start_time < test_duration:
                # Bu kısım gerçek implementasyonda sensör verisiyle doldurulacak
                # Şimdilik placeholder
                time.sleep(0.02)  # 50Hz
            
            # Toplanan verilerle optimal PID değerlerini hesapla
            # Bu kısım gelişmiş algoritma gerektirir
            
            self.logger.info(f"{axis} PID auto-tune tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Auto-tune hatası: {e}")
            # Orijinal değerleri geri yükle
            controller.set_gains(*original_gains)
    
    def get_system_status(self) -> Dict:
        """Sistem durumunu al"""
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "pid_stats": self.pid_controller.get_all_stats(),
            "control_frequency": self.control_frequency
        }

# Test ve kalibrasyon fonksiyonları
def test_pid_response(pid_config: Dict, test_signal: list, 
                      test_name: str = "PID Test"):
    """PID yanıtını test et"""
    controller = PIDController(**pid_config)
    
    print(f"\n{test_name}")
    print("-" * 40)
    
    responses = []
    for i, target_value in enumerate(test_signal):
        controller.set_setpoint(target_value)
        
        # Simüle edilmiş sistem yanıtı
        current_value = target_value * 0.8 + (responses[-1] if responses else 0) * 0.2
        
        output = controller.update(current_value)
        responses.append(output)
        
        print(f"Step {i+1:2d}: Target={target_value:6.2f}, "
              f"Current={current_value:6.2f}, Output={output:6.2f}")
    
    return responses

if __name__ == "__main__":
    # Test PID konfigürasyonu
    test_config = {
        "kp": 2.0,
        "ki": 0.1,
        "kd": 0.5,
        "max_output": 500,
        "integral_limit": 100,
        "setpoint": 0.0
    }
    
    # Step yanıt testi
    step_signal = [0, 10, 10, 10, 0, 0, -5, -5, 0]
    test_pid_response(test_config, step_signal, "Step Response Test")
