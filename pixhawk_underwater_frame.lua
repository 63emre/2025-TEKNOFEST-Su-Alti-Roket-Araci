-- TEKNOFEST 2025 Su Altı Roket Aracı - Custom Lua Script
-- Pixhawk PX4 için özel underwater vehicle frame ve navigation
-- Yazar: TEKNOFEST Takım
-- Versiyon: 1.0

-- ============================================================================
-- KONFIGÜRASYON PARAMETRELERI
-- ============================================================================

local MAV_FRAME_CUSTOM_UNDERWATER = 37  -- Custom underwater frame ID
local UNDERWATER_MODE_DEPTH_HOLD = 100  -- Custom depth hold mode
local UNDERWATER_MODE_WAYPOINT_NAV = 101 -- Custom waypoint navigation
local UNDERWATER_MODE_EMERGENCY_SURFACE = 102 -- Emergency surface mode
local UNDERWATER_MODE_ROCKET_LAUNCH = 103 -- Rocket launch preparation mode

-- PID Parametreleri (optimize edilmiş)
local DEPTH_PID = {kp = 150.0, ki = 10.0, kd = 45.0, max_output = 500}
local HEADING_PID = {kp = 4.0, ki = 0.3, kd = 1.2, max_output = 400}
local POSITION_PID = {kp = 2.5, ki = 0.1, kd = 0.8, max_output = 300}

-- Güvenlik limitleri
local MAX_DEPTH = 10.0  -- Maximum operating depth (meters)
local MIN_BATTERY_VOLTAGE = 19.8  -- Minimum battery voltage (6S = 3.3V/cell)
local EMERGENCY_SURFACE_TIMEOUT = 300  -- 5 dakika emergency timeout

-- Servo/Motor kanalları (HARDWARE_PIN_MAPPING.md'ye uygun)
local MOTOR_MAIN = 1
local SERVO_FIN_1 = 1  -- Front Left
local SERVO_FIN_2 = 2  -- Front Right
local SERVO_FIN_3 = 3  -- Rear Left  
local SERVO_FIN_4 = 4  -- Rear Right
local SERVO_ELEVATOR = 5
local SERVO_PAYLOAD_BAY = 6
local SERVO_SEPARATION = 7

-- ============================================================================
-- GLOBAL VARIABLES
-- ============================================================================

local underwater_mode = 0
local target_depth = 0.0
local current_depth = 0.0
local surface_pressure = 1013.25
local depth_hold_active = false
local emergency_triggered = false
local mission_start_time = 0
local last_heartbeat_time = 0

-- PID Controllers state
local depth_pid_integral = 0.0
local depth_pid_last_error = 0.0
local depth_pid_last_time = 0

local heading_pid_integral = 0.0
local heading_pid_last_error = 0.0
local heading_pid_last_time = 0

-- Navigation waypoints
local current_waypoint_index = 0
local waypoints = {}
local waypoint_tolerance = 2.0  -- meters

-- ============================================================================
-- UTILITY FUNCTIONS
-- ============================================================================

function get_current_time_ms()
    return millis()
end

function constrain_float(value, min_val, max_val)
    if value < min_val then
        return min_val
    elseif value > max_val then
        return max_val
    else
        return value
    end
end

function wrap_360(angle)
    angle = angle % 360
    if angle < 0 then
        angle = angle + 360
    end
    return angle
end

function calculate_distance(lat1, lon1, lat2, lon2)
    local R = 6371000  -- Earth radius in meters
    local lat1_rad = math.rad(lat1)
    local lat2_rad = math.rad(lat2)
    local dlat_rad = math.rad(lat2 - lat1)
    local dlon_rad = math.rad(lon2 - lon1)
    
    local a = math.sin(dlat_rad/2)^2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad/2)^2
    local c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c
end

function calculate_bearing(lat1, lon1, lat2, lon2)
    local lat1_rad = math.rad(lat1)
    local lat2_rad = math.rad(lat2)
    local dlon_rad = math.rad(lon2 - lon1)
    
    local y = math.sin(dlon_rad) * math.cos(lat2_rad)
    local x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    
    local bearing = math.deg(math.atan2(y, x))
    return wrap_360(bearing)
end

-- ============================================================================
-- PID CONTROLLER FUNCTIONS
-- ============================================================================

function update_depth_pid(setpoint, measurement)
    local current_time = get_current_time_ms()
    local dt = (current_time - depth_pid_last_time) / 1000.0  -- Convert to seconds
    
    if dt <= 0.0 then
        dt = 0.01
    end
    
    local error = setpoint - measurement
    
    -- Proportional
    local proportional = DEPTH_PID.kp * error
    
    -- Integral with windup protection
    depth_pid_integral = depth_pid_integral + error * dt
    local integral_max = DEPTH_PID.max_output / DEPTH_PID.ki
    depth_pid_integral = constrain_float(depth_pid_integral, -integral_max, integral_max)
    local integral = DEPTH_PID.ki * depth_pid_integral
    
    -- Derivative
    local derivative = DEPTH_PID.kd * (error - depth_pid_last_error) / dt
    
    -- Output
    local output = proportional + integral + derivative
    output = constrain_float(output, -DEPTH_PID.max_output, DEPTH_PID.max_output)
    
    -- Update for next iteration
    depth_pid_last_error = error
    depth_pid_last_time = current_time
    
    return output
end

function update_heading_pid(setpoint, measurement)
    local current_time = get_current_time_ms()
    local dt = (current_time - heading_pid_last_time) / 1000.0
    
    if dt <= 0.0 then
        dt = 0.01
    end
    
    -- Handle angle wrapping
    local error = setpoint - measurement
    if error > 180 then
        error = error - 360
    elseif error < -180 then
        error = error + 360
    end
    
    -- PID calculation
    local proportional = HEADING_PID.kp * error
    
    heading_pid_integral = heading_pid_integral + error * dt
    local integral_max = HEADING_PID.max_output / HEADING_PID.ki
    heading_pid_integral = constrain_float(heading_pid_integral, -integral_max, integral_max)
    local integral = HEADING_PID.ki * heading_pid_integral
    
    local derivative = HEADING_PID.kd * (error - heading_pid_last_error) / dt
    
    local output = proportional + integral + derivative
    output = constrain_float(output, -HEADING_PID.max_output, HEADING_PID.max_output)
    
    heading_pid_last_error = error
    heading_pid_last_time = current_time
    
    return output
end

-- ============================================================================
-- SENSOR READING FUNCTIONS
-- ============================================================================

function read_depth_sensor()
    -- Barometer tabanlı derinlik hesaplama (D300 sensör entegrasyonu için)
    local baro = barometer()
    if baro then
        local current_pressure = baro:get_pressure()
        if current_pressure then
            -- Hydrostatic pressure formula: depth = (P - P0) / (ρ * g)
            -- ρ = 1025 kg/m³ (seawater), g = 9.81 m/s²
            local pressure_diff = current_pressure - surface_pressure
            current_depth = math.max(0.0, (pressure_diff * 100) / (1025 * 9.81))
            return current_depth
        end
    end
    return nil
end

function read_current_position()
    local gps = gps:location()
    if gps then
        return gps:lat(), gps:lng()
    end
    return nil, nil
end

function read_current_heading()
    local ahrs = ahrs()
    if ahrs then
        local yaw = ahrs:get_yaw()
        if yaw then
            return math.deg(yaw)
        end
    end
    return nil
end

function check_battery_voltage()
    local battery = battery()
    if battery then
        local voltage = battery:voltage()
        if voltage then
            return voltage
        end
    end
    return nil
end

-- ============================================================================
-- MOTOR VE SERVO KONTROL FONKSİYONLARI
-- ============================================================================

function set_motor_throttle(channel, pwm_value)
    pwm_value = constrain_float(pwm_value, 1000, 2000)
    SRV_Channels:set_output_pwm_chan_timeout(channel - 1, pwm_value, 0)
end

function set_servo_position(channel, pwm_value)
    pwm_value = constrain_float(pwm_value, 1000, 2000)
    SRV_Channels:set_output_pwm_chan_timeout(channel - 1, pwm_value, 0)
end

function control_fins_x_configuration(roll_cmd, pitch_cmd, yaw_cmd)
    -- X konfigürasyonu fin kontrol matrisi (HARDWARE_PIN_MAPPING.md'ye uygun)
    local neutral = 1500
    
    local fin_1_pwm = neutral - pitch_cmd - roll_cmd + yaw_cmd  -- Front Left
    local fin_2_pwm = neutral - pitch_cmd + roll_cmd - yaw_cmd  -- Front Right
    local fin_3_pwm = neutral + pitch_cmd - roll_cmd - yaw_cmd  -- Rear Left
    local fin_4_pwm = neutral + pitch_cmd + roll_cmd + yaw_cmd  -- Rear Right
    
    set_servo_position(SERVO_FIN_1, fin_1_pwm)
    set_servo_position(SERVO_FIN_2, fin_2_pwm)
    set_servo_position(SERVO_FIN_3, fin_3_pwm)
    set_servo_position(SERVO_FIN_4, fin_4_pwm)
end

-- ============================================================================
-- UNDERWATER NAVIGATION MODES
-- ============================================================================

function depth_hold_control()
    local depth = read_depth_sensor()
    if not depth then
        return false
    end
    
    local depth_output = update_depth_pid(target_depth, depth)
    
    -- Motor throttle kontrolü
    local motor_throttle = 1500 + (depth_output / 2)  -- Scale depth output
    set_motor_throttle(MOTOR_MAIN, motor_throttle)
    
    -- Elevator kontrolü
    local elevator_pwm = 1500 - (depth_output / 3)
    set_servo_position(SERVO_ELEVATOR, elevator_pwm)
    
    return true
end

function waypoint_navigation()
    if current_waypoint_index >= #waypoints then
        return false  -- No more waypoints
    end
    
    local current_lat, current_lon = read_current_position()
    local current_heading = read_current_heading()
    
    if not current_lat or not current_lon or not current_heading then
        return false
    end
    
    local waypoint = waypoints[current_waypoint_index + 1]  -- Lua arrays are 1-indexed
    local distance = calculate_distance(current_lat, current_lon, waypoint.lat, waypoint.lon)
    local target_bearing = calculate_bearing(current_lat, current_lon, waypoint.lat, waypoint.lon)
    
    -- Check if waypoint reached
    if distance <= waypoint_tolerance then
        current_waypoint_index = current_waypoint_index + 1
        gcs:send_text(0, string.format("Waypoint %d reached!", current_waypoint_index))
        return true
    end
    
    -- Heading kontrolü
    local heading_output = update_heading_pid(target_bearing, current_heading)
    
    -- Distance-based speed control
    local speed_factor = math.min(1.0, distance / 10.0)  -- Slow down when close
    local forward_throttle = 1500 + (200 * speed_factor)  -- Max 200 PWM forward
    
    set_motor_throttle(MOTOR_MAIN, forward_throttle)
    
    -- Fin kontrolü (yaw only)
    control_fins_x_configuration(0, 0, heading_output)
    
    return true
end

function emergency_surface_mode()
    -- Acil yüzeye çıkış - maksimum hızla
    set_motor_throttle(MOTOR_MAIN, 1200)  -- Strong upward thrust
    set_servo_position(SERVO_ELEVATOR, 1200)  -- Nose up
    
    -- Finleri nötr pozisyonda tut
    control_fins_x_configuration(0, 0, 0)
    
    -- Payload bay'i acil durumda kapat
    set_servo_position(SERVO_PAYLOAD_BAY, 1000)
    
    return true
end

function rocket_launch_mode()
    -- +30° pitch angle için konfigürasyon
    local target_pitch = 30.0
    local current_pitch = ahrs():get_pitch()
    
    if current_pitch then
        current_pitch = math.deg(current_pitch)
        
        -- Pitch kontrolü
        local pitch_error = target_pitch - current_pitch
        local pitch_output = pitch_error * 5.0  -- Simple proportional control
        
        -- Elevator kontrolü ile pitch ayarı
        local elevator_pwm = 1500 + pitch_output
        set_servo_position(SERVO_ELEVATOR, elevator_pwm)
        
        -- Position hold throttle
        set_motor_throttle(MOTOR_MAIN, 1550)
        
        -- Pitch açısı uygun mu?
        if math.abs(pitch_error) < 5.0 then
            return true  -- Ready for rocket separation
        end
    end
    
    return false
end

-- ============================================================================
-- GÜVENLİK SİSTEMLERİ
-- ============================================================================

function check_safety_conditions()
    local current_time = get_current_time_ms()
    
    -- Battery voltage check
    local battery_voltage = check_battery_voltage()
    if battery_voltage and battery_voltage < MIN_BATTERY_VOLTAGE then
        gcs:send_text(0, "LOW BATTERY EMERGENCY!")
        emergency_triggered = true
        return false
    end
    
    -- Maximum depth check
    local depth = read_depth_sensor()
    if depth and depth > MAX_DEPTH then
        gcs:send_text(0, "MAX DEPTH EXCEEDED!")
        emergency_triggered = true
        return false
    end
    
    -- Mission timeout check
    if mission_start_time > 0 and (current_time - mission_start_time) > (EMERGENCY_SURFACE_TIMEOUT * 1000) then
        gcs:send_text(0, "MISSION TIMEOUT!")
        emergency_triggered = true
        return false
    end
    
    return true
end

function handle_emergency_situation()
    -- Switch to emergency surface mode
    underwater_mode = UNDERWATER_MODE_EMERGENCY_SURFACE
    target_depth = 0.0
    depth_hold_active = false
    
    gcs:send_text(0, "EMERGENCY MODE ACTIVATED!")
    
    return emergency_surface_mode()
end

-- ============================================================================
-- WAYPOINT YÖNETİMİ
-- ============================================================================

function add_waypoint(lat, lon, depth)
    local waypoint = {
        lat = lat,
        lon = lon,
        depth = depth or 2.0
    }
    table.insert(waypoints, waypoint)
    gcs:send_text(0, string.format("Waypoint added: %.6f, %.6f", lat, lon))
end

function clear_waypoints()
    waypoints = {}
    current_waypoint_index = 0
    gcs:send_text(0, "All waypoints cleared")
end

function load_mission_waypoints()
    -- TEKNOFEST Görev 1 için örnek waypoints
    clear_waypoints()
    
    local current_lat, current_lon = read_current_position()
    if current_lat and current_lon then
        -- 10m düz seyir waypoint
        local straight_lat = current_lat + 0.0001  -- Approximate 10m north
        add_waypoint(straight_lat, current_lon, 2.0)
        
        -- 50m uzaklaşma waypoint
        local offshore_lat = current_lat + 0.0005  -- Approximate 50m north
        add_waypoint(offshore_lat, current_lon, 2.0)
        
        -- Başlangıç noktasına dönüş
        add_waypoint(current_lat, current_lon, 2.0)
        
        gcs:send_text(0, "Mission waypoints loaded!")
    end
end

-- ============================================================================
-- MAVLink KOMUT İŞLEYİCİLERİ
-- ============================================================================

function handle_mavlink_command(command_int)
    local cmd = command_int:command()
    
    if cmd == mavlink_msgs.MAV_CMD_NAV_WAYPOINT then
        -- Waypoint komutu
        local lat = command_int:x() / 1e7
        local lon = command_int:y() / 1e7
        local alt = command_int:z()  -- Depth olarak kullan
        
        add_waypoint(lat, lon, alt)
        return mavlink_msgs.MAV_RESULT_ACCEPTED
        
    elseif cmd == mavlink_msgs.MAV_CMD_COMPONENT_ARM_DISARM then
        -- Arm/Disarm komutu
        local arm = command_int:param1()
        
        if arm == 1 then
            -- Arming
            mission_start_time = get_current_time_ms()
            underwater_mode = UNDERWATER_MODE_DEPTH_HOLD
            target_depth = 2.0  -- Default 2m depth
            depth_hold_active = true
            
            gcs:send_text(0, "UNDERWATER MODE ARMED!")
        else
            -- Disarming
            underwater_mode = 0
            depth_hold_active = false
            emergency_triggered = false
            
            -- Stop all motors/servos
            set_motor_throttle(MOTOR_MAIN, 1500)
            control_fins_x_configuration(0, 0, 0)
            
            gcs:send_text(0, "UNDERWATER MODE DISARMED!")
        end
        
        return mavlink_msgs.MAV_RESULT_ACCEPTED
        
    elseif cmd == mavlink_msgs.MAV_CMD_DO_SET_PARAMETER then
        -- Custom parameter setting
        local param_id = command_int:param1()
        local value = command_int:param2()
        
        if param_id == 1 then  -- Target depth
            target_depth = value
            gcs:send_text(0, string.format("Target depth set: %.1fm", target_depth))
        elseif param_id == 2 then  -- Underwater mode
            underwater_mode = value
            gcs:send_text(0, string.format("Underwater mode: %d", underwater_mode))
        end
        
        return mavlink_msgs.MAV_RESULT_ACCEPTED
    end
    
    return mavlink_msgs.MAV_RESULT_UNSUPPORTED
end

-- ============================================================================
-- ANA KONTROL DÖNGÜSÜ
-- ============================================================================

function update_underwater_control()
    -- Güvenlik kontrolleri
    if not check_safety_conditions() then
        if emergency_triggered then
            handle_emergency_situation()
            return
        end
    end
    
    -- Mode-specific control
    if underwater_mode == UNDERWATER_MODE_DEPTH_HOLD then
        depth_hold_control()
        
    elseif underwater_mode == UNDERWATER_MODE_WAYPOINT_NAV then
        if not waypoint_navigation() then
            -- No more waypoints, switch to depth hold
            underwater_mode = UNDERWATER_MODE_DEPTH_HOLD
            gcs:send_text(0, "Mission complete, holding depth")
        end
        depth_hold_control()  -- Always maintain depth
        
    elseif underwater_mode == UNDERWATER_MODE_EMERGENCY_SURFACE then
        emergency_surface_mode()
        
    elseif underwater_mode == UNDERWATER_MODE_ROCKET_LAUNCH then
        if rocket_launch_mode() then
            gcs:send_text(0, "Ready for rocket separation!")
        end
    end
    
    -- Heartbeat
    last_heartbeat_time = get_current_time_ms()
end

function send_underwater_telemetry()
    -- Custom telemetry messages
    local depth = read_depth_sensor() or 0.0
    local battery_voltage = check_battery_voltage() or 0.0
    
    -- Send as debug message
    gcs:send_text(6, string.format("UW: D=%.2fm T=%.2fm B=%.1fV M=%d", 
                                  depth, target_depth, battery_voltage, underwater_mode))
end

-- ============================================================================
-- SCRIPT BAŞLATMA VE MAIN LOOP
-- ============================================================================

function init()
    gcs:send_text(0, "TEKNOFEST Underwater Frame v1.0 Loaded!")
    
    -- Initialize PID timers
    local current_time = get_current_time_ms()
    depth_pid_last_time = current_time
    heading_pid_last_time = current_time
    
    -- Set surface pressure calibration
    local baro = barometer()
    if baro then
        local pressure = baro:get_pressure()
        if pressure then
            surface_pressure = pressure
            gcs:send_text(0, string.format("Surface pressure: %.1f mbar", surface_pressure))
        end
    end
    
    return true
end

-- Main update function - called by ArduPilot at 50Hz
function update()
    update_underwater_control()
    
    -- Send telemetry every 2 seconds (100 cycles at 50Hz)
    local current_time = get_current_time_ms()
    if (current_time % 2000) < 20 then  -- ~2 second intervals
        send_underwater_telemetry()
    end
    
    return update, 20  -- Schedule next run in 20ms (50Hz)
end

-- Register MAVLink command handler
mavlink_msgs.register_command_int_handler(handle_mavlink_command)

-- Script başlatma
gcs:send_text(0, "Initializing TEKNOFEST Underwater Frame...")
init()

-- Ana döngüyü başlat
return update() 