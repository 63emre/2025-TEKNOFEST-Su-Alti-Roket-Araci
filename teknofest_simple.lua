-- TEKNOFEST Su Altı Roket Aracı - Basit Versiyon
-- Minimal hafıza kullanımı

local SERVO_FIN_1 = 1
local SERVO_FIN_2 = 2  
local SERVO_FIN_3 = 3
local SERVO_FIN_4 = 4

function set_servo_pwm(channel, pwm)
    pwm = math.max(1000, math.min(2000, pwm))
    SRV_Channels:set_output_pwm_chan_timeout(channel - 1, pwm, 0)
end

function control_fins(roll, pitch, yaw)
    local center = 1500
    
    set_servo_pwm(SERVO_FIN_1, center - pitch - roll + yaw)
    set_servo_pwm(SERVO_FIN_2, center - pitch + roll - yaw) 
    set_servo_pwm(SERVO_FIN_3, center + pitch - roll - yaw)
    set_servo_pwm(SERVO_FIN_4, center + pitch + roll + yaw)
end

function update()
    control_fins(0, 0, 0)  -- Finleri nötr pozisyonda tut
    return update, 1000    -- 1 saniyede bir çalış
end

gcs:send_text(0, "TEKNOFEST Simple Script Loaded!")
return update() 