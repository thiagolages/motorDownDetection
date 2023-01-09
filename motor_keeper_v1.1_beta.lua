--variáveis

--local pwmMax = param:get("MOT_PWM_MAX")
local pwmMax = 1940
local critico = false
--local gripAberto =  param:get("GRIP_RELEASE")
local start_time = -1
local now = -1
local rtl = false

function motorkeeper()

    --pegar os pwm dos motores
    local MOT1 = SRV_Channels:get_output_pwm(37)
    local MOT2 = SRV_Channels:get_output_pwm(38)
    local MOT3 = SRV_Channels:get_output_pwm(35)
    local MOT4 = SRV_Channels:get_output_pwm(36)
    local MOT5 = SRV_Channels:get_output_pwm(34)
    local MOT6 = SRV_Channels:get_output_pwm(33)

    --verificar se o motor parou
    if arming:is_armed() == true then --verifica se está armado
        if MOT1 >= pwmMax  or MOT2 >= pwmMax or MOT3 >= pwmMax or MOT4 >= pwmMax or MOT5 >= pwmMax or MOT6 >= pwmMax then
            critico = true
            now = millis()
        else
            critico = false
            start_time = -1
            now = -1
        end
    end

    --se ele parar conta o tempo
    if critico == true and rtl == false then
        if start_time == -1 then
            start_time = now
        end
        if now >= start_time + 1000 then
            -- menssagem
            gcs:send_text(6, "Motor Down")
            -- liberar a carga
            SRV_Channels:set_output_pwm(28,1600)
            --liberar YAW
            param:set('MOT_YAW_HEADROOM',0)
            --desligar plnd
            param:set('PLND_ENABLED',0)
            --mudar modo de voo para smarthRTL
            vehicle:set_mode(21)
            rtl = true
        end
    end
    return motorkeeper, 100
end
gcs:send_text(6, "motor_keeper_v1.1_beta.lua is running")
return motorkeeper(), 10000