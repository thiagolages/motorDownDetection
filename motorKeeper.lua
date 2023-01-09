--variables
local numMotors = 6                         -- total number of motors
local motorsPWM = {-1, -1, -1, -1, -1, -1}  -- motorsPWM array
local motorsChannel = {37, 38, 35, 36, 34, 33}

local MODE_SMART_RTL        = 21
local PAYLOAD_CHANNEL       = 28
local PAYLOAD_RELEASE_PWM   = 1600

--local PWMmax = param:get("MOT_PWM_MAX")
local PWMmin = 1100 -- CHECK THIS VALUE BEFORE USING IN AIRCRAFT
local PWMmax = 1940

--local isGripOpen =  param:get("GRIP_RELEASE")
local startTime = -1

local function updateMotorsPWM()
    local channel
    for i = 1, numMotors-1, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
    end
end


local function areMotorsWithinPWMRange()

    local motorPWM = -1

    for i = 1, numMotors-1, 1 do
        motorPWM = motorsPWM[i]
        if (motorPWM >= PWMmax or motorPWM < PWMmin) then
            return false
        end
    end
    -- if nothing else returned false, then we should return true
    return true
end

local function isMotorDown()
    return areMotorsWithinPWMRange()
end

local function motorDownAction()

    if (startTime == -1) then
        startTime = millis()
    end

    if (millis() - startTime >= 1000) then

        -- send message
        gcs:send_text(6, "Motor Down")

        -- release payload
        SRV_Channels:set_output_pwm(PAYLOAD_CHANNEL,PAYLOAD_RELEASE_PWM)

        -- don't control YAW
        param:set('MOT_YAW_HEADROOM',0)

        -- disable precision landing
        param:set('PLND_ENABLED',0)

        -- change flght mode to smartRTL
        vehicle:set_mode(MODE_SMART_RTL)
    end
end


local function motorkeeper()

    -- upate motors PWM
    updateMotorsPWM()

    -- check if any motors have stopped
    if (isMotorDown()) then
        motorDownAction()
    end

    return motorkeeper, 100
end

gcs:send_text(6, "motor_keeper_v1.1_beta.lua is running")

return motorkeeper(), 10000

-- Code by Renan e Igão