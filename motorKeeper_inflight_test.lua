-- This script is intended to be used inside a CubeOrange Autopilot running ArduCopter v4.3+
-- script to check for possible motor failure and take action, if needed
-- by changing some parameters in-flight

-- variables
-- CHANGE TO 6 ON HEXA !!
local numMotors = 6                         -- total number of motors
local motorsPWM = {-1, -1, -1, -1, -1, -1}  -- motorsPWM array
local motorsChannel = {37, 38, 35, 36, 34, 33}

-- modes and channel numbers
local MODE_ALT_HOLD         = 2
local MODE_AUTO             = 3
local MODE_SMART_RTL        = 21
local CHANNEL_PAYLOAD       = 28
local PAYLOAD_RELEASE_PWM   = 1600

-- time control
local WAIT_TIME_MOTOR_DOWN              = 2000 -- time to wait before considering a motor failure, in milliseconds
local WAIT_TIME_MODE_AUTO_MS            = 1000 -- time to wait after entering AUTO mode, to look for potential motor failure, in milliseconds
-- local TIME_AFTER_AUTO_SHOULD_DISARM_MS  = 3000  -- time after AUTO that we can disarm the drone, in milliseconds
local startTimeMotorDown                = -1    -- to control total motor failure time; helps to determine if it's a real failure or not
local startTimeModeAuto                 = -1    -- to control time after entering auto mode

-- local PWMmax = param:get("MOT_PWM_MAX")
local PWMmin = 1290 -- minimum value in crash log: 1284
local PWMmax = 1890 -- maximum value in crash log: 1898

-- parameters that will be changed in case the aircraft loses a motor
local PARAMETERS_NAMES = {
    "MOT_YAW_HEADROOM",
    "ATC_SLEW_YAW",
    "ATC_ACCEL_Y_MAX",
    "ATC_ACCEL_R_MAX",
    "ATC_ACCEL_P_MAX",
    "ATC_RATE_R_MAX",
    "ATC_RATE_P_MAX",
    "ATC_RATE_Y_MAX",
    "PLND_ENABLED",
    "WPNAV_SPEED",
    "WPNAV_RADIUS",
    "WPNAV_ACCEL",
    "WPNAV_SPEED_DN"
}

-- will store parameters initial values. Will be filled by function getInitialParams()
local PARAMETERS = {}

-- parameters new values, in case aircraft loses a motor. Will be filled by function calculateParamsNewValue()
local PARAMETERS_NEW_VALUE = {}

-- get motors updated PWM value
local function updateMotorsPWM()
    local channel
    for i = 1, numMotors, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
        -- gcs:send_text(0, string.format("motor %d = %d", i, motorsPWM[i]))
    end
end

-- check if motors PWM is in healthy range
local function isAnyMotorOutsidePWMRange()

    local motorPWM = -1

    for i = 1, numMotors, 1 do
        -- gcs:send_text(0, string.format("i = %d", i))
        motorPWM = motorsPWM[i]
        if (motorPWM ~= nil) then
            if (motorPWM >= PWMmax or motorPWM <= PWMmin) then
                gcs:send_text(6, string.format("motor %d is NOT OK, PWM = %d", i, motorPWM ))
                return true
            end
            -- gcs:send_text(6, string.format("motorPWM %d is OK, value %d", i, motorPWM ))
        else
            gcs:send_text(6, string.format("motorPWM %d is nil", i))
        end
    end
    -- if nothing else returned true, then we should return false
    -- gcs:send_text(0, string.format("everything OK, returning false"))
    return false
end

-- check if any motors are down
local function isMotorDown()
    if (isAnyMotorOutsidePWMRange()) then

        if (startTimeMotorDown <= 0) then
            startTimeMotorDown = millis()
        end
        if(millis() - startTimeMotorDown >= WAIT_TIME_MOTOR_DOWN) then
            return true
        end
    else
        -- restart counting initial motor down time
        startTimeMotorDown = -1
    end

    return false
end

local function setParamsNewValue()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        param:set(paramName, PARAMETERS_NEW_VALUE[paramName])
        gcs:send_text(0, string.format("[motorKeeper.lua] Changing %s to %f", paramName, PARAMETERS_NEW_VALUE[paramName]))
    end
end

-- action to be taken in case the aircraft loses a motor
local function motorDownAction()

    -- send message
    gcs:send_text(0, "[motorKeeper.lua] Motor Down")

    -- change parameters
    gcs:send_text(0, "[motorKeeper.lua] Changing parameter values")
    setParamsNewValue()

    -- change flght mode to smartRTL
    gcs:send_text(0, "[motorKeeper.lua] Changing mode to SMART_RTL")
    vehicle:set_mode(MODE_SMART_RTL)

end


-- main logic to check for motor failure and take action, if needed
local function motorkeeper()

    updateMotorsPWM()

    -- check if any motors have stopped
    local mode = vehicle:get_mode()
    if(mode == MODE_AUTO and vehicle:is_armed()) then
        -- gcs:send_text(6, "[motorKeeper.lua] mode AUTO")
        if (startTimeModeAuto <= 0)then
            gcs:send_text(6, "[motorKeeper.lua] starting to count startTimeModeAuto")
            startTimeModeAuto = millis()
        end
        local timeSinceAuto = millis() - startTimeModeAuto
        if (timeSinceAuto >= WAIT_TIME_MODE_AUTO_MS) then
            if (isMotorDown()) then
                gcs:send_text(6, "[motorKeeper.lua] inside WAIT_TIME_MODE_AUTO_MS")
                motorDownAction()
            end
        end
    else
        startTimeModeAuto = -1
    end

    return motorkeeper, 50 -- run every 50ms
end

-- get initial parameters that will be changed in case the aircraft loses a motor
local function getInitialParams()
    -- fill PARAMETERS table with initial parameters
    for i, paramName in ipairs(PARAMETERS_NAMES) do
        PARAMETERS[paramName] = param:get(paramName)
        gcs:send_text(0, string.format(" %s initial value is %f", paramName, PARAMETERS[paramName]))
    end
end

-- setup the new parameter values to be set if aircraft loses a motor
local function calculateParamsNewValue()

    PARAMETERS_NEW_VALUE["MOT_YAW_HEADROOM"]= 0                                   -- "MOT_YAW_HEADROOM"
    PARAMETERS_NEW_VALUE["ATC_SLEW_YAW"]    = 0.5 * PARAMETERS["ATC_SLEW_YAW"]    -- "ATC_SLEW_YAW"
    PARAMETERS_NEW_VALUE["ATC_ACCEL_Y_MAX"] = 0.5 * PARAMETERS["ATC_ACCEL_Y_MAX"] -- "ATC_ACCEL_Y_MAX"
    PARAMETERS_NEW_VALUE["ATC_ACCEL_R_MAX"] = 0.7 * PARAMETERS["ATC_ACCEL_R_MAX"] -- "ATC_ACCEL_R_MAX"
    PARAMETERS_NEW_VALUE["ATC_ACCEL_P_MAX"] = 0.7 * PARAMETERS["ATC_ACCEL_P_MAX"] -- "ATC_ACCEL_P_MAX"
    PARAMETERS_NEW_VALUE["ATC_RATE_R_MAX"]  = 0.7 * PARAMETERS["ATC_RATE_R_MAX"]  -- "ATC_RATE_R_MAX"
    PARAMETERS_NEW_VALUE["ATC_RATE_P_MAX"]  = 0.7 * PARAMETERS["ATC_RATE_P_MAX"]  -- "ATC_RATE_P_MAX"
    PARAMETERS_NEW_VALUE["ATC_RATE_Y_MAX"]  = 0.7 * PARAMETERS["ATC_RATE_Y_MAX"]  -- "ATC_RATE_Y_MAX"
    PARAMETERS_NEW_VALUE["PLND_ENABLED"]    = 0                                   -- "PLND_ENABLED"
    PARAMETERS_NEW_VALUE["WPNAV_SPEED"]     = 1000                                -- "WPNAV_SPEED"
    PARAMETERS_NEW_VALUE["WPNAV_RADIUS"]    = 750                                 -- "WPNAV_RADIUS"
    PARAMETERS_NEW_VALUE["WPNAV_ACCEL"]     = 150                                 -- "WPNAV_ACCEL"
    PARAMETERS_NEW_VALUE["WPNAV_SPEED_DN"]  = 120                                 -- "WPNAV_SPEED_DN"

end


local function printParamsNewValue()
    for i, paramName in ipairs(PARAMETERS_NAMES) do
        gcs:send_text(0, string.format("%s new value is %f", paramName, PARAMETERS_NEW_VALUE[paramName]))
    end
end

-- setup function
local function setupMotorKeeper()

    getInitialParams()
    calculateParamsNewValue()
    printParamsNewValue()
    
end


gcs:send_text(0, "motorKeeper.lua is running")
setupMotorKeeper() -- setup parameters and everythig else that needs a setup

return motorkeeper(), 5000 -- run 5s after start