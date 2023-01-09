-- script to check for possible motor failure and take action, if needed
-- by changing some parameters in-flight

-- variables
local numMotors = 6                         -- total number of motors
local motorsPWM = {-1, -1, -1, -1, -1, -1}  -- motorsPWM array
local motorsChannel = {37, 38, 35, 36, 34, 33}

-- modes and channel numbers
local MODE_SMART_RTL        = 21
local CHANNEL_PAYLOAD       = 28
local PAYLOAD_RELEASE_PWM   = 1600

-- local PWMmax = param:get("MOT_PWM_MAX")
local PWMmin = 1100 -- CHECK THIS VALUE BEFORE USING IN AIRCRAFT
local PWMmax = 1940

-- local isGripOpen =  param:get("GRIP_RELEASE")
local startTime = -1

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
    for i = 1, numMotors-1, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
    end
end

-- check if motors PWM is in healthy range
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

-- check if any motors are down
local function isMotorDown()
    return areMotorsWithinPWMRange()
end

local function setParamsNewValue()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        param:set(paramName, PARAMETERS_NEW_VALUE[paramName])
        gcs:send_text(0, string.format("changing %s to %f", paramName, PARAMETERS_NEW_VALUE[paramName]))
    end
end

-- action to be taken in case the aircraft loses a motor
local function motorDownAction()

    if (startTime == -1) then
        startTime = millis()
    end

    if (millis() - startTime >= 1000) then

        -- send message
        gcs:send_text(6, "Motor Down")

        setParamsNewValue()

        -- -- release payload
        -- SRV_Channels:set_output_pwm(CHANNEL_PAYLOAD,PAYLOAD_RELEASE_PWM)

        -- -- don't control YAW
        -- param:set('MOT_YAW_HEADROOM',0)

        -- -- disable precision landing
        -- param:set('PLND_ENABLED',0)

        -- change flght mode to smartRTL
        vehicle:set_mode(MODE_SMART_RTL)
    end
end


-- main logic to check for motor failure and take action, if needed
local function motorkeeper()

    -- upate motors PWM
    updateMotorsPWM()

    -- check if any motors have stopped
    if (isMotorDown()) then
        motorDownAction()
    end

end

-- get initial parameters that will be changed in case the aircraft loses a motor
local function getInitialParams()
    -- fill PARAMETERS table with initial parameters
    for i, paramName in ipairs(PARAMETERS_NAMES) do
        PARAMETERS[paramName] = param:get(paramName)
        gcs:send_text(0, string.format("%s initial value is %f", paramName, PARAMETERS[paramName]))
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
local function setup()

    getInitialParams()
    calculateParamsNewValue()
    printParamsNewValue()
    
end

-- main loop function
local function loop()
    motorkeeper()
    return loop, 100 -- run every 100ms
end


gcs:send_text(6, "motorKeeper.lua is running")
setup() -- setup parameters and everythig else that needs a setup

-- return loop(), 10000 -- run 10s after start