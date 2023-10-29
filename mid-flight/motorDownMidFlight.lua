-- Code developed by Thiago Lages (github.com/thiagolages)
-- This code is provided under the MIT license.
-- Please refer to the LICENSE file in this repository.
-- The author provides NO WARRANTY, so use it at your own risk !

-- This script is intended to be used inside a CubeOrange Autopilot running ArduCopter v4.2.3+
-- script to check for possible motor failure and take action, if needed
-- by changing some parameters in-flight

----------------------------------------------------------------------------------
-- Please check the RPY angles, rates, and PWM ranges before starting to use it !
----------------------------------------------------------------------------------

local loop_time = 100 -- milliseconds. 1/100ms = 10Hz

-- enable debugging
local isDebug       = false                      -- use only when debugging

-- Safety checks

-- Safety check by RP angles (degrees)
local ROLL_ANGLE_THRESH     = 40
local PITCH_ANGLE_THRESH    = 40

-- Safety check by RPY rates (radians/s)
-- local ROLL_RATE_THRESH      = 0.90
-- local PITCH_RATE_THRESH     = 0.90
local YAW_RATE_THRESH       = 2.61 -- 2.61 rad/s ~= 150 degrees/s

-- Safety check by motor PWM (microsseconds)
local PWMoffset = 50                                    -- absolute value to add/subtract to PWMmin/MOTOR_PWM_THRESH_MAX to consider a motor failure

-- time control
local WAIT_TIME_MOTOR_DOWN_BY_PWM = 1500   -- time to wait before considering a motor failure, in milliseconds

----------------------------------------------------------------------------------------------------
----------------------------- DON'T CHANGE ANYTHING BEYOND THIS LINE ! -----------------------------
----------------------------------------------------------------------------------------------------

---------------
-- Variables --
---------------

local numMotors     = 6                         -- total number of motors
local motorsPWM     = {-1, -1, -1, -1, -1, -1}  -- motorsPWM array
local motorsChannel = {37, 38, 35, 36, 34, 33}  -- motors channels
local isArmed       = false                     -- whether aircraft is armed or not

-- MIN and MAX PWM values
local MOTOR_PWM_THRESH_MIN  = param:get("MOT_PWM_MIN") + PWMoffset  -- minimum value in crash log: 1284
local MOTOR_PWM_THRESH_MAX  = param:get("MOT_PWM_MAX") - PWMoffset  -- maximum value in crash log: 1898

-- Modes and channel numbers
local mode                      -- current aircraft mode
local COPTER_MODE_STABILIZE = 0
local COPTER_MODE_ACRO      = 1
local COPTER_MODE_ALT_HOLD  = 2
local COPTER_MODE_AUTO      = 3
local COPTER_MODE_GUIDED    = 4
local COPTER_MODE_LOITER    = 5
local COPTER_MODE_RTL       = 6
local COPTER_MODE_SMART_RTL = 21

-- Angles
local roll      -- roll  angle (degrees)
local pitch     -- pitch angle (degrees)
local yaw       -- yaw   angle (degrees)

-- Rates
local rates     -- aircraft rates
local roll_rate -- gyro in X axis (rad/s) - roll
local pitch_rate-- gyro in Y axis (rad/s) - pitch
local yaw_rate  -- gyro in Z axis (rad/s) - yaw

-- local TIME_AFTER_AUTO_SHOULD_DISARM_MS  = 3000  -- time after AUTO that we can disarm the drone, in milliseconds
local startTimeMotorDown                = -1    -- to control total motor failure time; helps to determine if it's a real failure or not
local startTimeModeAuto                 = -1    -- to control time after entering auto mode

-- will be used to take motor down action only once
local hasTakenMotorDownAction = false

-- check if default parameters are on drone
local isDefaultParameters = true;

-- parameters that will be changed in case the aircraft loses a motor
local PARAMETERS_NAMES = {
    "ANGLE_MAX",
    -- "ATC_SLEW_YAW",
    -- "ATC_ACCEL_Y_MAX",
    -- "ATC_ACCEL_R_MAX",
    -- "ATC_ACCEL_P_MAX",
    -- "ATC_RATE_R_MAX",
    -- "ATC_RATE_P_MAX",
    -- "ATC_RATE_Y_MAX",
    "MOT_YAW_HEADROOM",
    -- "PLND_ENABLED",
    "WPNAV_SPEED",
    "WPNAV_RADIUS",
    "WPNAV_ACCEL",
    -- "WPNAV_SPEED_DN"
}

-- will store parameters initial values. Will be filled by function getDefaultParams()
local PARAMETERS_DEFAULT = {}

-- parameters new values, in case aircraft loses a motor. Will be filled by function calculateParamsNewValue()
local PARAMETERS_NEW_VALUE = {}

local function isEmpty(s)
    return s == nil or s == ''
  end

-- get motors updated PWM value
local function updateMotorsPWM()
    local channel
    for i = 1, numMotors, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
        -- gcs:send_text(6, string.format("motor %d = %d", i, motorsPWM[i]))
    end
end

-- check if motors PWM is in healthy range
local function isAnyMotorOutsidePWMRange()

    local motorPWM = -1

    for i = 1, numMotors, 1 do
        -- gcs:send_text(6, string.format("i = %d", i))
        motorPWM = motorsPWM[i]
        if (motorPWM ~= nil) then
            if (motorPWM >= MOTOR_PWM_THRESH_MAX or motorPWM <= MOTOR_PWM_THRESH_MIN) then
                --if (not hasTakenMotorDownAction) then
                    --gcs:send_text(6, string.format("[MDMidFlight] motor %d is NOT OK, PWM = %d", i, motorPWM ))  
                --end
                return true
            end
            --gcs:send_text(6, string.format("motorPWM %d is OK, value %d", i, motorPWM ))
        else
            gcs:send_text(6, string.format("motorPWM %d is nil", i))
        end
    end
    -- if nothing else returned true, then we should return false
    -- gcs:send_text(6, string.format("everything OK, returning false"))
    return false
end

-- check if any motors are down
local function isMotorPWMOK()
    if (isAnyMotorOutsidePWMRange()) then

        if (startTimeMotorDown <= 0) then
            startTimeMotorDown = millis()
        end
        if(millis() - startTimeMotorDown >= WAIT_TIME_MOTOR_DOWN_BY_PWM) then
            if (not hasTakenMotorDownAction)then
                gcs:send_text(6, string.format("Motor PWM not OK for more than %d ms",WAIT_TIME_MOTOR_DOWN_BY_PWM))    
            end
            return false
        else
            return true
        end
    else
        -- restart counting initial motor down time
        startTimeMotorDown = -1
        return true
    end
end

-- get initial parameters that will be changed in case the aircraft loses a motor
local function getDefaultParams()
    -- fill PARAMETERS_DEFAULT table with initial parameters
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        PARAMETERS_DEFAULT[paramName] = param:get(paramName)
        if (isEmpty(PARAMETERS_DEFAULT[paramName]))then
            gcs:send_text(6, string.format("[MDMidFlight] %s NOT FOUND", paramName))
        else
            gcs:send_text(6, string.format("[MDMidFlight] %s default value is %f", paramName, PARAMETERS_DEFAULT[paramName]))
        end
    end
end

local function setDefaultParams()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        local result = param:set(paramName, PARAMETERS_DEFAULT[paramName])
        if (result) then
            gcs:send_text(6, string.format("[MDMidFlight] Changing %s to DEFAULT value: %f", paramName, PARAMETERS_DEFAULT[paramName]))    
        else
            gcs:send_text(6, string.format("[MDMidFlight] %s NOT FOUND, not setting it.", paramName))
        end
    end
    isDefaultParameters = true
end

local function setParamsNewValue()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        param:set(paramName, PARAMETERS_NEW_VALUE[paramName])
        gcs:send_text(6, string.format("[MDMidFlight] Changing %s to NEW value: %f", paramName, PARAMETERS_NEW_VALUE[paramName]))
    end
    isDefaultParameters = false
end

-- action to be taken in case the aircraft loses a motor
local function motorDownAction()

    -- send message
    gcs:send_text(0, "[MDMidFlight] Motor Down !")

    -- change parameters
    gcs:send_text(6, "[MDMidFlight] Changing parameter values")
    setParamsNewValue()

    hasTakenMotorDownAction = true

end

local function checkTriggerMode()

    local triggerMode

    if (isDebug) then
        triggerMode = COPTER_MODE_GUIDED -- for tests only
    else
        triggerMode = COPTER_MODE_AUTO
    end

    -- if we haven't entered AUTO at least once yet
    if (startTimeModeAuto <= 0 and mode == triggerMode) then
        gcs:send_text(6, "[MDMidFlight] Running code since entered AUTO.")
        startTimeModeAuto = millis() -- start the code
    end
end

-- Gets mode, RPY angles and velocities
local function getAircraftData()

    updateMotorsPWM()
    checkTriggerMode()

    mode        = vehicle:get_mode()            -- get current aircraft mode
    isArmed     = arming:is_armed()             -- if aircraft is armed

    -- if (not isArmed and not isDefaultParameters) then
    --     setDefaultParams()
    -- end

    roll        = math.deg(ahrs:get_roll())     -- roll  angle (degrees)
    pitch       = math.deg(ahrs:get_pitch())    -- pitch angle (degrees)
    yaw         = math.deg(ahrs:get_yaw())      -- yaw   angle (degrees)

    rates       = ahrs:get_gyro()
    roll_rate   = rates:x()                     -- gyro in X axis (rad/s) - roll
    pitch_rate  = rates:y()                     -- gyro in Y axis (rad/s) - pitch
    yaw_rate    = rates:z()                     -- gyro in Z axis (rad/s) - yaw

    if (isDebug and not hasTakenMotorDownAction) then
        gcs:send_text(6, string.format("r,p,y,rr,pr,yr = %.2f,%.2f,%.2f,%.2f,%.2f,%.2f",roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate))    
    end
end

local function isAttitudeOK()
    local result = (math.abs(roll) <= ROLL_ANGLE_THRESH and math.abs(pitch) <= PITCH_ANGLE_THRESH)
    if (not result and not hasTakenMotorDownAction) then
        gcs:send_text(0, string.format("[MDMidFlight] Aircraft has hit ROLL or PITCH ANGLES max values (%d, %d) ! Current values = (%f, %f)", ROLL_ANGLE_THRESH, PITCH_ANGLE_THRESH, roll, pitch))
    end
    return (result)
end

local function isRateYawOK()
    local result = (math.abs(yaw_rate) <= YAW_RATE_THRESH)
    if (not result and not hasTakenMotorDownAction) then
        gcs:send_text(0, string.format("[MDMidFlight] Aircraft has hit YAW RATE max value (%f) ! Current value = %f", YAW_RATE_THRESH, yaw_rate))
    end
    return (result)
end

local function isAircraftOK()
    local result
    if (isDebug) then
        result = (isAttitudeOK() and isRateYawOK())
    else
        result = (isAttitudeOK() and isRateYawOK() and isMotorPWMOK())
    end
    return result
end

-- main logic to check for motor failure and take action, if needed
local function motorDownMidFlight()

    getAircraftData()

    local condition

    if (isDebug) then
        condition = (startTimeModeAuto > 0) -- for tests only
    else
        condition = (isArmed and startTimeModeAuto > 0)
    end
    -- check if any motors have stopped
    if (condition) then
        if (not isAircraftOK() and not hasTakenMotorDownAction) then
            motorDownAction()
        end
    end

    return motorDownMidFlight, loop_time -- run every loop_time ms
end

-- setup the new parameter values to be set if aircraft loses a motor
local function calculateParamsNewValue()

    PARAMETERS_NEW_VALUE["ANGLE_MAX"]           = 2000  -- "ANGLE_MAX" - centidegrees (2000 = 20 degrees)
    PARAMETERS_NEW_VALUE["MOT_YAW_HEADROOM"]    = 0     -- "MOT_YAW_HEADROOM"
    PARAMETERS_NEW_VALUE["WPNAV_SPEED"]         = 1100  -- "WPNAV_SPEED"
    PARAMETERS_NEW_VALUE["WPNAV_RADIUS"]        = 750   -- "WPNAV_RADIUS"
    PARAMETERS_NEW_VALUE["WPNAV_ACCEL"]         = 50    -- "WPNAV_ACCEL"
    -- PARAMETERS_NEW_VALUE["WPNAV_SPEED_DN"]      = 120   -- "WPNAV_SPEED_DN"

    -- PARAMETERS_NEW_VALUE["ATC_SLEW_YAW"]    = 0.5 * PARAMETERS_DEFAULT["ATC_SLEW_YAW"]      -- "ATC_SLEW_YAW"
    -- PARAMETERS_NEW_VALUE["ATC_ACCEL_Y_MAX"] = 0.5 * PARAMETERS_DEFAULT["ATC_ACCEL_Y_MAX"]   -- "ATC_ACCEL_Y_MAX"
    -- PARAMETERS_NEW_VALUE["ATC_ACCEL_R_MAX"] = 0.7 * PARAMETERS_DEFAULT["ATC_ACCEL_R_MAX"]   -- "ATC_ACCEL_R_MAX"
    -- PARAMETERS_NEW_VALUE["ATC_ACCEL_P_MAX"] = 0.7 * PARAMETERS_DEFAULT["ATC_ACCEL_P_MAX"]   -- "ATC_ACCEL_P_MAX"
    -- PARAMETERS_NEW_VALUE["ATC_RATE_R_MAX"]  = 0.7 * PARAMETERS_DEFAULT["ATC_RATE_R_MAX"]    -- "ATC_RATE_R_MAX"
    -- PARAMETERS_NEW_VALUE["ATC_RATE_P_MAX"]  = 0.7 * PARAMETERS_DEFAULT["ATC_RATE_P_MAX"]    -- "ATC_RATE_P_MAX"
    -- PARAMETERS_NEW_VALUE["ATC_RATE_Y_MAX"]  = 0.7 * PARAMETERS_DEFAULT["ATC_RATE_Y_MAX"]    -- "ATC_RATE_Y_MAX"
    -- PARAMETERS_NEW_VALUE["PLND_ENABLED"]    = 0                                             -- "PLND_ENABLED"
end


local function printNewParams()
    for i, paramName in ipairs(PARAMETERS_NAMES) do
        if (not isEmpty(PARAMETERS_NEW_VALUE[paramName])) then
            gcs:send_text(6, string.format("[MDMidFlight] %s NEW value is %f", paramName, PARAMETERS_NEW_VALUE[paramName]))
        else
            gcs:send_text(6, string.format("[MDMidFlight] %s NEW value is nil",paramName))
        end
    end
    gcs:send_text(6, string.format("[MDMidFlight] MOTOR_PWM_THRESH_MIN = %f", MOTOR_PWM_THRESH_MIN))
    gcs:send_text(6, string.format("[MDMidFlight] MOTOR_PWM_THRESH_MAX = %f", MOTOR_PWM_THRESH_MAX))
    
end

-- setup function
local function setup()

    getDefaultParams()
    calculateParamsNewValue()
    printNewParams()

end


gcs:send_text(6, "[MDMidFlight] Started motorDownMidFlight.lua !")
setup() -- setup parameters and everythig else that needs a setup

return motorDownMidFlight(), 1000 -- run 1s after start