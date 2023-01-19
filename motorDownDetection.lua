-- code by Thiago Lages (github: @thiagolages)
-- variables

-- run script at this amount of millisecond interval
local loop_time = 10 -- milliseconds. 1/10ms = 100Hz

local numMotors     = 6                         -- total number of motors
local motorsPWM     = { -1, -1, -1, -1, -1, -1} -- motorsPWM array
local motorsChannel = {33, 34, 35, 36, 37, 38}
-- TEMPORARY !!
-- local numMotors = 4
-- local motorsPWM     = { -1, -1, -1, -1} -- motorsPWM array
-- local motorsChannel = { 37, 38, 35, 36 }

local sentDisarmMessage     = false   -- to send disarm message only once
local startTimeModeAuto     = -1      -- will determine the initial time we're actually checking motors to be down
local startTimeMotorDown    = -1
-- modes and channel numbers
local mode -- current aircraft mode
local COPTER_MODE_STABILIZE = 0
local COPTER_MODE_ACRO      = 1
local COPTER_MODE_ALT_HOLD  = 2
local COPTER_MODE_AUTO      = 3
local COPTER_MODE_GUIDED    = 4
local COPTER_MODE_LOITER    = 5
local COPTER_MODE_RTL       = 6
local COPTER_MODE_SMART_RTL = 21

-- times
local WAIT_TIME_MODE_AUTO_MS        = 750  -- wait 750ms after AUTO is enabled before starting to check for problems
local TIME_DETECTION_THRESHOLD_MS   = 2000 -- (2s) Time frame to run this motor down detection algorithm, aftert WAIT_TIME_MODE_AUTO_MS have passed after mode AUTO (milliseconds)
local WAIT_TIME_MOTOR_DOWN_MS       = 300  -- (300ms) Time to wait before considering a motor failure, in milliseconds

local roll  -- roll  angle (degrees)
local pitch -- pitch angle (degrees)
local yaw   -- yaw   angle (degrees)

local rates     -- aircraft rates
local roll_rate -- gyro in X axis (rad/s) - roll
local pitch_rate-- gyro in Y axis (rad/s) - pitch
local yaw_rate  -- gyro in Z axis (rad/s) - yaw

-- Safety check by RPY angles (degrees)
ROLL_ANGLE_THRESH   = 15
PITCH_ANGLE_THRESH  = 15
-- YAW_ANGLE_THRESH    = 45

-- Safety check by RPY rates (radians/s)
local ROLL_RATE_THRESH   = 0.90
local PITCH_RATE_THRESH  = 0.90
local YAW_RATE_THRESH    = 0.90

-- Safety check by motor PWM (microsseconds)
local MOTOR_PWM_THRESH_MIN = 1400
local MOTOR_PWM_THRESH_MAX = 1800

--functions

local function isEmpty(s)
    return s == nil or s == ''
end

-- get motors updated PWM value
local function updateMotorsPWM()
    local channel
    for i = 1, numMotors, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
        --gcs:send_text(6, string.format("motor %d = %d", i, motorsPWM[i]))
    end
end

local function getAircraftData()
    mode = vehicle:get_mode() -- get current aircraft mode

    roll  = math.deg(ahrs:get_roll()) -- roll  angle (degrees)
    pitch = math.deg(ahrs:get_pitch())-- pitch angle (degrees)
    yaw   = math.deg(ahrs:get_yaw())  -- yaw   angle (degrees)

    rates = ahrs:get_gyro()
    roll_rate  = rates:x()    -- gyro in X axis (rad/s) - roll
    pitch_rate = rates:y()    -- gyro in Y axis (rad/s) - pitch
    yaw_rate   = rates:z()    -- gyro in Z axis (rad/s) - yaw
    -- gcs:send_text(6, string.format("r,p,y,rr,pr,yr = %.2f,%.2f,%.2f,%.2f,%.2f,%.2f",roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate))
end

local function RPAnglesOK() -- true if OK
    gcs:send_text(6, string.format("[motorDownDetection.lua] rpy = %.2f,%.2f,%.2f", roll, pitch, yaw))
    return(math.abs(roll) <= ROLL_ANGLE_THRESH and math.abs(pitch) <= PITCH_ANGLE_THRESH)
end

local function RPYRatesOK() -- true if OK
    return(math.abs(roll_rate) <= ROLL_RATE_THRESH and math.abs(pitch_rate) <= PITCH_RATE_THRESH and math.abs(yaw_rate) <= YAW_RATE_THRESH)
end

-- check if motors PWM is in healthy range
local function isMotorPWMOK(motorNum, motorPWM)

    if (motorPWM ~= nil) then
        if (motorPWM >= MOTOR_PWM_THRESH_MIN and motorPWM <= MOTOR_PWM_THRESH_MAX) then
            -- gcs:send_text(6, string.format("[motorDownDetection.lua] motor %d is OK, PWM = %d", motorNum+1, motorPWM ))    
            return true
        else
            gcs:send_text(6, string.format("[motorDownDetection.lua] motor %d is NOT OK, PWM = %d", motorNum, motorPWM ))
            return false
        end
    else
        gcs:send_text(6, string.format("[motorDownDetection.lua] PWM from motor %d is nil", motorNum))
        return true -- return true because we don't have information on it
    end

end

local function isAnyMotorOutsidePWMRange() -- false if OK
    local result = false -- return result
    local motorPWM
    for motorNum = 1, numMotors, 1 do
        motorPWM = motorsPWM[motorNum]
        if(not isMotorPWMOK(motorNum, motorPWM))then
            result = true
        end
    end
    return result
end

local function areMotorsPWMOK()
    if (isAnyMotorOutsidePWMRange()) then
        -- start couting first moment when motors were considered outside normal range
        if (startTimeMotorDown <= 0) then
            startTimeMotorDown = millis()
        end

        if(millis() - startTimeMotorDown >= WAIT_TIME_MOTOR_DOWN_MS) then
            return false
        else -- if motors are not outside PWM range for at least WAIT_TIME_MOTOR_DOWN_MS, we are ok 
            return true
        end
    else -- if no motors outside PWM range, everything is OK
        startTimeMotorDown = -1 -- restart counting initial motor down time
        return true
    end
end

local function areMotorsOK()
    return areMotorsPWMOK()
end

local function motorDownAction()
    arming:disarm() -- disarm first
    if (not sentDisarmMessage) then
        gcs:send_text(6, string.format("[motorDownDetection.lua] Motor Down !"))
        gcs:send_text(6, string.format("[motorDownDetection.lua] Disarming drone !"))    
    end
    sentDisarmMessage = true
end

local function isAircraftOK()
    return areMotorsOK() --and RPAnglesOK() 
end

local function shouldCheckMotorDown(timeSinceAuto)
     -- wait some time before checking for problems, and check them for a period of time only (only during takeoff)
    return (timeSinceAuto >= WAIT_TIME_MODE_AUTO_MS and timeSinceAuto <= (WAIT_TIME_MODE_AUTO_MS + TIME_DETECTION_THRESHOLD_MS))
end

local function checkMotorDown() -- main loop

    if (arming:is_armed()) then
        if (mode == COPTER_MODE_AUTO) then
        -- if (mode == COPTER_MODE_GUIDED) then
            if (startTimeModeAuto <= 0) then
                gcs:send_text(6, "[motorDownDetector.lua] starting to count startTimeModeAuto")
                startTimeModeAuto = millis()
            end

            local timeSinceAuto = millis() - startTimeModeAuto
            if (shouldCheckMotorDown(timeSinceAuto)) then
                gcs:send_text(6, "[motorDownDetector.lua] inside WAIT_TIME_MODE_AUTO_MS")
                if (not isAircraftOK()) then
                    motorDownAction()
                end
            end
        end -- end mode AUTO
    end -- end if armed()
    return checkMotorDown
end

local function main()

    --gcs:send_text(6, "--------------------------------")
    updateMotorsPWM()
    getAircraftData()
    checkMotorDown()
    return main, loop_time
end

gcs:send_text(6, "Started motorDownDetection.lua !")

return main() -- run immediately after the script is launched