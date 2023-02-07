-- variables

-- run script at this amount of millisecond interval
local loop_time     = 10 -- milliseconds. 1/10ms = 100Hz

local numMotors     = 6                         -- total number of motors
local motorsPWM     = { -1, -1, -1, -1, -1, -1} -- motorsPWM array
local motorsChannel = { 37, 38, 35, 36, 34, 33 }--{ 33, 34, 35, 36, 37, 38}

-- TEMPORARY (for sim only)!!
-- local numMotors      = 4
-- local motorsPWM      = { -1, -1, -1, -1} -- motorsPWM array
-- local motorsChannel  = { 37, 38, 35, 36 }

local timeSinceAuto         = -1    -- will be used to enable or nor some checks
local finishedTakeoff       = false -- will be used to run code only if we are taking off
local sentDisarmMessage     = false -- to send disarm message only once
local startTimeModeAuto     = -1    -- will determine the initial time we're actually checking motors to be down
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
local WAIT_TIME_MOTOR_DOWN_MS       = 250  -- (250ms) Time to wait before considering a motor failure, in milliseconds

-- angles
local roll      -- roll  angle (degrees)
local pitch     -- pitch angle (degrees)
local yaw       -- yaw   angle (degrees)

-- rates
local rates     -- aircraft rates
local roll_rate -- gyro in X axis (rad/s) - roll
local pitch_rate-- gyro in Y axis (rad/s) - pitch
local yaw_rate  -- gyro in Z axis (rad/s) - yaw

-- Safety checks

-- Safety check by RP angles (degrees)
local ROLL_ANGLE_THRESH   = 13
local PITCH_ANGLE_THRESH  = 13

-- Safety check by RPY rates (radians/s)
local ROLL_RATE_THRESH   = 0.80
local PITCH_RATE_THRESH  = 0.80
local YAW_RATE_THRESH    = 0.80

-- Safety check by motor PWM (microsseconds)
local MOTOR_PWM_THRESH_MIN = 1300
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
        -- TEMPORARY !!
        --gcs:send_text(6, string.format("motor %d = %d", i, motorsPWM[i]))
    end
end

-- get mode, rpy angles and velocities
local function getAircraftData()
    mode    = vehicle:get_mode() -- get current aircraft mode

    roll        = math.deg(ahrs:get_roll()) -- roll  angle (degrees)
    pitch       = math.deg(ahrs:get_pitch())-- pitch angle (degrees)
    yaw         = math.deg(ahrs:get_yaw())  -- yaw   angle (degrees)

    rates       = ahrs:get_gyro()
    roll_rate   = rates:x()    -- gyro in X axis (rad/s) - roll
    pitch_rate  = rates:y()    -- gyro in Y axis (rad/s) - pitch
    yaw_rate    = rates:z()    -- gyro in Z axis (rad/s) - yaw
    -- gcs:send_text(6, string.format("r,p,y,rr,pr,yr = %.2f,%.2f,%.2f,%.2f,%.2f,%.2f",roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate))
end

local function RPAnglesOK() -- true if OK
    --gcs:send_text(6, string.format("[motorDownDetection.lua] rpy = %.2f,%.2f,%.2f", roll, pitch, yaw))
    return(math.abs(roll) <= ROLL_ANGLE_THRESH and math.abs(pitch) <= PITCH_ANGLE_THRESH)
end

local function RPYRatesOK() -- true if OK
    --gcs:send_text(6, string.format("[motorDownDetection.lua] rpyRates = %.2f,%.2f,%.2f", roll_rate, pitch_rate, yaw_rate))
    return(math.abs(roll_rate) <= ROLL_RATE_THRESH and math.abs(pitch_rate) <= PITCH_RATE_THRESH and math.abs(yaw_rate) <= YAW_RATE_THRESH)
end

-- check if motors PWM is in healthy range
local function isMotorPWMOK(motorNum, motorPWM)

    if (motorPWM ~= nil) then
        if (motorPWM >= MOTOR_PWM_THRESH_MIN and motorPWM <= MOTOR_PWM_THRESH_MAX) then
            --gcs:send_text(6, string.format("[motorDownDetection.lua] motor %d is OK, PWM = %d", motorNum, motorPWM ))    
            return true
        else
            --gcs:send_text(6, string.format("[motorDownDetection.lua] motor %d is NOT OK, PWM = %d", motorNum, motorPWM ))
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

local function handleStartTimeMotorDown()
    if (startTimeMotorDown == -1) then
        startTimeMotorDown = millis()
    end
end

local function areMotorsPWMOK()
    if (timeSinceAuto >= WAIT_TIME_MODE_AUTO_MS)then -- only run after a delay
        if (isAnyMotorOutsidePWMRange()) then
            -- start couting first moment when motors were considered outside normal range
            handleStartTimeMotorDown()

            local timeSinceMotorDown = millis() - startTimeMotorDown

            if(timeSinceMotorDown >= WAIT_TIME_MOTOR_DOWN_MS) then
                gcs:send_text(6, string.format("[motorDownDetection.lua] Duration of PWM value is >= %d ms!",WAIT_TIME_MOTOR_DOWN_MS))
                return false
            else -- if motors are not outside PWM range for at least WAIT_TIME_MOTOR_DOWN_MS, we are ok
                return true
            end
        else -- if no motors outside PWM range, everything is OK
            startTimeMotorDown = -1 -- restart counting initial motor down time
            return true
        end
    else
        return true -- since we're not analyzing RPM yet, return true so we don't spoil the other aircraft logics
    end
end

local function areMotorsOK()
    return areMotorsPWMOK()
end

local function motorDownAction()
    arming:disarm() -- disarm first
    if (not sentDisarmMessage) then
        gcs:send_text(6, string.format("[motorDownDetection.lua] Motor Failure !"))
        gcs:send_text(6, string.format("[motorDownDetection.lua] Disarming drone !"))
    end
    sentDisarmMessage = true
end

local function isAircraftOK()
    return RPAnglesOK() and RPYRatesOK() and areMotorsOK()
end

local function checkTakeoffStatus() -- true means "taking off", false otherwise

    if (finishedTakeoff) then
        --gcs:send_text(6, "[motorDownDetector.lua] NOT running code anymore")
        return false -- if we finished motor dection, return immediately
    end

     -- consider taking off for a period of time only (first few seconds)
    local isTakingOff = (timeSinceAuto <= (WAIT_TIME_MODE_AUTO_MS + TIME_DETECTION_THRESHOLD_MS))

    -- if we're not within the time frame to check for motor down
    if (not isTakingOff) then
        finishedTakeoff = true -- finished detection
    end

    return isTakingOff
end

local function timeSinceAutoOverflow()
    -- to avoid conflicts with any overflow
    if (timeSinceAuto < 0) then
        return true
    else
        return false
    end
end

local function handleTimeSinceAuto()
    if ( startTimeModeAuto ~= -1 ) then
        timeSinceAuto = millis() - startTimeModeAuto
    end
end

local function handleStartTimeModeAuto()
    if ( startTimeModeAuto == -1 ) then -- will only run in the first time we enter AUTO mode
        --gcs:send_text(6, "[motorDownDetector.lua] starting to count startTimeModeAuto")
        startTimeModeAuto = millis()
    end
end

local function checkAircraftStatus() -- main loop

    if ( arming:is_armed() and mode == COPTER_MODE_AUTO ) then

        handleStartTimeModeAuto()                   -- initializes startTimeModeAuto, if needed
        handleTimeSinceAuto()                       -- initializes timeSinceAuto, if needed
        if (timeSinceAutoOverflow()) then return end-- if 'timeSinceAuto' overflows, we return

        if (checkTakeoffStatus()) then  -- true means "taking off", false otherwise
            --gcs:send_text(6, "[motorDownDetector.lua] inside WAIT_TIME_MODE_AUTO_MS")
            if (not isAircraftOK()) then
                motorDownAction()
            end
        end
    end -- end if armed()

    return checkAircraftStatus
end

local function main()

    --gcs:send_text(6, "--------------------------------")
    updateMotorsPWM()
    getAircraftData() -- get mode, rpy angles and velocities
    checkAircraftStatus()

    return main, loop_time
end

gcs:send_text(6, "[motorDownDetection.lua] Started motorDownDetection.lua !")

return main() -- run immediately after the script is launched