-- Code developed by Thiago Lages (github.com/thiagolages)
-- This code is provided under the MIT license.
-- Please refer to the LICENSE file in this repository.
-- The author provides NO WARRANTY, so use it at your own risk !

----------------------------------------------------------------------------------
-- Please check the RPY angles, rates, and PWM ranges before starting to use it !
----------------------------------------------------------------------------------

---------------
-- Variables --
---------------

-- Run script at this amount of millisecond interval
local loop_time     = 10 -- milliseconds. 1/10ms = 100Hz

-- enable debugging
local isDebug       = false                      -- use only when debugging

-- Motors channels and values
local numMotors             = 6                         -- total number of motors
local motorsPWM             = { -1, -1, -1, -1, -1, -1} -- motorsPWM array
local motorsChannel         = { 37, 38, 35, 36, 34, 33 }--{ 33, 34, 35, 36, 37, 38}

------------------------------------------------------------------------------------------------------------------------------
-- Uncomment if using on a quad simulator (like Gazebo Iris), and comment above 'numMotors', 'motorsPWM' and 'motorsChannel'

-- local numMotors      = 4
-- local motorsPWM      = { -1, -1, -1, -1} -- motorsPWM array
-- local motorsChannel  = { 37, 38, 35, 36 }

-- Also, change COPTER_MODE_AUTO inside 'motorDownDetection' function to any other mode you want.
------------------------------------------------------------------------------------------------------------------------------

local timeSinceAuto         = -1    -- will be used to enable or nor some checks
local finishedTakeoff       = false -- will be used to run code only if we are taking off
local sentDisarmMessage     = false -- to send disarm message only once
local startTimeModeAuto     = -1    -- will determine the initial time we're actually checking motors to be down
local startTimeMotorDown    = -1

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

-- Time frames
local WAIT_TIME_MODE_AUTO_MS        = 750  -- wait 750ms after AUTO is enabled before starting to check for problems
local TIME_DETECTION_THRESHOLD_MS   = 2000 -- (2s) Time frame to run this motor down detection algorithm, aftert WAIT_TIME_MODE_AUTO_MS have passed after mode AUTO (milliseconds)
local WAIT_TIME_MOTOR_DOWN_MS       = 250  -- (250ms) Time to wait before considering a motor failure, in milliseconds

-- Angles
local roll      -- roll  angle (degrees)
local pitch     -- pitch angle (degrees)
local yaw       -- yaw   angle (degrees)

-- Rates
local rates     -- aircraft rates
local roll_rate -- gyro in X axis (rad/s) - roll
local pitch_rate-- gyro in Y axis (rad/s) - pitch
local yaw_rate  -- gyro in Z axis (rad/s) - yaw

-- Safety checks

-- Safety check by RP angles (degrees)
local ROLL_ANGLE_THRESH     = 13
local PITCH_ANGLE_THRESH    = 13

-- Safety check by RPY rates (radians/s)
local ROLL_RATE_THRESH      = 0.80
local PITCH_RATE_THRESH     = 0.80
local YAW_RATE_THRESH       = 0.80

-- Safety check by motor PWM (microsseconds)
-- MOT_PWM
local MOT_PWM_MIN = param:get("MOT_PWM_MIN")
local MOT_PWM_MAX = param:get("MOT_PWM_MAX")

-- MOT_SPIN
local MOT_SPIN_MIN = param:get("MOT_SPIN_MIN")
local MOT_SPIN_MAX = param:get("MOT_SPIN_MAX")

-- THRESHOLDS
local PWMRange          = math.floor( MOT_PWM_MAX - MOT_PWM_MIN )               -- PWM range used in calculations
local motSpinMinPWM     = MOT_PWM_MIN + math.floor( MOT_SPIN_MIN*PWMRange )     -- Motor minimum spin PWM value, corrected using MOT_SPIN_MIN
local PWMoffsetFromMin  = 100                                                   -- Absolute value to add to motSpinMinPWM to consider a motor failure. Calculate from logs.

-- Safety check by motor PWM (microsseconds)
local MOTOR_PWM_THRESH_MIN  = motSpinMinPWM + PWMoffsetFromMin                  -- Threshold considers corrected spin min and adds an offset.   Minimum value in crash log: 1284
local MOTOR_PWM_THRESH_MAX  = 1800                                              -- Using absolute value for max threshold.                      Maximum value in crash log: 1898

---------------
-- Functions --
---------------

-- Gets motors updated PWM value
local function updateMotorsPWM()
    local channel
    for i = 1, numMotors, 1 do
        channel     = motorsChannel[i]
        motorsPWM[i]= SRV_Channels:get_output_pwm(channel)
    end
end

-- Gets mode, RPY angles and velocities
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

-- Checks if Roll, Pith and Yaw angles are within the normal range (in degrees). Returns true if all is OK
local function RPAnglesOK()
    local anglesOK = (math.abs(roll) <= ROLL_ANGLE_THRESH and math.abs(pitch) <= PITCH_ANGLE_THRESH)
    if (debug and not anglesOK) then -- print cause of motor stop detection
       gcs:send_text(6, string.format("[motorDownDetection] rpy = %.2f,%.2f,%.2f", roll, pitch, yaw))
       gcs:send_text(6, string.format("[motorDownDetection] rp threshold (abs) = %.2f,%.2f", ROLL_ANGLE_THRESH, PITCH_ANGLE_THRESH))
    end
    return (anglesOK)
end

-- Checks if Roll, Pith and Yaw rates (angular velocities) are within the normal range (in radians/s). Returns true if all is OK
local function RPYRatesOK()
    local ratesOK = (math.abs(roll_rate) <= ROLL_RATE_THRESH and math.abs(pitch_rate) <= PITCH_RATE_THRESH and math.abs(yaw_rate) <= YAW_RATE_THRESH)
    if (debug and not ratesOK) then -- print cause of motor stop detection
       gcs:send_text(6, string.format("[motorDownDetection] rpyRates = %.2f,%.2f,%.2f", roll_rate, pitch_rate, yaw_rate))
       gcs:send_text(6, string.format("[motorDownDetection] rpyRates threshold (abs) = %.2f,%.2f,%.2f", ROLL_RATE_THRESH, PITCH_RATE_THRESH, YAW_RATE_THRESH))
    end
    return(ratesOK)
end

-- Checks if motors PWM is in healthy range
local function isMotorPWMOK(motorNum, motorPWM)

    if (motorPWM ~= nil) then
        if (motorPWM >= MOTOR_PWM_THRESH_MIN and motorPWM <= MOTOR_PWM_THRESH_MAX) then
            --gcs:send_text(6, string.format("[motorDownDetection] motor %d is OK, PWM = %d", motorNum, motorPWM ))    
            return true
        else
            if(isDebug) then
                gcs:send_text(6, string.format("[motorDownDetection] motor %d is NOT OK, PWM = %d", motorNum, motorPWM ))
            end
            return false
        end
    else
        if(isDebug) then
            gcs:send_text(6, string.format("[motorDownDetection] PWM from motor %d is nil", motorNum))
        end
        return true -- return true because we don't have information on it
    end

end

-- Checks if any motorsPWM are outside the specified range
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

-- Handles initialization of startTimeMotorDown counter
local function handleStartTimeMotorDown()
    if (startTimeMotorDown == -1) then
        startTimeMotorDown = millis()
    end
end

-- Checks if motors PWM are outside the range for long enough to be considered as 'down'
local function areMotorsPWMOK()
    if (timeSinceAuto >= WAIT_TIME_MODE_AUTO_MS)then -- only run after a delay
        if (isAnyMotorOutsidePWMRange()) then
            -- start counting the first moment when motors were considered outside normal range
            handleStartTimeMotorDown()

            local timeSinceMotorDown = millis() - startTimeMotorDown

            if(timeSinceMotorDown >= WAIT_TIME_MOTOR_DOWN_MS) then
                if(isDebug) then
                    gcs:send_text(6, string.format("[motorDownDetection] Duration of PWM value is >= %d ms!",WAIT_TIME_MOTOR_DOWN_MS))
                    gcs:send_text(6, string.format("[motorDownDetection] Motors PWM min/max threshold = %d, %d", MOTOR_PWM_THRESH_MIN, MOTOR_PWM_THRESH_MAX))
                end
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

-- Checks if motors are in normal conditions. Currently only analyzing PWM values but more checks can be added,
-- such as motor current, temperature, etc., and appended to the return statement, in the form of functions
local function areMotorsOK()
    return areMotorsPWMOK()
end

-- Action to take if a motor failure is detected
local function motorDownAction()
    arming:disarm() -- disarm first
    if (not sentDisarmMessage) then
        gcs:send_text(6, string.format("[motorDownDetection] Motor Failure !"))
        gcs:send_text(6, string.format("[motorDownDetection] Disarming drone !"))
    end
    sentDisarmMessage = true
end

-- Checks if aircraft status is OK during takeoff
local function isAircraftOK()
    return RPAnglesOK() and RPYRatesOK() and areMotorsOK() -- -- more checks can be added, like GPS status, EKF status, etc.
end

-- Checks takeoff status. True means "taking off", false otherwise
local function checkTakeoffStatus()

    if (finishedTakeoff) then
        --gcs:send_text(6, "[motorDownDetection] NOT running code anymore")
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

-- Avoids conflicts with any overflow
local function timeSinceAutoOverflow()
    if (timeSinceAuto < 0) then
        return true
    else
        return false
    end
end

-- Handles initialization of timeSinceAuto counter
local function handleTimeSinceAuto()
    if ( startTimeModeAuto ~= -1 ) then
        timeSinceAuto = millis() - startTimeModeAuto
    end
end

-- Handles initialization startTimeModeAuto counter
local function handleStartTimeModeAuto()
    if ( startTimeModeAuto == -1 ) then -- will only run in the first time we enter AUTO mode
        --gcs:send_text(6, "[motorDownDetection] starting to count startTimeModeAuto")
        startTimeModeAuto = millis()
    end
end

-- Main loop. Checks if any motors are down, as soon as aircraft is armed and in AUTO mode.
local function motorDownDetection()

    if ( arming:is_armed() and mode == COPTER_MODE_AUTO ) then

        handleStartTimeModeAuto()                   -- initializes startTimeModeAuto, if needed
        handleTimeSinceAuto()                       -- initializes timeSinceAuto, if needed
        if (timeSinceAutoOverflow()) then return end-- if 'timeSinceAuto' overflows, we return

        if (checkTakeoffStatus()) then              -- true means "taking off", false otherwise
            --gcs:send_text(6, "[motorDownDetection] inside WAIT_TIME_MODE_AUTO_MS")
            if (not isAircraftOK()) then
                motorDownAction()
            end
        end
    end -- end if armed()

    return motorDownDetection
end

local function main()

    updateMotorsPWM()
    getAircraftData()
    motorDownDetection()

    return main, loop_time
end

gcs:send_text(6, string.format("[motorDownDetection] Started motorDownDetection.lua !"))
gcs:send_text(6, string.format("[motorDownDetection] RP Angles = %.1f,%.1f"      ,ROLL_ANGLE_THRESH   , PITCH_ANGLE_THRESH))
gcs:send_text(6, string.format("[motorDownDetection] RPY Rates = %.2f,%.2f,%.2f" ,ROLL_RATE_THRESH    , PITCH_RATE_THRESH, YAW_RATE_THRESH))
gcs:send_text(6, string.format("[motorDownDetection] PWM Values= (%d,%d)"        ,MOTOR_PWM_THRESH_MIN, MOTOR_PWM_THRESH_MAX))
if (isDebug) then
    gcs:send_text(6, string.format("[motorDownDetection] MOT_PWM_MIN,MOT_PWM_MAX Values = (%d,%d)"       ,MOT_PWM_MIN,  MOT_PWM_MAX))
    gcs:send_text(6, string.format("[motorDownDetection] MOT_SPIN_MIN,MOT_SPIN_MAX Values = (%.2f,%.2f)" ,MOT_SPIN_MIN, MOT_SPIN_MAX))
end

return main() -- run immediately after the script is launched