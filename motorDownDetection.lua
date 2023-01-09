-- code by Thiago Lages (github: @thiagolages)
-- variables
local numMotors           = 6       -- total number of motors
local motorsPWM           = {}      -- motorsPWM array
local sentDisarmMessage   = false   -- to send disarm message only once
local startTime           = -1      -- will determine the initial time we're actually checking motors to be down

local mode -- current aircraft mode

local roll  -- roll  angle (degrees)
local pitch -- pitch angle (degrees)
local yaw   -- yaw   angle (degrees)

local rates     -- aircraft rates
local roll_rate -- gyro in X axis (rad/s) - roll
local pitch_rate-- gyro in Y axis (rad/s) - pitch
local yaw_rate  -- gyro in Z axis (rad/s) - yaw

-- Safety check by RPY angles (degrees)
-- ROLL_ANGLE_THRESH   = 20
-- PITCH_ANGLE_THRESH  = 20
-- YAW_ANGLE_THRESH    = 20

-- Time frame to run this motor down detection algorithm (milliseconds)
local TIME_DETECTION_THRESHOLD = 3000 -- 3 seconds

-- Safety check by RPY rates (radians/s)
local ROLL_RATE_THRESH   = 0.70
local PITCH_RATE_THRESH  = 0.70
local YAW_RATE_THRESH    = 0.70

-- Safety check by motor PWM (microsseconds)
local MOTOR_PWM_THRESH_MIN = 1300
local MOTOR_PWM_THRESH_MAX = 1700

-- run script at this amount of millisecond interval
local loop_time = 50

--functions

local function updateMotorsPWM() -- loop to update Motors PWM values

    local servox_function = {37,38,35,36,34,33} -- SERVOx_FUNCTION in the autopilot

    for i=1,numMotors,1 do
        motorsPWM[i] = SRV_Channels:get_output_pwm(servox_function[i])
        --gcs:send_text(0, string.format("Motor %d = %d PWM", i, motorsPWM[i]))
    end

end

local function getAircraftStatus()
    mode = vehicle:get_mode() -- get current aircraft mode

    roll  = math.deg(ahrs:get_roll()) -- roll  angle (degrees)
    pitch = math.deg(ahrs:get_pitch())-- pitch angle (degrees)
    yaw   = math.deg(ahrs:get_yaw())  -- yaw   angle (degrees)

    rates = ahrs:get_gyro()
    roll_rate  = rates:x()    -- gyro in X axis (rad/s) - roll
    pitch_rate = rates:y()    -- gyro in Y axis (rad/s) - pitch
    yaw_rate   = rates:z()    -- gyro in Z axis (rad/s) - yaw
end

-- local function rpyAnglesOK() -- true if OK
--     return(math.abs(roll) <= ROLL_ANGLE_THRESH and math.abs(pitch) <= PITCH_ANGLE_THRESH and math.abs(yaw) <= YAW_ANGLE_THRESH)
-- end

local function rpyRatesOK() -- true if OK
    return(math.abs(roll_rate) <= ROLL_RATE_THRESH and math.abs(pitch_rate) <= PITCH_RATE_THRESH and math.abs(yaw_rate) <= YAW_RATE_THRESH)
end

local function isMotorPWMOK(motorPWM) -- true if OK
    return(motorPWM >= MOTOR_PWM_THRESH_MIN and motorPWM <= MOTOR_PWM_THRESH_MAX)
end

local function areMotorsPWMOK() -- true if OK
    local result = true -- return result
    local motorPWM
    for i=0,numMotors-1,1 do
        motorPWM = motorsPWM[i]
        if(not isMotorPWMOK(motorPWM))then
            result = false
        end
    end
    return result
end

local function areMotorsOK()
    return areMotorsPWMOK() and rpyRatesOK() -- add any other checks here, with a logical AND
end

local function checkMotorDown() -- main loop

    -- COPTER_MODE_AUTO = 3
    if (mode == 3 and arming:is_armed()) then
        if (startTime <= 0) then
            startTime = millis()
        end
        if ((millis() - startTime) <= TIME_DETECTION_THRESHOLD) then -- only check for motor down in the initial seconds after arming
            -- if (not rpyRatesOK(roll_rate, pitch_rate, yaw_rate)) then
            --     if (not sentDisarmMessage) then
            --         gcs:send_text(0, "Disarming drone because roll, pitch or yaw rates are too high !")
            --         sentDisarmMessage = true
            --     end
            --     -- disarm drone
            --     arming:disarm()
            -- end
            if (not areMotorsOK())then
                arming:disarm()
            end
        end
    else
        sentDisarmMessage = false
    end

    return checkMotorDown, loop_time  -- run checkMotorDown() every 100ms
end

local function main()

    updateMotorsPWM()
    getAircraftStatus()
    checkMotorDown()

end

gcs:send_text(6, "Started motorDownDetection.lua !")

return main() -- run immediately after the script is launched