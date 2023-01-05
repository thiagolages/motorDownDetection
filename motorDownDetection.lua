-- global variables
NumMotors = 6  -- total number of motors
MotorsPWM = {} -- motorsPWM array
InitialTime = tonumber(millis())
SentModeMessage = false

local loop_time = 100 -- run script at this amount of millisecond interval

--functions

local function updateMotorsPWM() -- loop to update Motors PWM values

    local servox_function = {37,38,35,36,34,33} -- SERVOx_FUNCTION in the autopilot

    for i=1,NumMotors,1 do
        MotorsPWM[i] = SRV_Channels:get_output_pwm(servox_function[i])
        --gcs:send_text(0, string.format("Motor %d = %d PWM", i, MotorsPWM[i]))
    end

end

local function checkMotorDown() -- main loop
    updateMotorsPWM()
    local mode = vehicle:get_mode() -- get current aircraft mode

    local rollErrorThresh   = 15
    local pitchErrorThresh  = 15
    local yawErrorThresh    = 15

    local roll  = math.deg(ahrs:get_roll())
    local pitch = math.deg(ahrs:get_pitch())
    local yaw   = math.deg(ahrs:get_yaw())

    -- COPTER_MODE_AUTO = 3
    -- gcs:send_text(0, string.format("Read Mode = %d",tonumber(mode)))
    if (mode == 3 and arming:is_armed()) then
        if (math.abs(roll) > rollErrorThresh or math.abs(pitch) > pitchErrorThresh or math.abs(yaw) > yawErrorThresh) then
            if (not SentModeMessage) then
                gcs:send_text(0, "Disarming drone because roll, pitch or yaw errors are too high !")
                SentModeMessage = true
            end
            -- disarm drone
            arming:disarm()
        end
    else
        SentModeMessage = false
    end

    return checkMotorDown, loop_time  -- run checkMotorDown() every 100ms
end

gcs:send_text(6, "Started motorDownDetection.lua !")

return checkMotorDown() -- run immediately after the script is launched

