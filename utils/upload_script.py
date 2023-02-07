import os

# address to connect to your aircraft. Either locally (SITL) or real aircraft via wifi (something like 192.168.0.X,115200)
# Make sure to use the baud rate, separated by comma, in 'master_address', if connecting to your real aircraft (and not locally)
master_address              = "\"127.0.0.1:14880\"" 
#master_address              = "/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_27003F000E51313132383631-if00,115200"
local_scripts_folder        = os.getcwd() + "/"
ardupilot_scripts_folder    = "APM/scripts/"
lua_script_name             = "motorDownDetection.lua"

cmd="mavproxy.py --master="+master_address+" --cmd=\"module load ftp; ftp rm "+ardupilot_scripts_folder+lua_script_name+"; ftp put "+local_scripts_folder+lua_script_name+" "+ardupilot_scripts_folder+lua_script_name+"; scripting restart;\""
print(cmd)
os.system(cmd)
exit()
