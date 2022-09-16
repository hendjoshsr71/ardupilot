-- EFI Scripting backend driver for SkyPower

-- Authors: Andrew Tridgell & Josh Henderson

local SCR_VERSION = "4.4"
local DRIVER_NAME = "Skypower CAN"


--[[ Settings Required
Notes: the Skypower Driver uses addresses only in the CAN 11-bit address space. Thus if you can guarantee other devices 
do not utilize these addresses multiple drivers may use the bus. However, the Skypower protocol

The protocol has no method to identify multiple engines on the same bus.
The protocol has high CAN utilization due to CAN packets that are used internal to the enigne being published externally.
As well as being limited to 500 kbit/s.

CAN_D1_PROTOCOL 10 (Scripting Driver 1)
CAN_P1_DRIVER 1 (First driver)
CAN_D1_BITRATE 500000 (500 kbit/s)

--]]

---- GLOBAL USER PARAMETERS
local MAV_SEVERITY_ERROR = 3

local EFI1_PARAM_TABLE = {}
EFI_PARAM_TABLE.KEY = 35
EFI_PARAM_TABLE.PREFIX = "EFI1_"

---- END: GLOBAL USER PARAMETERS


---- GENERIC FUNCTIONS

-- bind a parameter to a variable given 
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(param_table, name, idx, default_value)
   -- gcs:send_text(0, string.format("Key = %f, prefix = %s, name = %s", param_table.KEY, param_table.PREFIX, name))
   assert(param:add_param(param_table.KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(param_table.PREFIX .. name)
end

function get_time_sec()
   return millis():tofloat() * 0.001
end

-- Type conversion functions
function get_uint16(frame, ofs)
   return frame:data(ofs) + (frame:data(ofs+1)<<8)
end

function get_uint32(frame, ofs)
   return uint32_t(frame:data(ofs) + (frame:data(ofs+1)<<8) + (frame:data(ofs+2)<<16) + (frame:data(ofs+3)<<32))
end

function get_int32(frame, ofs)
   return get_uint32(frame,ofs).toint()
end

function get_int16(frame, ofs)
   local ret = uint32_t(get_uint16(frame, ofs))

   return ret.toint()
end

function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

---- END: GENERIC FUNCTIONS


---- GLOBAL VARIABLES
local GKWH_TO_LBS_HP_HR = 0.0016439868
local LITRES_TO_LBS = 1.6095    -- 6.1 lbs of fuel per gallon -> 1.6095

-- SkyPower EFI Telemetry Messages
local ECU_FRM_00B = uint32_t(0x00B)    -- hidden throttle Response to a command from a CAN ID message 0x00A 


-- Register for the CAN drivers
local driver1 = CAN.get_device(25)

if not driver1 then
   gcs:send_text(0,string.format("EFI CAN Telemetry: Failed to load drivers"))
   return
end

-- Setup EFI Parameters
assert(param:add_table(EFI_PARAM_TABLE.KEY, EFI_PARAM_TABLE.PREFIX, 5), 'could not add EFI param table')

-- local EFI_TELEM_RATE   = bind_add_param(EFI_PARAM_TABLE, 'TELEM_RATE',     1, 4)     --
-- local EFI_LOG_RATE     = bind_add_param(EFI_PARAM_TABLE, 'LOG_RATE',       2, 25)


local EFI_SIM_TEL_HZ   = bind_add_param(EFI1_PARAM_TABLE, 'SIM_TEL_HZ',    1, 50)     -- Simulated Telemetry Frequency 


---- END: GLOBAL VARIABLES

-- Simulate the EFI State and output it over the setup CAN driver
local efi_sim_state = {}
function generate_telemetry()

   -- Generate Simulated States
   local time = get_time_sec()
   local update_rate = 10.0

   efi_sim_state.rpm = uint32_t(1500.0 * math.sin((time / update_rate) * (math.pi/2.0)) + 2000.0)

   efi_sim_state.atmospheric_pressure_kpa = 15000.0 * math.sin((time / update_rate) * 5) + 20000.0
   efi_sim_state.spark_dwell_time_ms = 15000.0 * math.sin((time / update_rate) * 2 * math.pi) + 20000.0

end


local last_time = 0.0
function update()


   -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_1_state = _efi_state()
   efi_1_state:engine_speed_rpm(efi_sim_state.rpm)
   efi_1_state:atmospheric_pressure_kpa(efi_sim_state.atmospheric_pressure_kpa)
   efi_1_state:spark_dwell_time_ms(efi_sim_state.spark_dwell_time_ms)

   last_time = millis()
   efi_1_state:last_updated_ms(last_time)

   -- Set the EFI_State
   efi:handle_scripting(efi_1_state)
end

gcs:send_text(0,string.format("EFI_driver: loaded"))

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000/EFI1_UPDATE_HZ:get()
end

-- start running update loop
return protected_wrapper()