---- GLOBAL USER PARAMETERS
local MAV_SEVERITY_ERROR = 3

local EFI1_PARAM_TABLE = {}
EFI1_PARAM_TABLE.KEY = 35
EFI1_PARAM_TABLE.PREFIX = "EFI1_"

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

----  END: GENERIC FUNCTIONS

assert(param:add_table(EFI1_PARAM_TABLE.KEY, EFI1_PARAM_TABLE.PREFIX, 10), 'could not add param table')

local EFI1_UPDATE_HZ    = bind_add_param(EFI1_PARAM_TABLE, 'UPDATE_HZ',    1, 50)


local last_time = 0.0
function update()

   -- Generate Simulated States
   local time = get_time_sec()
   local update_rate = 10.0

   local rpm = uint32_t(1500.0 * math.sin((time / update_rate) * (math.pi/2.0)) + 2000.0)

   local atmospheric_pressure_kpa = 15000.0 * math.sin((time / update_rate) * 5) + 20000.0
   local spark_dwell_time_ms = 15000.0 * math.sin((time / update_rate) * 2 * math.pi) + 20000.0

   -- Build up the EFI_State
    local efi_1_state = _efi_state()
   efi_1_state:engine_speed_rpm(rpm)
   efi_1_state:atmospheric_pressure_kpa(atmospheric_pressure_kpa)
   efi_1_state:spark_dwell_time_ms(spark_dwell_time_ms)

   last_time = millis()
   efi_1_state:last_updated_ms(last_time)

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