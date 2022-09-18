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

function show_frame(dnum, frame)
    gcs:send_text(0,string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
end
---- END: GENERIC FUNCTIONS


---- GLOBAL VARIABLES
local GKWH_TO_LBS_HP_HR = 0.0016439868
local LITRES_TO_LBS = 1.6095    -- 6.1 lbs of fuel per gallon -> 1.6095

-- SkyPower EFI Telemetry Messages
local EFI_FRM_00B = uint32_t(0x00B)    -- hidden throttle Response to a command from a CAN ID message 0x00A 


-- Register for the CAN drivers
local driver1 = CAN.get_device(25)

if not driver1 then
    gcs:send_text(0,string.format("EFI CAN Telemetry: Failed to load drivers"))
    return
end

-- Setup EFI Parameters
assert(param:add_table(EFI_PARAM_TABLE.KEY, EFI_PARAM_TABLE.PREFIX, 5), 'could not add EFI param table')

local EFI_SIM_TEL_HZ   = bind_add_param(EFI_PARAM_TABLE, 'SIM_TEL_HZ',    1, 50)     -- Simulated Telemetry Frequency 
local EFI_ESC_IDX      = bind_add_param(EFI_PARAM_TABLE, 'ESC_IDX',       2, 1)      -- ESC Index to be used by the EFI engine for RPM feedback

-- local EFI_TELEM_RATE   = bind_add_param(EFI_PARAM_TABLE, 'TELEM_RATE',     1, 4)     --
-- local EFI_LOG_RATE     = bind_add_param(EFI_PARAM_TABLE, 'LOG_RATE',       2, 25)



assert(EFI_ESC_IDX:get() < 1, 'EFI_ESC_IDX: must be greater 1 or greater')


---- END: GLOBAL VARIABLES



--[[
   EFI Telemetry
--]]
local function engine_control(_driver, _idx)
    local self = {}

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
    local gen_amps = 0
    local gen_rpm = 0
    local gen_batt_current = 0
    local egt1 = 0
    local egt2 = 0
    local cht1 = 0
    local cht2 = 0
    local inj_ang = 0
    local inj_time = 0
    local target_load = 0
    local current_load = 0
    local throttle_angle = 0
    local sfc = 0
    local sfc_icao = 0
    local last_sfc_t = 0
    local total_fuel_used_lbs = 0
    local driver = _driver
    local idx = _idx

    -- Generator Data Structure
    local gen = {}
    gen.amps = 0.0
    gen.rpm  = 0.0
    gen.batt_current = 0.0

    -- Temperatures Data Structure
    local temps = {}
    temps.egt1 = 0.0
    temps.egt2 = 0.0
    temps.cht1 = 0.0
    temps.cht2 = 0.0

    -- read telemetry packets
    function self.update_telemetry()
        local max_packets = 25
        local count = 0
        while count < max_packets do
            frame = driver:read_frame()
            count = count + 1
            if not frame then
            break
            end

            -- All Frame IDs for this EFI Engine are in the 11-bit address space
            local EFI_id = frame:id()
            if EFI_id < 2048 then
            self.handle_EFI_packet(frame, _idx)
            end
        end
    end


   -- handle an EFI packet
   function self.handle_EFI_packet(frame, idx)
        local id = frame:id()
        if id == uint32_t(0x100) then
            rpm = get_uint16(frame, 0)
            throttle_angle = get_uint16(frame, 4)
            -- esc_telem:update_rpm(idx - 1, math.floor(rpm * 0.5), 0)    -- ESC::update_rpm() is indexed from 0
        elseif id == 0x101 then
            air_pressure = get_uint16(frame, 4)
        elseif id == 0x102 then
            inj_ang = get_uint16(frame, 2) * 0.1
            inj_time = get_uint16(frame, 4) * 0.1
        elseif id == 0x105 then
            temps.cht1 = get_uint16(frame, 0) * 0.1
            temps.egt1 = get_uint16(frame, 4) * 0.1
        elseif id == 0x113 then
            gen.amps = get_uint16(frame, 2) * 0.01
        elseif id == 0x114 then
            gen.rpm = get_uint16(frame, 0)
        elseif id == 0x115 then
            gen.batt_current = get_uint16(frame, 4) * 0.01
        elseif id == 0x10A then
            target_load = get_uint16(frame, 6) * 0.1
        elseif id == 0x10D then
            current_load = get_uint16(frame, 2) * 0.1
        elseif id == 0x10C then
            temps.cht2 = get_uint16(frame, 4) * 0.1
            temps.egt2 = get_uint16(frame, 6) * 0.1
        elseif id == 0x10F then
            sfc = get_uint16(frame, 6) * GKWH_TO_LBS_HP_HR
            sfc_icao = get_uint16(frame, 4) * 0.001         -- Remove Multiplier of 1000
            local now = get_time_sec()
            local dt = now - last_sfc_t
            last_sfc_t = now
            if dt > 1.0 then
            dt = 0.0
            end
            total_fuel_used_lbs = total_fuel_used_lbs + dt * (sfc_icao/3600.0) * LITRES_TO_LBS
        end
   end

    function self.send_telemetry()
        -- gcs:send_named_float(string.format('RPM%u', idx), rpm)
        -- gcs:send_named_float(string.format('GEN_RPM%u', idx), gen_rpm)
    --   gcs:send_named_float(string.format('GEN_AMPS%u', idx), gen_amps)
        gcs:send_named_float(string.format('GEN_BATT_CURR%u', idx), gen_batt_current)
        gcs:send_named_float(string.format('EGT%u', idx), 0.5*(egt1+egt2))
        gcs:send_named_float(string.format('CHT%u', idx), 0.5*(cht1+cht2))
        -- gcs:send_named_float(string.format('SFC%u', idx), sfc)
        -- gcs:send_named_float(string.format('TFC%u', idx), total_fuel_used_lbs)
        return total_fuel_used_lbs
    end

    -- log the controller internals
    function self.log_telemetry(name)
        logger.write('EFI','I,RPM,ET1,ET2,CT1,CT2,SFC,TFC,InjA,InjT,TLd,CLd,TA','Bffffffffffff','#------------','-------------',
                    idx, rpm, egt1, egt2, cht1, cht2, sfc, total_fuel_used_lbs,
                    inj_ang, inj_time, target_load, current_load, throttle_angle)

        -- Generator Logger Fields
        logger.write('EFIG', 'I,Amp,BatA,RPM', 'Bfff','#---','----', idx, gen_amps, gen_batt_current, gen_rpm)
    end

    -- return the instance
    return self
end      -- end function engine_control(_driver, _idx)








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

    -- Set the ESC RPM State for Notch Filtering
    esc_telem:update_rpm(EFI_ESC_IDX - 1, math.floor(rpm), 0)    -- ESC::update_rpm() is indexed from 0

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