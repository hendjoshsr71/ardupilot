--[[ 
Name: EFI Scripting backend driver for SkyPower
Authors: Andrew Tridgell & Josh Henderson

Settings Required
Notes: the Skypower Driver uses addresses only in the CAN 11-bit address space. Thus if you can guarantee other devices 
do not utilize these addresses multiple drivers may use the bus. However, the Skypower protocol

The protocol has no method to identify multiple engines on the same bus.
The protocol has high CAN utilization due to CAN packets that are used internal to the enigne being published externally.
As well as being limited to 500 kbit/s.

CAN_D1_PROTOCOL 10 (Scripting Driver 1)
CAN_P1_DRIVER 1 (First driver)
CAN_D1_BITRATE 500000 (500 kbit/s)

--]]

-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = "4.4"
local SCRIPT_NAME       = "EFI: Skypower CAN"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION > SCRIPT_AP_VERSION, string.format('%s Requires: %s:%s. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))


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

-- Type conversion functions
function get_uint16(frame, ofs)
    return frame:data(ofs) + (frame:data(ofs + 1) << 8)
end

function get_uint32(frame, ofs)
    return uint32_t(frame:data(ofs) + (frame:data(ofs + 1) << 8) + (frame:data(ofs + 2) << 16) + (frame:data(ofs + 3) <<
        32))
end

function get_int32(frame, ofs)
    return get_uint32(frame, ofs).toint()
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
    gcs:send_text(0,
          string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum,
                  frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
end

---- END: GENERIC FUNCTIONS


---- GLOBAL VARIABLES
local GKWH_TO_LBS_HP_HR = 0.0016439868
local LITRES_TO_LBS = 1.6095 -- 6.1 lbs of fuel per gallon -> 1.6095

-- SkyPower EFI Telemetry Messages
local EFI_FRM_00B = uint32_t(0x00B) -- hidden throttle Response to a command from a CAN ID message 0x00A


-- Register for the CAN drivers
local driver1 = CAN.get_device(25)

if not driver1 then
    gcs:send_text(0, string.format("EFI CAN Telemetry: Failed to load drivers"))
    return
end

-- Setup EFI Parameters
assert(param:add_table(EFI1_PARAM_TABLE.KEY, EFI1_PARAM_TABLE.PREFIX, 5), 'could not add EFI1 param table')

local EFI1_UPDATE_HZ    = bind_add_param(EFI1_PARAM_TABLE, 'UPDATE_HZ',        1, 200)    -- Script update frequency
local EFI1_SIM_TEL_HZ   = bind_add_param(EFI1_PARAM_TABLE, 'SIM_TEL_HZ',       2, 50)     -- Simulated Telemetry Frequency
local EFI1_ESC_IDX      = bind_add_param(EFI1_PARAM_TABLE, 'ESC_IDX',          3, 1)      -- ESC Index to be used by the EFI engine for RPM feedback

local EFI1_TELEM_RATE   = bind_add_param(EFI1_PARAM_TABLE, 'TELEM_RATE',       4, 0)      -- Telemetry Rate (Hz): rate to send telemetry via scripting
local EFI1_LOG_RATE     = bind_add_param(EFI1_PARAM_TABLE, 'LOG_RATE',         5, 0)     -- LOG_RATE (Hz): rate to log telemetry via scripting

assert(EFI1_ESC_IDX:get() < 1, 'EFI1_ESC_IDX: must be greater 1 or greater')
---- END: GLOBAL VARIABLES


--[[
   EFI Engine Object


--]]
local function engine_control(_driver, _idx)
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local _efi_state = _efi_state()
    local _cylinder_state = Cylinder_Status()

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
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
    local gen        = {}
    gen.amps         = 0.0
    gen.rpm          = 0.0
    gen.batt_current = 0.0

    -- Temperature Data Structure
    local temps = {}
    temps.egt1 = 0.0        -- Engine Gas Temperature: Sensor 1
    temps.egt2 = 0.0        -- Engine Gas Temperature: Sensor 2
    temps.cht1 = 0.0        -- Cylinder Head Temperature: Sensor 1
    temps.cht2 = 0.0        -- Cylinder Head Temperature: Sensor 2

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
        elseif id == uint32_t(0x101) then
            air_pressure = get_uint16(frame, 4)
        elseif id == uint32_t(0x102) then
            inj_ang = get_uint16(frame, 2) * 0.1
            inj_time = get_uint16(frame, 4) * 0.1
        elseif id == uint32_t(0x105) then
            temps.cht1 = get_uint16(frame, 0) * 0.1
            temps.egt1 = get_uint16(frame, 4) * 0.1
        elseif id == uint32_t(0x113) then
            gen.amps = get_uint16(frame, 2) * 0.01
        elseif id == uint32_t(0x114) then
            gen.rpm = get_uint16(frame, 0)
        elseif id == uint32_t(0x115) then
            gen.batt_current = get_uint16(frame, 4) * 0.01
        elseif id == uint32_t(0x10A) then
            target_load = get_uint16(frame, 6) * 0.1
        elseif id == uint32_t(0x10D) then
            current_load = get_uint16(frame, 2) * 0.1
        elseif id == uint32_t(0x10C) then
            temps.cht2 = get_uint16(frame, 4) * 0.1
            temps.egt2 = get_uint16(frame, 6) * 0.1
        elseif id == uint32_t(0x10F) then
            sfc = get_uint16(frame, 6) * GKWH_TO_LBS_HP_HR
            sfc_icao = get_uint16(frame, 4) * 0.001 -- Remove Multiplier of 1000
            local now = get_time_sec()
            local dt = now - last_sfc_t
            last_sfc_t = now
            if dt > 1.0 then
                dt = 0.0
            end
            total_fuel_used_lbs = total_fuel_used_lbs + dt * (sfc_icao / 3600.0) * LITRES_TO_LBS
        end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_efi_state()
        -- Cylinder_Status
        -- _cylinder_state:ignition_timing_deg(0.0)
        -- _cylinder_state:injection_time_ms(0.0)
        _cylinder_state:cylinder_head_temperature(0.5 * (temps.cht1 + temps.cht2))
        _cylinder_state:exhaust_gas_temperature(0.5 * (temps.eht1 + temps.eht2))
        -- _cylinder_state:lambda_coefficient(0.0)

        -- _efi_state:general_error(0)
        _efi_state:engine_load_percent(current_load)
        _efi_state:engine_speed_rpm(uint32_t(rpm))
        -- _efi_state:spark_dwell_time_ms(0.0)

        -- _efi_state:atmospheric_pressure_kpa(air_pressure)
        -- _efi_state:intake_manifold_pressure_kpa(0.0)
        -- _efi_state:intake_manifold_temperature(0.0)
        -- _efi_state:coolant_temperature(0.0)
        -- _efi_state:oil_pressure(0.0)
        -- _efi_state:fuel_pressure(0.0)
        
        _efi_state:fuel_consumption_rate_cm3pm(0.0)
        _efi_state:estimated_consumed_fuel_volume_cm3(0.0)
        _efi_state:throttle_position_percent(0.0)

        -- This should be set in the C++ ? 
        -- _efi_state:ecu_index(0)



        -- Need to add target_load to the EFI state
        -- _efi_state:engine_taget_load_percent(target_load)

        -- Unimplemented ENUM States
        -- _efi_state:engine_state()



        -- Where should last_updated come from? Here or the last time we read certain fields?
        -- Fro example last_rpm_time_updated_ms?
        last_efi_state_time = millis()
        _efi_state:last_updated_ms(last_efi_state_time)


        -- Set the EFI_State into the EFI scripting driver
        efi:handle_scripting(_efi_state)
    end -- set_efi_state()


    function self.send_telemetry()
        gcs:send_named_float(string.format('RPM%u', idx), rpm)
        -- gcs:send_named_float(string.format('GEN_RPM%u', idx), gen.rpm)
        gcs:send_named_float(string.format('GEN_AMPS%u', idx), gen.amps)
        gcs:send_named_float(string.format('EGT%u', idx), 0.5 * (temps.egt1 + temps.egt2))
        gcs:send_named_float(string.format('CHT%u', idx), 0.5 * (temps.cht1 + temps.cht2))
        -- gcs:send_named_float(string.format('SFC%u', idx), sfc)
        -- gcs:send_named_float(string.format('TFC%u', idx), total_fuel_used_lbs)
    end

    -- log the controller internals
    function self.log_telemetry(name)
        logger.write('EFI', 'I,RPM,ET1,ET2,CT1,CT2,SFC,TFC,InjA,InjT,TLd,CLd,TA', 'Bffffffffffff', '#------------', '-------------',
                    idx, rpm, temps.egt1, temps.egt2, temps.cht1, temps.cht2, sfc, total_fuel_used_lbs,
                    inj_ang, inj_time, target_load, current_load, throttle_angle)

        -- Generator Logger Fields
        logger.write('EFIG', 'I,Amp,BatA,RPM', 'Bfff', '#---', '----', idx, gen.amps, gen.batt_current, gen.rpm)
    end

    -- return the instance
    return self
end -- end function engine_control(_driver, _idx)

-- Simulate the EFI State and output it over the setup CAN driver
local efi_sim_state = {}
function generate_telemetry()

    -- Generate Simulated States
    local time = get_time_sec()
    local update_rate = 10.0

    efi_sim_state.rpm = uint32_t(1500.0 * math.sin((time / update_rate) * (math.pi / 2.0)) + 2000.0)

    efi_sim_state.atmospheric_pressure_kpa = 15000.0 * math.sin((time / update_rate) * 5) + 20000.0
    efi_sim_state.spark_dwell_time_ms = 15000.0 * math.sin((time / update_rate) * 2 * math.pi) + 20000.0

end


local engine1 = engine_control(driver1, 1)

local last_efi_state_time = 0.0

function update()

    -- Parse Driver Messages
    engine1.update_telemetry()

    -- Build up States and send to relevant libraries
    engine1.set_efi_state()

    -- Set the ESC RPM State for Notch Filtering
    esc_telem:update_rpm(EFI1_ESC_IDX - 1, math.floor(rpm), 0) -- ESC::update_rpm() is indexed from 0

    -- Logging and GCS_Telemetry inside of scripting
    local now = get_time_sec()
   if now - last_telem_t > 1.0 / EFI1_TELEM_RATE:get() then
      last_telem_t = now
      engine1.send_telemetry()
   end

   if now - last_log_t > 1.0 / EFI1_LOG_RATE:get() then
      last_log_t = now
      engine1.log_telemetry()
   end

end

gcs:send_text(0, SCRIPT_NAME .. string.format(" loaded"))

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
    return protected_wrapper, 1000 / EFI1_UPDATE_HZ:get()
end

-- start running update loop
return protected_wrapper()