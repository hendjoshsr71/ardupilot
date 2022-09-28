--[[
   simulator for CAN EFI
   this can be used with a loopback cable between CAN1 and CAN2 to test CAN EFI Drivers
--]]

local driver2 = CAN.get_device2(25)
if not driver2 then
   gcs:send_text(0,string.format("EFISIM: Failed to load CAN driver"))
   return
end

local PARAM_TABLE_KEY = 13
local PARAM_TABLE_PREFIX = "EFISIM_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not add param table')

local EFISIM_TYPE  = bind_add_param('TYPE', 1, 0)  -- Simulated Sensor Type: 1: Skypower EFI, 2: MGM ESCs

function get_time_sec()
   return millis():tofloat() * 0.001
end

local FRM_100 = uint32_t(0x100)
local FRM_101 = uint32_t(0x101)
local FRM_102 = uint32_t(0x102)
local FRM_104 = uint32_t(0x104)
local FRM_105 = uint32_t(0x105)
local FRM_106 = uint32_t(0x106)

function put_u8(msg, ofs, v)
   msg:data(ofs,v&0xFF)
end

function put_u16(msg, ofs, v)
   msg:data(ofs,v&0xFF)
   msg:data(ofs+1,v>>8)
end

function put_u32(msg, ofs, v)
   msg:data(ofs+0,v&0xFF)
   msg:data(ofs+1,(v>>8)&0xFF)
   msg:data(ofs+2,(v>>16)&0xFF)
   msg:data(ofs+3,(v>>24)&0xFF)
end

local rev_counter = 0

--[[
   send SkyPower data. Called at 100Hz
--]]
function send_SkyPower(driver)

   --local msg = CANFrame()
   local t = get_time_sec()

   local RPM = 1200 + math.floor(1000*math.sin(t))
   rev_counter = rev_counter + (RPM/60.0)*0.01

   -- 0x100
   local msg = CANFrame()
   msg:id(FRM_100)
   put_u16(msg,0,RPM)
   put_u16(msg,2,13*10) -- ignition angle
   put_u16(msg,4,45*10) -- throttle angle
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x101
   msg = CANFrame()
   msg:id(FRM_101)
   put_u16(msg,2,917) -- air pressure
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x102
   msg = CANFrame()
   msg:id(FRM_102)
   put_u16(msg,0,7*10) -- ingition gap
   put_u16(msg,2,270*10) -- injection angle
   put_u16(msg,4,37000) -- injection time
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x104
   msg = CANFrame()
   msg:id(FRM_104)
   put_u16(msg,0,math.floor(14.8*10)) -- supply voltage
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x105
   msg = CANFrame()
   msg:id(FRM_105)
   put_u16(msg,0,172*10) -- engine temp head 1
   put_u16(msg,2,65*10) -- air temp
   put_u16(msg,4,320*10) -- exhaust temp
   put_u16(msg,6,113*10) -- ecu temp
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x106
   msg = CANFrame()
   msg:id(FRM_106)
   put_u32(msg,0,math.floor(rev_counter))
   put_u8(msg,4,math.floor(t))
   put_u8(msg,5,math.floor(t/60))
   put_u16(msg,6,math.floor(t/3600))
   msg:dlc(8)
   driver:write_frame(msg, 10000)
end

--[[]
   MGM ESCs

--]]
local MGM = {}
MGM.ESC_NUM = 2
MGM.CAN_EXTENDED_BIT = 0x80000000   -- Set the 31st Bit for Extended CAN
MGM.CAN_EXTENDED_BASE = 0x14A30000
MGM.CAN1_OFFSET        = 0x00
MGM.CAN2_OFFSET        = 0x14    -- aka 20


-- Pre-compute the frame ID for all commands for each ESC due to the slowness of creating uint32_t's as they are objects
-- Frame IDs go form 0x00 to 0x07 with the Offsets added onto the frame IDs
ESC_FRM_IDS = {{0, 1,2,3,4,5,6,7},{0,1,2,3,4,5,6,7}}
function create_can_frame_table()
   local offset_table = {MGM.CAN1_OFFSET, MGM.CAN2_OFFSET}
   for i = 1, MGM.ESC_NUM do
      for j = 1, 8 do
         ESC_FRM_IDS[i][j] = uint32_t((j - 1) + offset_table[i] + MGM.CAN_EXTENDED_BASE + MGM.CAN_EXTENDED_BIT)
         -- ESC_FRM_IDS[i][j] = uint32_t((j - 1) + 20)
         -- ESC_FRM_IDS[i][j] = 0x01
      end
   end
end
create_can_frame_table()

function send_MGM_ESC(driver, esc_idx)

   local t = get_time_sec()

   local RPM = 1200 + math.floor(1000*math.sin(t))
   rev_counter = rev_counter + (RPM/60.0)*0.01

   -- BASE + 0x01
   local msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][2])

   -- Try just directly setting the ID HERE
   -- msg:id(0x14A30001)
   


   put_u16(msg, 0, 112 * 10)      -- Voltage (V), Scale: 0.1V
   put_u16(msg, 2, 12 * 10)                     -- Current (A), Scale: 0.1A
   put_u32(msg, 4, RPM)                         -- RPM, Scale: 1 RPM

   -- FIX INTS
   -- put_i16(msg, 2, 12 * 10)                     -- Current (A), Scale: 0.1A
   -- put_i32(msg, 4, RPM)                         -- RPM, Scale: 1 RPM

   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x02
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][3])

   put_u16(msg, 0, 32)                           -- Motor Temperature, Scale: 1 deg C
   put_u16(msg, 2, 35)                           -- ESC Temperature, Scale: 1 deg C
   -- BYTE 4 & 5 Reserved
   put_u16(msg, 6, 42)                           -- External Sensor Temperature, Scale: 1 deg C

   -- FIX INTS
   -- put_i16(msg, 0, 32)                           -- Motor Temperature, Scale: 1 deg C
   -- put_i16(msg, 2, 35)                           -- ESC Temperature, Scale: 1 deg C
   -- -- BYTE 4 & 5 Reserved
   -- put_i16(msg, 6, 42)                           -- External Sensor Temperature, Scale: 1 deg C

   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x03
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][4])
   put_u16(msg, 0, 32)                           -- Output PWM, Scale: 
   put_u16(msg, 2, 35)                           -- Input PWM, Scale: 

-- FIX INTS
   -- put_i16(msg, 0, 32)                           -- Output PWM, Scale: 
   -- put_i16(msg, 2, 35)                           -- Input PWM, Scale: 

   -- BYTE 4, 5, 6, 7: Reserved
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x04
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][5])
   put_u32(msg, 0, 0x80030002)                           -- ERROR BITMASK
   put_u32(msg, 4, 0x09040003)                           -- WARNING BITMASK
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x05
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][6])
   put_u32(msg, 0, 0x80030002)                           -- Notice BITMASK
   -- BYTE 4 & 5 Reserved
   put_u16(msg, 6, 0x1245)                           -- ESC init status
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x06
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][7])
   put_u16(msg, 0, 57.2 * 10)       -- Battery Internal Voltage (V), Scale: 0.1V
   put_u16(msg, 2, 64.5 * 1000)     -- External Feeding Voltage (V), Scale: 0.001V
   put_u16(msg, 4, 52)              -- Phase Current Maximum (A), Scale: 1A
   put_u16(msg, 6, 25)              -- Phase Current Average (A), Scale: 1A
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- BASE + 0x07
   msg = CANFrame()
   msg:id(ESC_FRM_IDS[esc_idx][8])
   -- BYTE 0, 1, 2, 3, 4, 5: Reserved
   put_u8(msg, 6, 5.6 * 10)        -- Internal Bus Voltage (V), Scale: 0.1V
   put_u8(msg, 7, 85)              -- MOSFET Temperature (deg C), Scale: 1 deg C
   msg:dlc(8)
   driver:write_frame(msg, 10000)

end


function update()
   -- local sim_type = math.floor(EFISIM_TYPE:get())
   local sim_type = 2
   if sim_type == 1 then
      send_SkyPower(driver2)
   elseif sim_type == 2 then
      -- send_MGM_ESC(driver2,1)
      send_MGM_ESC(driver2,2)
   end

   return update, 10
end

gcs:send_text(0,string.format("EFISIM: loaded"))

return update()
