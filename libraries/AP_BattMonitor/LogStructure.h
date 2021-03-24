#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BATTMONITOR \
    LOG_BAT_MSG, \
    LOG_BCL_MSG, \
    LOG_BATI_MSG

// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: current * time
// @Field: EnrgTot: energy this battery has produced
// @Field: Temp: measured temperature
// @Field: Res: estimated battery resistance
struct PACKED log_BAT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    float    voltage_resting;
    float    current_amps;
    float    current_total;
    float    consumed_wh;
    int16_t  temperature; // degrees C * 100
    float    resistance;
};

// @LoggerMessage: BCL
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: battery voltage
// @Field: V1: first cell voltage
// @Field: V2: second cell voltage
// @Field: V3: third cell voltage
// @Field: V4: fourth cell voltage
// @Field: V5: fifth cell voltage
// @Field: V6: sixth cell voltage
// @Field: V7: seventh cell voltage
// @Field: V8: eighth cell voltage
// @Field: V9: ninth cell voltage
// @Field: V10: tenth cell voltage
// @Field: V11: eleventh cell voltage
// @Field: V12: twelfth cell voltage
struct PACKED log_BCL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    uint16_t cell_voltages[12];
};

// @LoggerMessage: BATI
// @Vehicles: All
// @Description: Smart Battery Information
// @Field: TimeUS: Time since system startup
// @Field: id: battery monitor instance number
// @Field: func: battery function
// @Field: type: battery type (chemistry)
// @Field: capD: design capacity when full according to manufacturer, -1: field not provided.
// @Field: capF: capacity when full (accounting for battery degradation), -1: field not provided.
// @Field: cyc: charge/discharge cycle count. UINT16_MAX: field not provided.
// @Field: ser: serial number in ASCII characters, 0 terminated. All 0: field not provided.
// @Field: name: product name encoded as 'device names'_'manufacturer'
struct PACKED log_BATI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint8_t  function;
    uint8_t  type;
    int32_t  capacity_design;
    int32_t  capacity_full;
    uint16_t cycles;
    char     serial_number[16];
    char     product_name[64];
};

#define LOG_STRUCTURE_FROM_BATTMONITOR        \
    { LOG_BAT_MSG, sizeof(log_BAT), \
        "BAT", "QBfffffcf", "TimeUS,Instance,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res", "s#vvAiJOw", "F-000!/?0" },  \
    { LOG_BCL_MSG, sizeof(log_BCL), \
        "BCL", "QBfHHHHHHHHHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12", "s#vvvvvvvvvvvvv", "F-0CCCCCCCCCCCC" }, \
    { LOG_BATI_MSG, sizeof(log_BATI), \
        "BATI", "QBBBiiHNZ", "TimeUS,id,func,type,capD,capF,cyc,ser,name", "s#--ii?--", "F000!!0--" },
