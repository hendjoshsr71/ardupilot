#include "AP_BattMonitor_Backend.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// Write BAT data packet(s)
void AP_BattMonitor_Backend::Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const
{
    bool has_curr = has_current();

    const struct log_BAT pkt{
        LOG_PACKET_HEADER_INIT(LOG_BAT_MSG),
        time_us             : time_us,
        instance            : instance,
        voltage             : _state.voltage,
        voltage_resting     : _state.voltage_resting_estimate,
        current_amps        : has_curr ? _state.current_amps : AP::logger().quiet_nanf(),
        current_total       : has_curr ? _state.consumed_mah : AP::logger().quiet_nanf(),
        consumed_wh         : has_curr ? _state.consumed_wh : AP::logger().quiet_nanf(),
        temperature         : (int16_t) ( has_temperature() ? _state.temperature * 100 : 0),
        resistance          : _state.resistance
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write BCL data packet if has_cell_voltages
void AP_BattMonitor_Backend::Log_Write_BCL(const uint8_t instance, const uint64_t time_us) const
{
    if (!has_cell_voltages()) {
        return;
    }
    
    struct log_BCL cell_pkt{
        LOG_PACKET_HEADER_INIT(LOG_BCL_MSG),
        time_us             : time_us,
        instance            : instance,
        voltage             : _state.voltage
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(_state.cell_voltages.cells); i++) {
        cell_pkt.cell_voltages[i] = _state.cell_voltages.cells[i] + 1;
    }
    AP::logger().WriteBlock(&cell_pkt, sizeof(cell_pkt));

    // check battery structure can hold all cells  
    static_assert(ARRAY_SIZE(_state.cell_voltages.cells) == ARRAY_SIZE(cell_pkt.cell_voltages),
                    "Battery cell number doesn't match in library and log structure");
}


void AP_BattMonitor_Backend::Log_Write_BATI(uint8_t instance) const
{
#if SMART_BATTMON_ENABLED  // SMART_BATT_LOGGING_ENABELED
    uint16_t cycles;
    const bool got_cycle_count = get_cycle_count(cycles);

    int32_t capacity_design;
    const bool got_capacity_design = get_design_capacity(capacity_design);

    int32_t capacity_full;
    const bool got_capacity_full = get_full_charge_capacity(capacity_full);

    char serial_number[MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN]{};
    get_serial_number(serial_number, ARRAY_SIZE(serial_number));

    char product_name[MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN]{};
    get_product_name(product_name, ARRAY_SIZE(product_name));

    struct log_BATI pkt{
        LOG_PACKET_HEADER_INIT(LOG_BATI_MSG),
        time_us             : AP_HAL::micros64(),
        instance            : instance,
        function            : MAV_BATTERY_FUNCTION_UNKNOWN,
        type                : MAV_BATTERY_TYPE_UNKNOWN,
        capacity_design     : got_capacity_design ? capacity_design : -1,
        capacity_full       : got_capacity_full ? capacity_full : -1,
        cycles              : (uint16_t) (got_cycle_count ? cycles : UINT16_MAX),
        serial_number       : {},
        product_name        : {}
    };

    strncpy_noterm(pkt.serial_number, serial_number, sizeof(pkt.serial_number));
    strncpy_noterm(pkt.product_name, product_name, sizeof(pkt.product_name));
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}
