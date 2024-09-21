#define __EXPORT __attribute__ ((visibility ("default")))

#ifndef __cplusplus
#error "C++ should be defined!!!"
#endif

#include <stdint.h>

// Functions that are called by the SLPI LINK server into AP client
extern "C" {
    // Called by the SLPI LINK server to initialize and start AP
    int slpi_link_client_init(void) __EXPORT;

    // Called by the SLPI LINK server when there is a new message for AP
    int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes) __EXPORT;
}

// Functions in SLPI LINK server that are called by AP client into DSP
extern "C" {
    // Send a message to the applications processor
    int sl_client_send_data(const uint8_t *data, int data_len_in_bytes);

    void sl_client_register_fatal_error_cb(void (*func)(void));

    // Interrupt callback registration
    int sl_client_register_interrupt_callback(int (*func)(int, void*, void*), void* arg);

    // Get DSP CPU utilization (0 - 100)
    int sl_client_get_cpu_utilization(void);

    // I2C interface API
    int sl_client_config_i2c_bus(uint8_t bus_number, uint8_t address, uint32_t frequency);
    void sl_client_set_address_i2c_bus(int fd, uint8_t address);
    int sl_client_i2c_transfer(int fd, const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);

    // SPI interface API
    int sl_client_spi_transfer(int fd, const uint8_t *send, uint8_t *recv, const unsigned len);
    int sl_client_config_spi_bus(void);

    // UART interface API
    int sl_client_config_uart(uint8_t port_number, uint32_t speed);
    int sl_client_uart_write(int fd, const char *data, const unsigned data_len);
    int sl_client_uart_read(int fd, char *buffer, const unsigned buffer_len);
}

// IDs for serial ports
// See here for documentation on mapping of pins: https://docs.modalai.com/voxl2-connectors/
#define QURT_UART_GPS 6         // FIXME Connector J19 : SSC_QUP6              :  /dev/ttyHS????? :  PIN #2 (TX)  : PIN #3 RX
#define QURT_UART_RCIN 7        // FIXME:  Connector J19 : SSC_QUP7 :  /dev/ttyHS??? : PIN #10 (TX)  : PIN #11 (RX) 
#define QURT_UART_ESC 2         // FIXME Connector J18  : SSC_QUP_2         :  /dev/ttyHS???? :  PIN #2 (TX)  : PIN #3 RX

//  FIXME NOT SURE  which is RX vs TX Connector J5 : {????} :  /dev/ttyHS2 : PIN #B18 (RX)  GPIO_2_QUP19_L2  : PIN #B19 TX GPIO_3_QUP19_L3 


// APPS Processor UARTS NOT AVAILABLE YET IN AP
// Connector J8  : GPIO_14_CAM2_UART5    :  /dev/ttyHS0 :  PIN #38 (TX) : PIN #40 RX
// #define QURT_UART_OSD 5  Connector J3  : GPIO_23_UART7         :  /dev/ttyHS1 :  PIN #3 (RX)  : PIN #5 TX
// Connector J3 : DBG_UART12 :  /dev/tty??? : PIN #27 (GPIO_35_DBG_UART12_RX)  : PIN #29 (GPIO_34_DBG_UART12_TX) 
// Connector J5 : UART13 includes CTS & RTS :  /dev/tty??? : PIN #D5 (GPIO_36_UART13_CTS)  : PIN #D6 (GPIO_37_UART13_RTS) : PIN #D7 (GPIO_38_UART13_TXD) : PIN #D8 (GPIO_39_UART13_RXD) 
