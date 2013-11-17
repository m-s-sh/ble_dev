#include "stdbool.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_gpio.h"
#include "app_uart.h"
#include <app_fifo.h>
#include "ble_dev_board_profile.h"



#ifdef MK_TRACE_UART


#ifndef BLE_DEV_BOARD_UART_TX_PIN
        #error "Missing definition of BLE_DEV_BOARD_UART_TX_PIN!        Example:        #define BLE_DEV_BOARD_UART_TX_PIN       6"
#endif
#ifndef BLE_DEV_BOARD_UART_RX_PIN
        #error "Missing definition of BLE_DEV_BOARD_UART_RX_PIN!        Example:        #define BLE_DEV_BOARD_UART_RX_PIN       7"
#endif

#define FIFO_LENGTH(F)             (F->write_pos - F->read_pos)               /**< Macro to calculate length of a FIFO. */

static app_fifo_t       mk_rx_fifo;     /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */
static app_fifo_t       mk_tx_fifo;     /**< TX FIFO buffer for storing data to be transmitted on the UART when TXD is ready. Data is put to the buffer on using app_uart_put(). */


/** @brief States for the app_uart state machine. */
typedef enum {
        UART_OFF,               /**< app_uart state OFF, indicating CTS is low. */
        UART_READY,             /**< app_uart state ON, indicating CTS is high. */
        UART_ON,                /**< app_uart state TX, indicating UART is ongoing transmitting data. */
        UART_WAIT_CLOSE,        /**< app_uart state WAIT CLOSE, indicating that CTS is low, but a byte is currently being transmitted on the line. */
} app_uart_states_t;

static volatile app_uart_states_t  m_current_state = UART_OFF;              /**< State of the state machine. */


uint32_t mk_trace(const char* format, ...)
{
        char tmp_buff[64];
        uint32_t ret_value = NRF_SUCCESS;
        int32_t i = 0;

        va_list ap;
        va_start(ap, format);
        int32_t nLength = vsnprintf(tmp_buff, 64, format, ap);
        va_end(ap);

        for(i=0; i<nLength; i++) {
                ret_value = app_fifo_put(&mk_tx_fifo, tmp_buff[i]);
                if(NRF_SUCCESS != ret_value)
                        break;
        }

        if((i != 0) && (UART_READY == m_current_state)) {
                uint8_t ch;
                ret_value = app_fifo_get(&mk_tx_fifo, &ch);
                if(NRF_SUCCESS == ret_value)
                {
                        m_current_state = UART_ON;
                        NRF_UART0->TXD = ch;
                }
        }
        return ret_value;
}



int fgetc(FILE *f) {
        uint8_t ch;
        uint32_t ret_value;
        do {
                ret_value = app_fifo_get(&mk_tx_fifo, &ch);
        } while(ret_value != NRF_SUCCESS);
        return(ch);
}


uint32_t mk_trace_init(uint32_t tx_buf_size, uint32_t rx_buf_size)
{
        uint32_t err_code;
        ASSERT(tx_buf_size != 0);
        ASSERT(rx_buf_size != 0);

        // Configure buffer RX buffer.
        uint8_t *rx_buf = malloc(rx_buf_size);
        err_code = app_fifo_init(&mk_rx_fifo, rx_buf, rx_buf_size);
        if (err_code != NRF_SUCCESS) {
                // Propagate error code.
                return err_code;
        }

        // Configure buffer TX buffer.
        uint8_t *tx_buf = malloc(tx_buf_size);
        err_code = app_fifo_init(&mk_tx_fifo, tx_buf, tx_buf_size);
        if (err_code != NRF_SUCCESS) {
                // Propagate error code.
                return err_code;
        }

    // Configure RX and TX pins.
        //      secure correct signal levels when UART is off
    nrf_gpio_cfg_output(BLE_DEV_BOARD_UART_TX_PIN);
        nrf_gpio_pin_set(BLE_DEV_BOARD_UART_TX_PIN);
    nrf_gpio_cfg_input(BLE_DEV_BOARD_UART_RX_PIN, NRF_GPIO_PIN_NOPULL);
    NRF_UART0->PSELTXD = BLE_DEV_BOARD_UART_TX_PIN;
    NRF_UART0->PSELRXD = BLE_DEV_BOARD_UART_RX_PIN;
    NRF_UART0->PSELRTS = UART_PIN_DISCONNECTED;
    NRF_UART0->PSELCTS = UART_PIN_DISCONNECTED;

    // Configure baud rate and parity.
    NRF_UART0->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud921600 << UART_BAUDRATE_BAUDRATE_Pos);
        NRF_UART0->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos);
    NRF_UART0->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);

        NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
    NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);

        NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;

        m_current_state = UART_READY;

    // Enable UART interrupt
    NRF_UART0->INTENCLR = 0xffffffffUL;
    NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
                          (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
                          (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);

        return NRF_SUCCESS;
}


/**@brief UART Interrupt handler.
 *
 * @details UART interrupt handler to process TX Ready when TXD is available,
 * RX Ready when a byte is received, or in case of error when receiving a byte.
 */
void UART0_IRQHandler(void)
{
    if (NRF_UART0->EVENTS_RXDRDY != 0) {        // Handle reception

                NRF_UART0->EVENTS_RXDRDY = 0;  // Clear UART RX event flag
                // Write received byte to FIFO
                uint32_t err_code = app_fifo_put(&mk_rx_fifo, (uint8_t)NRF_UART0->RXD);
                if (err_code != NRF_SUCCESS)
                        return;
    }

    if (NRF_UART0->EVENTS_TXDRDY != 0) {        // Handle transmission.
                NRF_UART0->EVENTS_TXDRDY = 0;  // Clear UART TX event flag.
                uint8_t ch;
                uint32_t err_code = app_fifo_get(&mk_tx_fifo, &ch);
        if(NRF_SUCCESS == err_code) {
                        NRF_UART0->TXD = ch;
                        m_current_state = UART_ON;
                } else {
                        m_current_state = UART_READY;
                }
    }

    if (NRF_UART0->EVENTS_ERROR != 0){  // Handle errors.
                NRF_UART0->EVENTS_ERROR = 0;  // Clear UART ERROR event flag.
                // Clear error source.
                uint32_t error_source = NRF_UART0->ERRORSRC;
                NRF_UART0->ERRORSRC = error_source;
    }
}




#endif
