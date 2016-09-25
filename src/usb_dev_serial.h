#include <stdint.h>

//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND			100
#define SYSTICK_PERIOD_MS				(1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Defines required to redirect UART0 via USB.
//
//*****************************************************************************
#define USB_UART_BASE           UART0_BASE
#define USB_UART_PERIPH         SYSCTL_PERIPH_UART0
#define USB_UART_INT            INT_UART0

//*****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//*****************************************************************************
#define DEFAULT_BIT_RATE        115200
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE)

//*****************************************************************************
//
// Defines required to redirect UART0 via USB.
//
//*****************************************************************************
#define TX_GPIO_BASE            GPIO_PORTA_BASE
#define TX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define TX_GPIO_PIN             GPIO_PIN_1

#define RX_GPIO_BASE            GPIO_PORTA_BASE
#define RX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define RX_GPIO_PIN             GPIO_PIN_0


//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
extern volatile uint32_t g_ui32Flags;
extern char *g_pcStatus;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
extern volatile uint32_t g_ui32UARTTxCount;
extern volatile uint32_t g_ui32UARTRxCount;


#define USB_BUFFER_SIZE	256
extern uint8_t usb_recieve_buffer[USB_BUFFER_SIZE];

extern uint16_t get_USB_CDC_Data(uint8_t *buffer);
extern void			send_USB_CDC_Data(uint8_t *buffer);
