#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "hw_gpio.h"
#include "timer.h"
#include "interrupt.h"
#include "pin_map.h"
#include "uart.h"
#include "i2c.h"
#include "systick.h"
#include "pwm.h"

//#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
//#include "utils/ustdlib.h"
#include "usb_serial_structs.h"
#include "utils/uartstdio.h"
#include "usb_dev_serial.h"

#include "defines.h"
#include "config.h"

void PeripheralClock_Config(void) {
	/// Config PLL for 80MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // 1.25 MHz
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  
	SysCtlPeripheralEnable(USB_UART_PERIPH);//SYSCTL_PERIPH_UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
}

void GPIO_Config(void) {
	/// Board LEDs
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	
	/// Board Buttons
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
	GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
	
	/// Configure the required pins for USB operation.
	GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
	
	/// USB: Enable and configure the UART RX and TX pins
	GPIOPinTypeUART(TX_GPIO_BASE, TX_GPIO_PIN);
	GPIOPinTypeUART(RX_GPIO_BASE, RX_GPIO_PIN);
	
	/// UART for bluetooth
	// rx/tx
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// state
	GPIOPinTypeQEI(GPIO_PORTB_BASE, GPIO_PIN_4);
	
	/// I2C for 9DOF sensor
	GPIOPinConfigure(GPIO_PE4_I2C2SCL);
	GPIOPinConfigure(GPIO_PE5_I2C2SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
	
	/// PWM for motors
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinConfigure(GPIO_PA7_M1PWM3);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
}

void Timers_Config(void) {

	// Timer 1 - System
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() / 100) - 1); // 100 Hz
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER1_BASE, TIMER_A);

	// Timer 2
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER2_BASE, TIMER_A, (SysCtlClockGet() / 600) - 1); // 600 Hz
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER2_BASE, TIMER_A);
	
}

void PWM_Config(void) {
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_GEN_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, (SysCtlPWMClockGet() / 50)); // 50 Hz
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (SysCtlPWMClockGet() / 1000)); // 1 mS
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, (SysCtlPWMClockGet() / 1000)); // 1 mS
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_GEN_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, (SysCtlPWMClockGet() / 50)); // 50 Hz
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, (SysCtlPWMClockGet() / 1000)); // 1 mS
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, (SysCtlPWMClockGet() / 1000)); // 1 mS
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void SysTick_Config(void) {
	SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);
	SysTickIntEnable();
	SysTickEnable();
}

void UART_Config(void) {
	
	/// UART0 for USB
	// Set the default UART configuration.
	UARTConfigSetExpClk(USB_UART_BASE, SysCtlClockGet(), DEFAULT_BIT_RATE, DEFAULT_UART_CONFIG);
	UARTFIFOLevelSet(USB_UART_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
	// Configure and enable UART interrupts.
	UARTIntClear(USB_UART_BASE, UARTIntStatus(USB_UART_BASE, false));
	UARTIntEnable(USB_UART_BASE, (UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_RT | UART_INT_TX | UART_INT_RX));
	
	/// UART1 for Bluetooth
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
	UARTFIFODisable(UART1_BASE);
	UARTIntEnable(UART1_BASE, UART_INT_RX);
	//UARTEnable(UART1_BASE);
}

void USB_Config(void) {
	
  /// Enable lazy stacking for interrupt handlers.  This allows floating-point
  /// instructions to be used within interrupt handlers, but at the expense of
  /// extra stack usage.
	//ROM_FPULazyStackingEnable();
	
	/// Initialize the transmit and receive buffers.
	USBBufferInit(&g_sTxBuffer);
	USBBufferInit(&g_sRxBuffer);

	/// Set the USB stack mode to Device mode with VBUS monitoring.
	USBStackModeSet(0, eUSBModeForceDevice, 0);

	/// Pass our device information to the USB library and place the device on the bus.
	USBDCDCInit(0, &g_sCDCDevice);
}

void I2C_Config(void) {
	I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true); // false = 100kbs, true = 400kbs
}

void NVIC_Config(void) {
	IntMasterEnable();
	
	/// Timer 1
	IntEnable(INT_TIMER1A);
	
	/// Timer 2
	IntEnable(INT_TIMER2A);
	
	/// USB interrupt
	IntEnable(USB_UART_INT);
	
	/// UART
	IntPrioritySet(INT_UART1, 0);
	IntEnable(INT_UART1);
}
