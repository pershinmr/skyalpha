#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "timer.h"
#include "interrupt.h"
#include "uart.h"
#include "fpu.h"
#include "pwm.h"
#include "uart.h"

#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "usb_dev_serial.h"

#include "defines.h"
#include "config.h"
#include "var.h"

#include "adxl345.h"
#include "hmc5883l.h"
#include "itg3200.h"

#include "kalman.h"


#define	__MEASURE_ACCELEROMETER         0
#define __MEASURE_COMPASS						1
#define __MEASURE_GYROSCOPE					2

#define __COMPASS_X_OFFSET					-82.0f
#define __COMPASS_Y_OFFSET					136.5f
#define __COMPASS_Z_OFFSET					-283.0f

#define __KP		0.01f
#define __KD		0.01f
#define __KI		0.001f

#define __TORQUE_MAX		100

enum {
	__BT_STANDBY,
	__BT_RECEIVE_TORQUE
};


uint32_t			pwm_msec;

uint16_t			user_torque;
uint16_t			torque[4];
Vect3d				accel, gyro, compass;
Vect3d				gyro_prev;
uint8_t				sensors_state;
float					roll, pitch, yaw;
float					roll_des, pitch_des, yaw_des;
float					u_roll, u_pitch, u_yaw;
kalman_data		k_roll, k_pitch, k_yaw;

uint8_t				bt_reciever_state;
uint8_t				bt_byte;


void TIMER1A_Handler(void) {
	uint16_t	i;
	uint8_t		usb_data[128];
	uint16_t	data_len;
	float			acc_roll, acc_pitch, acc_yaw;
	float			yaw_x, yaw_y;
	float			compass_x, compass_y, compass_z;
	float			roll_err, pitch_err, yaw_err;
	
	
	acc_pitch =-((atan2f(accel.x, -accel.z)*180)/3.14159f);
	acc_roll = 	((atan2f(accel.y, -accel.z)*180)/3.14159f);
	kalman_innovate(&k_roll,	acc_roll,		gyro.x/14.7f);
	kalman_innovate(&k_pitch,	acc_pitch,	gyro.y/14.7f);
	roll	= k_roll.x[0];
	pitch = k_pitch.x[0];
	
	compass_x = compass.x - __COMPASS_X_OFFSET;
	compass_y = compass.y - __COMPASS_Y_OFFSET;
	compass_z = compass.z - __COMPASS_Z_OFFSET;
	yaw_x = compass_x * cosf(pitch*3.14159f/180) + compass_z * sinf(roll*3.14159f/180) * sinf(pitch*3.14159f/180) + compass_y * cosf(roll*3.14159f/180) * sinf(pitch*3.14159f/180);
	yaw_y = compass_z * cosf(roll*3.14159f/180) - compass_y * sinf(roll*3.14159f/180);
	acc_yaw = -((atan2f(yaw_y, yaw_x)*180)/3.14159f);
	kalman_innovate(&k_yaw,		acc_yaw,		gyro.z/14.7f);
	yaw		=	k_yaw.x[0];
	/*
	sprintf((char*)usb_data, "X:%06i,Y:%06i,Z:%06i\n", (int16_t)(roll*100), (int16_t)(pitch*100), (int16_t)(yaw*100));
	send_USB_CDC_Data(usb_data);
	*/
	
	roll_des = 0;
	pitch_des = 0;
	yaw_des = 0;
	
	roll_err	+= roll_des - roll;
	pitch_err	+= pitch_des - pitch;
	yaw_err		+= yaw_des - yaw;
	
	u_roll =	__KP * (roll_des - roll)		+ __KD * (((roll_des - roll)/_dt)		- gyro.x/14.7f*_dt) + __KI * roll_err;
	u_pitch =	__KP * (pitch_des - pitch)	+ __KD * (((pitch_des - pitch)/_dt)	- gyro.y/14.7f*_dt) + __KI * pitch_err;
	u_yaw =		__KP * (yaw_des - yaw)			+ __KD * (((yaw_des - yaw)/_dt)			- gyro.z/14.7f*_dt) + __KI * yaw_err;
	
	/*		
	sprintf((char*)usb_data, "X:%06i,Y:%06i,Z:%06i\n", (int16_t)(roll*100), (int16_t)(pitch*100), (int16_t)(yaw*100));
	send_USB_CDC_Data(usb_data);
	sprintf((char*)usb_data, "u_roll: %i \t u_pitch: %i \t u_yaw: %i \n", (int32_t)(u_roll*100), (int32_t)(u_pitch*100), (int32_t)(u_yaw*100));
	send_USB_CDC_Data(usb_data);
	*/
	
	if (get_USB_CDC_Data(usb_data)) {
		switch (usb_data[0]) {
			default:
					user_torque = atoi((char *)usb_data);
					sprintf((char*)usb_data, "torque setted: %d\n", user_torque);
					send_USB_CDC_Data(usb_data);
				break;
		}
	}
	
	torque[0] = /*(uint16_t)(- u_pitch + u_roll) +*/ user_torque;
	torque[1] = /*(uint16_t)(+ u_pitch + u_roll) +*/ user_torque;
	torque[2] = /*(uint16_t)(- u_pitch - u_roll) +*/ user_torque;
	torque[3] = /*(uint16_t)(+ u_pitch - u_roll) +*/ user_torque;
	
	for (i = 0; i < 4; i++) {
		if (torque[i] > __TORQUE_MAX) torque[i] = __TORQUE_MAX;
		if (torque[i] > __TORQUE_MAX) torque[i] = __TORQUE_MAX;
	}
	
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pwm_msec + (pwm_msec * torque[0] / __TORQUE_MAX));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, pwm_msec + (pwm_msec * torque[1] / __TORQUE_MAX));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pwm_msec + (pwm_msec * torque[2] / __TORQUE_MAX));
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pwm_msec + (pwm_msec * torque[3] / __TORQUE_MAX));

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);	// Clear the timer interrupt
}

void TIMER2A_Handler(void) {
	int16_t accel_data[3];
	int16_t gyro_data[3];
	int16_t compass_data[3];

#ifdef __USE_IMU	
	switch (sensors_state) {
		case __MEASURE_ACCELEROMETER:
				adxl345_ReadXYZ(&accel_data[0], &accel_data[1], &accel_data[2]);
				accel.x += ((float)accel_data[0] - accel.x)/10;
				accel.y += ((float)accel_data[1] - accel.y)/10;
				accel.z += ((float)accel_data[2] - accel.z)/10;
				sensors_state = __MEASURE_GYROSCOPE;
			break;
		
		case __MEASURE_GYROSCOPE:
				itg3200_ReadXYZ(&gyro_data[0], &gyro_data[1], &gyro_data[2]);
				gyro.x += ((float)gyro_data[0] - gyro.x)/10;
				gyro.y += ((float)gyro_data[1] - gyro.y)/10;
				gyro.z += ((float)gyro_data[2] - gyro.z)/10;
				sensors_state = __MEASURE_COMPASS;
			break;
		
		case __MEASURE_COMPASS:
				hmc5883l_ReadXYZ(&compass_data[0], &compass_data[1], &compass_data[2]);
				compass.x += ((float)compass_data[0] - compass.x)/10;
				compass.y += ((float)compass_data[1] - compass.y)/10;
				compass.z += ((float)compass_data[2] - compass.z)/10;
				sensors_state = __MEASURE_ACCELEROMETER;
			break;
	}
#endif
	
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);	// Clear the timer interrupt
}

int main(void)
{
	FPULazyStackingEnable();
	FPUEnable();
	
	PeripheralClock_Config();
	GPIO_Config();
	Timers_Config();
	PWM_Config();
	SysTick_Config();
	UART_Config();
	USB_Config();
	I2C_Config();
	NVIC_Config();

#ifdef __USE_IMU
	adxl345_Init();
	hmc5883l_Init();
	itg3200_Init();
#endif
	
	kalman_init(&k_roll);
	kalman_init(&k_pitch);
	kalman_init(&k_yaw);
	
	pwm_msec = SysCtlPWMClockGet() / 1000;

  while(1)
  {

  }
}

void UART1_Handler(void)
{
uint8_t		usb_data[128];
uint32_t	UART_Int_Status;
	
	UART_Int_Status = UARTIntStatus(UART1_BASE, true);
	UARTIntClear(UART1_BASE, UART_Int_Status);
	
	BT_data = UARTCharGet(UART1_BASE);
	//UARTCharPut(UART1_BASE, BT_data+1);
	sprintf((char*)usb_data, "Recieved data: %c\n", BT_data);
	send_USB_CDC_Data(usb_data);
	
	switch (bt_reciever_state) {
		case __BT_STANDBY:
			switch (BT_data) {
				case 't':
					bt_reciever_state = __BT_RECEIVE_TORQUE;
					break;
				default:
					break;
			}
			break;
			
		case __BT_RECEIVE_TORQUE:
			user_torque = BT_data;
			sprintf((char*)usb_data, "torque setted: %d\n", user_torque);
			send_USB_CDC_Data(usb_data);
			bt_reciever_state = __BT_STANDBY;
			break;
	}
	
	if (LED_state) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	} else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	}
	LED_state = !LED_state;
}
