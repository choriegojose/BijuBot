/** @file encoder.c
 *
 * @brief Module to control a quadrature encoder.
 @par
 * COPYRIGHT NOTICE: (c) 2021 All rights reserved.
 * Propietary: Jose Ignacio Choriego - cho16523@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "encoder.h"

/*!
 * @brief Configure the module 0 of the quadrature encoder.
 *
 * @param[in] motor_ratio The gear ratio in the motor.
 * @param[in] encoder_pulses The maximum amount of pulses the encoder can 
 *            perceive.
 * @param[in] swap If the motor is acting on the opposite direction then the
 *            signals of the QEI module can be swapped to not change the
 *            hardware.
 *
 * @return void.
 */
void
qei_module0_config(uint32_t motor_ratio, uint32_t encoder_pulses, bool swap)
{
    uint32_t temp;
    if (swap == true)
    {
        temp = QEI_CONFIG_SWAP;
    }
    else
    {
        temp = QEI_CONFIG_NO_SWAP;
    }
	// Enable QEI Module 0 pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);


	// Wait for the QEI0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
	{
	}

	// Unlock pin PD7
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Set pins for PHA0 and PHB0
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);

	// Configure pins for use by the QEI peripheral
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//Make sure quadrature encoder is off
	QEIDisable(QEI0_BASE);
	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR |
		QEI_INTTIMER | QEI_INTINDEX);

	//Configure quadrature encoder using FT0481 top limit
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | temp), 
        motor_ratio * encoder_pulses);

	//Enable quadrature encoder
	QEIEnable(QEI0_BASE);

	QEIPositionSet(QEI0_BASE,0);

	//Enable noise filter
	QEIFilterDisable(QEI0_BASE);
	QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_2);
	QEIFilterEnable(QEI0_BASE);
}

/*!
 * @brief Configure the module 1 of the quadrature encoder.
 *
 * @param[in] motor_ratio The gear ratio in the motor.
 * @param[in] encoder_pulses The maximum amount of pulses the encoder can 
 *            perceive.
 * @param[in] swap If the motor is acting on the opposite direction then the
 *            signals of the QEI module can be swapped to not change the
 *            hardware.
 *
 * @return void.
 */
void
qei_module1_config(uint32_t motor_ratio, uint32_t encoder_pulses, bool swap)
{
    uint32_t temp;
    if (swap == true)
    {
        temp = QEI_CONFIG_SWAP;
    }
    else
    {
        temp = QEI_CONFIG_NO_SWAP;
    }
	// Enable QEI Module 1 pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);


	// Wait for the QEI1 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1))
	{
	}

	//Set pins for PHA1 and PHB1
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	//Configure pins for use by the QEI peripheral
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	//Make sure quadrature encoder is off
	QEIDisable(QEI1_BASE);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR |
		QEI_INTTIMER | QEI_INTINDEX);

	//Configure quadrature encoder using FT0481 top limit
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | temp),
        motor_ratio * encoder_pulses);

	//Enable quadrature encoder
	QEIEnable(QEI1_BASE);

	QEIPositionSet(QEI1_BASE, 0);

	//Enable noise filter
	QEIFilterDisable(QEI1_BASE);
	QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_2);
	QEIFilterEnable(QEI1_BASE);
}

/*!
 * @brief Get the position of the motor in degrees.
 *
 * @param[in] qei_base The module in which we want to read the info.
 * @param[in] motor_ratio The gear ratio in the motor.
 * @param[in] encoder_pulses The maximum amount of pulses the encoder can 
 *            perceive.
 *
 * @return The current position of the motor.
 */
float
get_position_in_degrees(uint32_t qei_base, uint32_t motor_ratio,
                        uint32_t encoder_pulses)
{
    uint32_t pos_in_pulses;
    float position;
    pos_in_pulses = QEIPositionGet(qei_base);
    position = pos_in_pulses * (2*3.1416/(motor_ratio*encoder_pulses));
return position;
}

/*** end of file ***/
