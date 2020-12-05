/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define ARM_MATH_CM3

#include "gd32f1x0.h"

#include "../Inc/setup.h"
#include "../Inc/defines.h"
#include "../Inc/config.h"
#include "../Inc/it.h"
#include "../Inc/bldc.h"
#include "../Inc/commsMasterSlave.h"
#include "../Inc/commsSteering.h"
#include "../Inc/commsBluetooth.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>

#ifdef MASTER
int32_t vel_slave = 0; 							// global variable for steering. -1000 to 1000
int32_t vel_master = 0; 						// global variable for speed.    -1000 to 1000
FlagStatus activateWeakening = RESET;			// global variable for weakening
FlagStatus beepsBackwards = RESET;  			// global variable for beeps backwards

extern uint8_t buzzerFreq;						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;					// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern float batteryVoltage; 					// global variable for battery voltage
extern float currentDC;							// global variable for current dc
extern float realSpeed;							// global variable for real Speed
uint8_t slaveError = 0;							// global variable for slave error

extern FlagStatus timedOut;						// Timeoutvariable set by timeout timer

uint32_t inactivity_timeout_counter = 0;		// Inactivity counter
uint32_t steerCounter = 0;						// Steer counter for setting update rate

void ShowBatteryState(uint32_t pin);
void BeepsBackwards(FlagStatus beepsBackwards);
void ShutOff(void);
#endif


//----------------------------------------------------------------------------
// MAIN function
//----------------------------------------------------------------------------
int main (void)
{
#ifdef MASTER
	FlagStatus enable = RESET;
	FlagStatus enableSlave = RESET;
	FlagStatus chargeStateLowActive = SET;
	int16_t sendSlaveValue = 0;
	uint8_t sendSlaveIdentifier = 0;
	int8_t index = 8;
	int16_t pwmSlave = 0;
	int16_t pwmMaster = 0;
#endif
	
	//SystemClock_Config();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 100);
	
	// Init watchdog
	if (Watchdog_init() == ERROR)
	{
		// If an error accours with watchdog initialization do not start device
		while(1);
	}
	
	// Init Interrupts
	Interrupt_init();
	
	// Init timeout timer
	TimeoutTimer_init();
	
	// Init GPIOs
	GPIO_init();
	
	// Activate self hold direct after GPIO-init
	gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, SET);
	
	// Init usart master slave
	USART_MasterSlave_init();
	
	// Init ADC
	ADC_init();
	
	// Init PWM
	PWM_init();
	
	// Device has 1,6 seconds to do all the initialization
	// afterwards watchdog will be fired
	fwdgt_counter_reload();
	
	// Init usart steer/bluetooth
	USART_Steer_COM_init();
	
#ifdef MASTER
	// Startup-Sound
	for (; index >= 0; index--)
	{
		buzzerFreq = index;
		Delay(10);
	}
	buzzerFreq = 0;
	
	// Wait until button is pressed
	while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN))
	{
		// Reload watchdog while button is pressed
		fwdgt_counter_reload();
	}
#endif
	
	while(1)
	{
#ifdef MASTER
		steerCounter++;	
		if ((steerCounter % 2) == 0)
		{	
			// Request steering data
			SendSteerDevice();
		}
		
		
		vel_master = vel_slave = 100;
		
		pwmMaster = CLAMP(vel_master, -1000, 1000);
		pwmSlave  = CLAMP(vel_slave,  -1000, 1000);
		ResetTimeout();
		
		// Read charge state
		chargeStateLowActive = gpio_input_bit_get(CHARGE_STATE_PORT, CHARGE_STATE_PIN);
		
		// Enable is depending on charger is connected or not
		enable = chargeStateLowActive;
		
		// Enable channel output
		SetEnable(enable);
		
		// Decide if slave will be enabled
		enableSlave = (enable == SET && timedOut == RESET) ? SET : RESET;
		
		// Decide which process value has to be sent
		switch(sendSlaveIdentifier)
		{
		case 0:
			sendSlaveValue = currentDC * 100;
			break;
		case 1:
			sendSlaveValue = batteryVoltage * 100;
			break;
		case 2:
			sendSlaveValue = realSpeed * 100;
			break;
		default:
			break;
		}
		
		// Set output
		SetPWM(pwmMaster);
		SendSlave(-pwmSlave, enableSlave, RESET, chargeStateLowActive, sendSlaveIdentifier, sendSlaveValue);
		
		// Increment identifier
		sendSlaveIdentifier++;
		if (sendSlaveIdentifier > 2)
		{
			sendSlaveIdentifier = 0;
		}
		
		// Show green battery symbol when battery level BAT_LOW_LVL1 is reached
		if (batteryVoltage > BAT_LOW_LVL1)
		{
			// Show green battery light
			ShowBatteryState(LED_GREEN);
			
			// Beeps backwards
			BeepsBackwards(beepsBackwards);
		}
		// Make silent sound and show orange battery symbol when battery level BAT_LOW_LVL2 is reached
		else if (batteryVoltage > BAT_LOW_LVL2 && batteryVoltage < BAT_LOW_LVL1)
		{
			// Show orange battery light
			ShowBatteryState(LED_ORANGE);
			
			buzzerFreq = 5;
			buzzerPattern = 8;
		}
		// Make even more sound and show red battery symbol when battery level BAT_LOW_DEAD is reached
		else if  (batteryVoltage > BAT_LOW_DEAD && batteryVoltage < BAT_LOW_LVL2)
		{
			// Show red battery light
			ShowBatteryState(LED_RED);
			
			buzzerFreq = 5;
			buzzerPattern = 1;
		}
		// Shut device off, when battery is dead
		else if (batteryVoltage < BAT_LOW_DEAD)
		{
			ShutOff();
		}
		else
		{
			ShutOff();
		}
		
		// Shut device off when button is pressed
		if (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN))
		{
			while (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN)) {}
			ShutOff();
		}
		
		// Calculate inactivity timeout (Except, when charger is active -> keep device running)
		if (ABS(pwmMaster) > 50 || ABS(pwmSlave) > 50 || !chargeStateLowActive)
		{
			inactivity_timeout_counter = 0;
		}
		else
		{
			inactivity_timeout_counter++;
		}
		
		// Shut off device after INACTIVITY_TIMEOUT in minutes
		if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))
		{ 
			ShutOff();
		}
#endif	
		
		Delay(DELAY_IN_MAIN_LOOP);
		
		// Reload watchdog (watchdog fires after 1,6 seconds)
		fwdgt_counter_reload();
	}
}

#ifdef MASTER
//----------------------------------------------------------------------------
// Turns the device off
//----------------------------------------------------------------------------
void ShutOff(void)
{
	int index = 0;
	
	buzzerPattern = 0;
	for (; index < 8; index++)
	{
		buzzerFreq = index;
		Delay(10);
	}
	buzzerFreq = 0;
	
	// Send shut off command to slave
	SendSlave(0, RESET, SET, RESET, RESET, RESET);
	
	// Disable usart
	usart_deinit(USART_MASTERSLAVE);
	
	// Set pwm and enable to off
	SetEnable(RESET);
	SetPWM(0);
	
	gpio_bit_write(SELF_HOLD_PORT, SELF_HOLD_PIN, RESET);
	while(1)
	{
		// Reload watchdog until device is off
		fwdgt_counter_reload();
	}
}

//----------------------------------------------------------------------------
// Shows the battery state on the LEDs
//----------------------------------------------------------------------------
void ShowBatteryState(uint32_t pin)
{
	gpio_bit_write(LED_GREEN_PORT, LED_GREEN, pin == LED_GREEN ? SET : RESET);
	gpio_bit_write(LED_ORANGE_PORT, LED_ORANGE, pin == LED_ORANGE ? SET : RESET);
	gpio_bit_write(LED_RED_PORT, LED_RED, pin == LED_RED ? SET : RESET);
}

//----------------------------------------------------------------------------
// Beeps while driving backwards
//----------------------------------------------------------------------------
void BeepsBackwards(FlagStatus beepsBackwards)
{
	// If the speed is less than -50, beep while driving backwards
	if (beepsBackwards == SET && vel_master < -50)
	{
		buzzerFreq = 5;
		buzzerPattern = 4;
	}
	else
	{
		buzzerFreq = 0;
		buzzerPattern = 0;
	}
}
#endif
