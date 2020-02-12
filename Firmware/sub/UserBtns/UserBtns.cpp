/**
 * @file HallEncoders.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "UserBtns.h"
#include <AnalogIn.h>

/**
 * Private subsystem info
 */
namespace UserBtns
{
	const uint8_t pin_btns = 0;	// TODO fix
	AnalogIn btns(pin_btns);
}

/**
 * @brief Gets ID of button press
 * @return Button ID (1-4, or 0 if no press)
 * 
 * Voltage mapping:
 * - Button 1 = 0.0V
 * - Button 2 = 0.5V
 * - Button 3 = 1.0V
 * - Button 4 = 1.5V
 * - No button = 3.3V
 */
uint8_t UserBtns::get()
{
	float volt = btns.read();
	if (volt < 0.25f) return 1;
	else if (volt < 0.75f) return 2;
	else if (volt < 1.25f) return 3;
	else if (volt < 1.75f) return 4;
	else return 0;
}
