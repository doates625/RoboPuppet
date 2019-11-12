/**
 * @file I2CBus.cpp
 */
#include <I2CBus.h>

/**
 * Private data and methods
 */
namespace I2CBus
{
	TwoWire* const wire = &Wire;		// Wire interface
	const uint32_t clock_rate = 100000;	// I2C clock rate
	const uint8_t pin_scl = 19;			// SCL pin
	const uint8_t pin_sda = 18;			// SDA pin
	bool init_complete = false;			// Init flag
}

/**
 * @brief Initializes I2C bus
 */
void I2CBus::init()
{
	if (!init_complete)
	{
		wire->begin();
		wire->setClock(clock_rate);
		wire->setSCL(pin_scl);
		wire->setSDA(pin_sda);
		init_complete = true;
	}
}

/**
 * @brief Returns TwoWire pointer
 */
TwoWire* I2CBus::get_wire()
{
	return wire;
}