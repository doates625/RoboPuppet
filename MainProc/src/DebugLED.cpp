/**
 * @file DebugLED.cpp
 */
#include <DebugLED.h>
#include <WProgram.h>

/**
 * Private data and methods
 */
namespace DebugLED
{
    const uint8_t pin_led = 13;
    float init_complete = false;
}

/**
 * @brief Initializes and turns off LED.
 */
void DebugLED::init()
{
    if (!init_complete)
    {
        pinMode(pin_led, OUTPUT);
        off();
        init_complete = true;
    }
}

/**
 * @brief Turns LED on.
 */
void DebugLED::on()
{
    digitalWrite(pin_led, HIGH);
}

/**
 * @brief Turns LED off.
 */
void DebugLED::off()
{
    digitalWrite(pin_led, LOW);
}