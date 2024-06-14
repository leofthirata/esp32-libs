#ifndef __LED_HPP__
#define __LED_HPP__

#include "main.h"

namespace BiColorStatus
{
enum class State
{
    TURN_ON,
    BAT_FULL,
    BAT_CHARGING,
    BAT_ERROR,
    WORKING,
    TRANSMITTING,
    RECEIVING,
    SEND_POSITION,
    STORING_POSITION,
    SIMCARD_ERROR,
};

void init();
void turnOn();
void batCharged();
void batCharging();
void batError();
void working();
void transmitting();
void receiving();
void sendPosition();
void storingPosition();
void simCardError();

enum Color: uint16_t
{
    RED = 0xFF00,
    GREEN = 0x00FF,
    ORANGE = 0xFFFF,
    NONE = 0x0000
};

} // namespace BiColor

#endif //__LED_HPP__