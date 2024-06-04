#include "led.hpp"

namespace BiColor
{
bool pop = false;
uint32_t lastColor = 0x00;

static void updateLedBicolor(State state)
{
    static State last_state = State::INIT;
    if (pop)
    {
        led->pop();
        pop = false;
    }

    switch (state)
    {

    case State::INIT:
    {
        led->blink(Color::ORANGE, 1000);
        last_state = State::INIT;
        break;
    }

    case State::BAT_FULL:
    {
        led->color(Color::GREEN);
        last_state = State::BAT_FULL;
        break;
    }

    case State::BAT_CHARGING:
    {
        led->blink(Color::GREEN, 1000);         
        last_state = State::BAT_CHARGING;
        break;
    }

    case State::BAT_ERROR:
    {
        led->color(Color::ORANGE);
        last_state = State::BAT_ERROR;
        break;
    }

    case State::WORKING:
    {
        led->blink(Color::ORANGE, 1000);
        last_state = State::WORKING;
        break;
    }

    case State::TRANSMITTING:
    {
        led->color(Color::GREEN);
        last_state = State::TRANSMITTING;
        break;
    }

    case State::RECEIVING:
    {
        led->color(Color::RED);
        last_state = State::RECEIVING;
        break;
    }

    case State::SEND_POSITION:
    {
        led->blink(Color::RED, 1000);
        last_state = State::SEND_POSITION;
        break;
    }

    case State::STORING_POSITION:
    {
        led->blink(Color::RED, 500);
        last_state = State::STORING_POSITION;
        break;
    }

    case State::SIMCARD_ERROR:
    {
        led->blink(Color::RED, 300);
        last_state = State::SIMCARD_ERROR;
        break;
    }

    }
}

void init()
{
    updateLedBicolor(State::INIT);
}

void batCharged()
{
    updateLedBicolor(State::BAT_FULL);
}

void batCharging()
{
    updateLedBicolor(State::BAT_CHARGING);
}

void batError()
{
    updateLedBicolor(State::BAT_ERROR);
}

void working()
{
    updateLedBicolor(State::WORKING);
}

void transmitting()
{
    updateLedBicolor(State::TRANSMITTING);
}

void receiving()
{
    updateLedBicolor(State::RECEIVING);
}

void sendPosition()
{
    updateLedBicolor(State::SEND_POSITION);
}

void storingPosition()
{
    updateLedBicolor(State::STORING_POSITION);
}

void simCardError()
{
    updateLedBicolor(State::SIMCARD_ERROR);
}

} // namespace BiColor