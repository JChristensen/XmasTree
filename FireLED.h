// LEDs for small ceramic Christmas tree, powered by an ATTiny84A microcontroller.
// Copyright (C) 2023 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

// Flickering LED effect similar to flames.
class FireLED
{
    public:
        FireLED(uint8_t pin, uint8_t minDutyCycle, uint8_t maxDutyCycle,
            uint32_t minDelay, uint32_t maxDelay, uint8_t pin2=255)
            : m_pin {pin}, m_minDutyCycle {minDutyCycle}, m_maxDutyCycle {maxDutyCycle},
              m_minDelay {minDelay}, m_maxDelay {maxDelay}, m_pin2 {pin2} {}
        void begin();
        void run();

    private:
        enum states_t {WAIT, ADJUST};
        uint8_t
            m_pin,
            m_minDutyCycle,
            m_maxDutyCycle,
            m_dutyCycle,
            m_nextDutyCycle;
        uint32_t
            m_minDelay,
            m_maxDelay,
            m_lastChange,
            m_duration,     // interval to wait between changes
            m_lastAdjust;
        uint8_t m_pin2;     // optional slave pin
        uint32_t m_fadeInterval {4};
        states_t m_state {WAIT};
        bool m_fadeUp;
};

// initialization
void FireLED::begin()
{
    pinMode(m_pin, OUTPUT);
    if (m_pin2 < 255) pinMode(m_pin2, OUTPUT);
    m_lastChange = millis();
    m_duration = 0;
}

// run the LED
void FireLED::run()
{
    uint32_t ms = millis();
    switch (m_state) {
        case WAIT:
            if (ms - m_lastChange >= m_duration) {
                m_state = ADJUST;
                m_lastChange = ms;
                m_duration = m_minDelay + random(m_maxDelay - m_minDelay);
                m_nextDutyCycle = m_minDutyCycle + random(m_maxDutyCycle - m_minDutyCycle);
                m_fadeUp = m_nextDutyCycle > m_dutyCycle;
                m_lastAdjust = ms;
            }
            break;

        case ADJUST:
            if (m_dutyCycle == m_nextDutyCycle) {
                m_state = WAIT;
            }
            else if (ms - m_lastAdjust >= m_fadeInterval) {
                m_lastAdjust = ms;
                m_fadeUp ? ++m_dutyCycle : --m_dutyCycle;
                analogWrite(m_pin, m_dutyCycle);
                if (m_pin2 < 255) analogWrite(m_pin2, m_dutyCycle);
            }
            break;
    }
}
