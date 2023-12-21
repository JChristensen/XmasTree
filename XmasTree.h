// LEDs for small ceramic Christmas tree, powered by an ATTiny84A microcontroller.
// Copyright (C) 2023 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#include "FireLED.h"

// A Christmas tree with various lighting effects.
class XmasTree
{
    public:
        XmasTree() {}
        void begin();
        void fire();
        void twinkle(uint32_t dly);
        void spin(uint32_t dly);
        void fadePairs(uint32_t dly);
        void rotate(uint32_t dly);
        void fadeUp(uint8_t ledNbr, uint32_t dly);
        void fadeDn(uint8_t ledNbr, uint32_t dly);
        void fadeAllDn(uint32_t dly);
        void fadeAllUp(uint32_t dly);
        void fadeDnUp(uint8_t ledNbr1, uint8_t ledNbr2, uint32_t dly);
        void on(uint8_t ledNbr);
        void off(uint8_t ledNbr);
        void allOff();
        void lampTest();

    private:
        void checkVcc();
        int  readVcc();
        void gotoSleep();

        int m_minVcc {0};
        uint32_t m_lastVccCheck {0};
        const uint32_t m_VccCheckInterval {1000};
        uint32_t m_effectStart;
        const uint32_t m_effectDuration {30000};

        const uint8_t
            m_leds[4]       {3, 2, 4, 5},
            m_boostEnable   {7},
            m_debugLED      {8},
            m_buttonPin     {9},
            m_unusedPins[4] {0, 1, 6, 10};

        const uint8_t
            MIN_DUTY_CYCLE {16},
            MAX_DUTY_CYCLE {255};
        const uint32_t
            MIN_DELAY {10},
            MAX_DELAY {250};

        //FireLED f1{m_leds[0], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, MIN_DELAY, MAX_DELAY};
        //FireLED f2{m_leds[1], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, MIN_DELAY, MAX_DELAY};
        //FireLED f3{m_leds[2], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, MIN_DELAY, MAX_DELAY};
        //FireLED f4{m_leds[3], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, MIN_DELAY, MAX_DELAY};
        FireLED f2{m_leds[1], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, MIN_DELAY, MAX_DELAY, m_leds[3]};

        movingAvg Vcc{6};
        Button btn{m_buttonPin};    // for future use
};

void XmasTree::begin()
{
    pinMode(m_boostEnable, OUTPUT);
    digitalWrite(m_boostEnable, HIGH);
    pinMode(m_debugLED, OUTPUT);

    for (uint8_t i = 0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
        pinMode(m_leds[i], OUTPUT);
    }

    for (uint8_t i = 0; i < sizeof(m_unusedPins) / sizeof(m_unusedPins[0]); i++) {
        pinMode(m_unusedPins[i], INPUT_PULLUP);
    }

    btn.begin();
    Vcc.begin();
    // assume Vcc is either 5V or 3V3; set min accordingly
    m_minVcc = (readVcc()) < 3600 ? 3000 : 4500;
}

void XmasTree::fire()
{
    m_effectStart = millis();
    //f1.begin();
    f2.begin();
    //f3.begin();
    //f4.begin();

    while (millis() - m_effectStart < m_effectDuration) {
        //f1.run();
        f2.run();
        //f3.run();
        //f4.run();
        checkVcc();
    }
    allOff();
}

void XmasTree::twinkle(uint32_t dly)
{
    m_effectStart = millis();
    while (millis() - m_effectStart < m_effectDuration) {
        uint8_t led = random(4);
        on(led);
        delay(dly);
        off(led);
        checkVcc();
    }
}

void XmasTree::spin(uint32_t dly)
{
    m_effectStart = millis();
    fadeUp(0, 4);

    while (millis() - m_effectStart < m_effectDuration) {
        fadeDnUp(0, 1, dly);
        fadeDnUp(1, 2, dly);
        fadeDnUp(2, 3, dly);
        fadeDnUp(3, 0, dly);
        checkVcc();
    }
    fadeDn(0, 4);
}

// fade leds up and down in pairs
void XmasTree::fadePairs(uint32_t dly)
{
    m_effectStart = millis();
    // start with all off
    for (int i=0; i<4; i++) {off(i);}
    // fade 0 & 2 up
    for (int i=0; i<256; i++) {
        int val = i;
        analogWrite(m_leds[0], val);
        analogWrite(m_leds[2], val);
        delay(dly);
    }

    while (millis() - m_effectStart < m_effectDuration) {
        // fade 0 & 2 down, then 1 & 3 up
        constexpr int overlap = 64;
        for (int i=0; i<255-overlap; i++) {
            int valDn = 255 - i;
            analogWrite(m_leds[0], valDn);
            analogWrite(m_leds[2], valDn);
            checkVcc();
            delay(dly);
        }
        for (int i=255-overlap; i<256; i++) {
            int valDn = 255 - i;
            int valUp = i - (255 - overlap);
            analogWrite(m_leds[0], valDn);
            analogWrite(m_leds[2], valDn);
            analogWrite(m_leds[1], valUp);
            analogWrite(m_leds[3], valUp);
            checkVcc();
            delay(dly);
        }
        for (int i=overlap; i<256; i++) {
            int valUp = i;
            analogWrite(m_leds[1], valUp);
            analogWrite(m_leds[3], valUp);
            checkVcc();
            delay(dly);
        }

        // fade 1 & 3 down, then 0 & 2 up
        for (int i=0; i<255-overlap; i++) {
            int valDn = 255 - i;
            analogWrite(m_leds[1], valDn);
            analogWrite(m_leds[3], valDn);
            checkVcc();
            delay(dly);
        }
        for (int i=255-overlap; i<256; i++) {
            int valDn = 255 - i;
            int valUp = i - (255 - overlap);
            analogWrite(m_leds[1], valDn);
            analogWrite(m_leds[3], valDn);
            analogWrite(m_leds[0], valUp);
            analogWrite(m_leds[2], valUp);
            checkVcc();
            delay(dly);
        }
        for (int i=overlap; i<256; i++) {
            int valUp = i;
            analogWrite(m_leds[0], valUp);
            analogWrite(m_leds[2], valUp);
            checkVcc();
            delay(dly);
        }
    }

    // fade 0 & 2 Down
    for (int i=0; i<256; i++) {
        int val = 255 - i;
        analogWrite(m_leds[0], val);
        analogWrite(m_leds[2], val);
        delay(dly);
    }
    checkVcc();
}

void XmasTree::rotate(uint32_t dly)
{
    m_effectStart = millis();
    on(0); delay(dly);

    while (millis() - m_effectStart < m_effectDuration) {
        for (int i=0; i<4; i++) {
            int next = (i + 1 > 3) ? 0 : i + 1;
            on(next); off(i);
            delay(dly);
            checkVcc();
        }
    }
    off(0);
}

// fade the given led number up, delay between steps in milliseconds
void XmasTree::fadeUp(uint8_t ledNbr, uint32_t dly) {
    for (int i=0; i<256; i++) {
        int val = i;
        analogWrite(m_leds[ledNbr], val);
        delay(dly);
    }
}

// fade the given led number down, delay between steps in milliseconds
void XmasTree::fadeDn(uint8_t ledNbr, uint32_t dly) {
    for (int i=0; i<256; i++) {
        int val = 255 - i;
        analogWrite(m_leds[ledNbr], val);
        delay(dly);
    }
}

// fade all leds down simultaneously, delay between steps in milliseconds
void XmasTree::fadeAllDn(uint32_t dly) {
    for (int i=0; i<256; i++) {
        int val = 255 - i;
        for (uint8_t i = 0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
            analogWrite(m_leds[i], val);
        }
        delay(dly);
    }
}

// fade all leds up simultaneously, delay between steps in milliseconds
void XmasTree::fadeAllUp(uint32_t dly) {
    for (int i=0; i<256; i++) {
        int val = 255;
        for (uint8_t i = 0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
            analogWrite(m_leds[i], val);
        }
        delay(dly);
    }
}

// fade ledNbr1 down, fade ledNbr2 up, delay between steps in milliseconds
void XmasTree::fadeDnUp(uint8_t ledNbr1, uint8_t ledNbr2, uint32_t dly) {
    for (int i=0; i<256; i++) {
        int valUp = i;
        int valDn = 255 - i;
        analogWrite(m_leds[ledNbr1], valDn);
        analogWrite(m_leds[ledNbr2], valUp);
        delay(dly);
    }
}

// turn the given led on
void XmasTree::on(uint8_t ledNbr)
{
    digitalWrite(m_leds[ledNbr], HIGH);
}

// turn the given led off
void XmasTree::off(uint8_t ledNbr)
{
    digitalWrite(m_leds[ledNbr], LOW);
}

// turn all leds off
void XmasTree::allOff()
{
    for (uint8_t i=0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
        digitalWrite(m_leds[i], LOW);
    }

}

// lamp test - if button pressed at power on, run a lamp test
// that never returns, else just blip the debug led.
void XmasTree::lampTest()
{
    if (btn.isPressed()) {
        digitalWrite(m_debugLED, HIGH);
        delay(1000);
        digitalWrite(m_debugLED, LOW);
        delay(250);
        for (uint8_t n=0; n<2; n++) {
            for (uint8_t i=0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
                uint8_t j = (i == 0) ? sizeof(m_leds) / sizeof(m_leds[0]) - 1 : i - 1;
                digitalWrite(m_leds[j], LOW);
                digitalWrite(m_leds[i], HIGH);
                delay(1000);
            }
        }
        while (true) {
            for (uint8_t i=0; i < sizeof(m_leds) / sizeof(m_leds[0]); i++) {
                uint8_t j = (i == 0) ? sizeof(m_leds) / sizeof(m_leds[0]) - 1 : i - 1;
                digitalWrite(m_leds[j], LOW);
                digitalWrite(m_leds[i], HIGH);
                delay(10000);
            }
        }
    }
    else {
        digitalWrite(m_debugLED, HIGH);
        delay(250);
        digitalWrite(m_debugLED, LOW);
        delay(250);
    }
}

// read 1.1V reference against Vcc.
// returns Vcc in millivolts.
// from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
int XmasTree::readVcc()
{
    ADMUX = _BV(MUX5) | _BV(MUX0);
    delay(5);                               // Vref settling time
    ADCSRA |= _BV(ADSC);                    // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);  // wait for it to complete
    return 1126400L / ADC;                  // calculate AVcc in mV (1.1 * 1000 * 1024)
}

// read Vcc and if it is not holding up, put the mcu to sleep.
void XmasTree::checkVcc()
{
    uint32_t ms = millis();
    if ( ms - m_lastVccCheck >= m_VccCheckInterval) {
        m_lastVccCheck = ms;
        int v = readVcc();
        int avgVcc = Vcc.reading(v);
        if (avgVcc < m_minVcc) {
            off(0); off(1); off(2); off(3);
            digitalWrite(m_debugLED, LOW);
            gotoSleep();
        }
    }
}

// put the mcu to sleep.
void XmasTree::gotoSleep()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
//    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));  // INT0 on low level
//    GIMSK |= _BV(INT0);                   // enable INT0
    uint8_t adcsra = ADCSRA;                // save ADCSRA
    ADCSRA &= ~_BV(ADEN);                   // disable ADC
    cli();                                  // stop interrupts to ensure the BOD timed sequence executes as required
//    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);   // turn off the brown-out detector
//    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
//    MCUCR = mcucr1;
//    MCUCR = mcucr2;
//    sei();                                // keep interrupts disabled, sleep until external reset or power-on reset
    PORTA &= ~_BV(PORTA3);                  // turn off m_boostEnable, the fast way
    sleep_cpu();                            // go to sleep
    sleep_disable();                        // wake up here
    ADCSRA = adcsra;                        // restore ADCSRA
}
