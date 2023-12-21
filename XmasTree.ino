// LEDs for small ceramic Christmas tree, powered by an ATTiny84A microcontroller.
// J.Christensen Nov-2023
//
// Hardware design: https://github.com/JChristensen/XmasTreeLEDs
// This firmware:   https://github.com/JChristensen/XmasTree
//
// Developed with ATTinyCore 1.5.2 by Spence Konde and Arduino 1.8.19.
// Clock source 1MHz internal, Pin mapping CCW, LTO disabled, millis() enabled,
// EEPROM retained, BOD 1.8V.
// Set fuses: E:0xFF, H:0xD6, L:0x62 (same as factory settings, except 1.8V BOD)
// avrdude -p t84 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v
//
// Copyright (C) 2023 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#include <movingAvg.h>  // http://github.com/JChristensen/movingAvg
#include <JC_Button.h>  // https://github.com/JChristensen/JC_Button
#include <avr/sleep.h>
#include "XmasTree.h"

XmasTree tree;

void setup()
{
    tree.begin();
    tree.lampTest();
}

void loop()
{
    tree.spin(2);
    tree.twinkle(100);
    tree.fire();
    tree.rotate(500);
    tree.fadePairs(4);
}
