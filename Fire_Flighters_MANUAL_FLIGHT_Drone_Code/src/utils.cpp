/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

#include "../include/utils.h"

unsigned long intervalLed = 500;

void led_init() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void blink_led(){
    volatile unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= intervalLed) {
        previousMillis = currentMillis;
        // blink led
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}