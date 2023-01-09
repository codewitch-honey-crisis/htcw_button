#include <Arduino.h>
#include <button.hpp>
#define BUTTON_PIN 0
#define BUTTON_DEBOUNCE 10
#define BUTTON_INVERT true
using namespace arduino;
// declare an interrupt based button
int_button<BUTTON_PIN,BUTTON_DEBOUNCE,BUTTON_INVERT> button_a_raw;
// declare a multi-button wrapper for it
multi_button button_a(button_a_raw);
void setup() {
    Serial.begin(115200);
    // initialize
    button_a.initialize();
    // set the callbacks
    button_a.on_pressed_changed([](bool pressed,void* state) { Serial.print("button a: "); Serial.println(pressed?"pressed":"released"); });
    button_a.on_click([](int clicks,void* state) {Serial.print("button a: "); Serial.print(clicks);Serial.println(" clicks");});
    button_a.on_long_click([](void* state){Serial.println("button a: long click");});
}

void loop() {
    // pump the button
    button_a.update();
}