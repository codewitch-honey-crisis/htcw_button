#include <button.hpp>
using namespace arduino;

void basic_button::do_move(basic_button& rhs) {
    m_on_pressed_changed = rhs.m_on_pressed_changed;
    rhs.m_on_pressed_changed = nullptr;
    m_on_pressed_changed_state = rhs.m_on_pressed_changed_state;
    m_last_change_ms = rhs.m_last_change_ms;
    m_pressed = rhs.m_pressed;
}

basic_button::basic_button(basic_button&& rhs) {
    do_move(rhs);
}
basic_button& basic_button::operator=(basic_button&& rhs) {
    do_move(rhs);
    return *this;
}
basic_button::basic_button(uint8_t pin, uint32_t debounce_ms,bool open_high ) : m_pin(pin),m_debounce_ms(debounce_ms),m_open_high(open_high), m_pressed(-1), m_on_pressed_changed(nullptr), m_on_pressed_changed_state(nullptr) {
}
bool basic_button::initialized() const {
    return m_pressed!=-1;
}
void basic_button::initialize() {
    if (m_pressed == -1) {
        m_last_change_ms = 0;
        if (m_open_high) {
#if defined(INPUT_PULLUP)
            pinMode(m_pin, INPUT_PULLUP);
#else
            pinMode(m_pin, INPUT);
            digitalWrite(m_pin,HIGH);
#endif
        } else {
#if defined(INPUT_PULLDOWN)
            pinMode(m_pin, INPUT_PULLDOWN);
#else
            pinMode(m_pin, INPUT);
            digitalWrite(m_pin,LOW);
#endif
        }
        m_pressed = 0;
        m_pressed = raw_pressed();
    }
}
void basic_button::deinitialize() {
    m_pressed = -1;
}
void basic_button::on_pressed_changed(button_on_pressed_changed callback, void* state) {
    m_on_pressed_changed = callback;
    m_on_pressed_changed_state = state;
}
bool basic_button::raw_pressed() {
    initialize();
    return m_open_high ? !digitalRead(m_pin) : digitalRead(m_pin);
}
bool basic_button::pressed() {
    initialize();
    return m_pressed == 1;
}
void basic_button::update() {
    bool pressed = raw_pressed();
    if (pressed != m_pressed) {
        uint32_t ms = millis();
        if (ms - m_last_change_ms >= m_debounce_ms) {
            if (m_on_pressed_changed != nullptr) {
                m_on_pressed_changed(pressed, m_on_pressed_changed_state);
            }
            m_pressed = pressed;
            m_last_change_ms = ms;
        }
    }
}
