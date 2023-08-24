#include <button.hpp>
#ifndef ARDUINO
#include <driver/gpio.h>
#include <string.h>
using namespace esp_idf;
#else
using namespace arduino;
#endif

#ifndef ARDUINO
static uint32_t millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}
#endif
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
basic_button::basic_button(uint8_t pin, uint32_t debounce_ms, bool open_high) : m_pin(pin), m_debounce_ms(debounce_ms), m_open_high(open_high), m_pressed(-1), m_on_pressed_changed(nullptr), m_on_pressed_changed_state(nullptr) {
}
bool basic_button::initialized() const {
    return m_pressed != -1;
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
#ifdef ARDUINO
    return m_open_high ? !digitalRead(m_pin) : digitalRead(m_pin);
#else
    int i = gpio_get_level((gpio_num_t)m_pin);
    return m_open_high ? !i : i;
#endif
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
            m_pressed = pressed;
            if (m_on_pressed_changed != nullptr) {
                m_on_pressed_changed(pressed, m_on_pressed_changed_state);
            }
            m_last_change_ms = ms;
        }
    }
}

void basic_button::initialize() {
    if (m_pressed == -1) {
        m_last_change_ms = 0;
#ifdef ARDUINO
        if (m_open_high) {
#if defined(INPUT_PULLUP)
            pinMode(m_pin, INPUT_PULLUP);
#else
            pinMode(m_pin, INPUT);
            digitalWrite(m_pin, HIGH);
#endif
        } else {
#if defined(INPUT_PULLDOWN)
            pinMode(m_pin, INPUT_PULLDOWN);
#else
            pinMode(m_pin, INPUT);
            digitalWrite(m_pin, LOW);
#endif
        }
#else
        gpio_config_t io_conf;
        memset(&io_conf, 0, sizeof(io_conf));
        // disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // set as output mode
        io_conf.mode = GPIO_MODE_INPUT;
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = 1 << m_pin;
        if (m_open_high) {
            // disable pull-down mode
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            // enable pull-up mode
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        } else {
            // enable pull-down mode
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            // disable pull-up mode
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        }
        // configure GPIO with the given settings
        gpio_config(&io_conf);
#endif
        m_pressed = 0;
        m_pressed = raw_pressed();
    }
}
