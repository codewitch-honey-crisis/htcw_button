#pragma once
#include <Arduino.h>
namespace arduino {
// standard button callback - when pressed or depressed
typedef void (*button_callback)(bool pressed, void* state);
// represents a simple button
template <uint8_t Pin, uint32_t DebounceMS = 10, bool OpenHigh = false>
class button final {
   public:
    // the button type
    using type = button;
    // the pin of the button
    constexpr static const uint8_t pin = Pin;
    // milliseconds of debounce
    constexpr static const uint32_t debounce_ms = DebounceMS;
    // true if switch is inverted such that it's high when not pressed
    constexpr static const bool open_high = OpenHigh;

   private:
    int m_pressed;
    button_callback m_callback;
    void* m_state;
    uint32_t m_last_change_ms;
    button(const button& rhs) = delete;
    button& operator=(const button& rhs) = delete;

   public:
    button(button&& rhs) {
        m_callback = rhs.m_callback;
        rhs.m_callback = nullptr;
        m_state = rhs.m_state;
        m_last_change_ms = rhs.m_last_change_ms;
        m_pressed = rhs.m_pressed;
    }
    button& operator=(button&& rhs) {
        m_callback = rhs.m_callback;
        rhs.m_callback = nullptr;
        m_state = rhs.m_state;
        m_last_change_ms = rhs.m_last_change_ms;
        m_pressed = rhs.m_pressed;
        return *this;
    }
    button() : m_pressed(-1), m_callback(nullptr), m_state(nullptr) {
    }
    // initializes the button
    bool initialize() {
        if (m_pressed == -1) {
            m_last_change_ms = 0;
            if (open_high) {
                pinMode(pin, INPUT_PULLUP);
            } else {
                pinMode(pin, INPUT_PULLDOWN);
            }
            m_pressed = 0;
            m_pressed = raw_pressed();
        }
        return m_pressed != -1;
    }
    // sets the callback
    inline void callback(button_callback callback, void* state = nullptr) {
        m_callback = callback;
        m_state = state;
    }
    // indicates whether the button is pressed (no debounce)
    inline bool raw_pressed() {
        initialize();
        return open_high ? !digitalRead(pin) : digitalRead(pin);
    }
    // indicates whether the button is pressed
    inline bool pressed() {
        initialize();
        return m_pressed == 1;
    }
    // updates the button. should be done in loops
    void update() {
        bool pressed = raw_pressed();
        if (pressed != m_pressed) {
            uint32_t ms = millis();
            if (ms - m_last_change_ms >= debounce_ms) {
                if (m_callback != nullptr) {
                    m_callback(pressed, m_state);
                }
                m_pressed = pressed;
                m_last_change_ms = ms;
            }
        }
    }
};

// the callbacks for the extended button
typedef void (*button_ex_callback)(void* state);

// represents an extended button
template <uint8_t Pin, uint32_t DebounceMS = 10, bool OpenHigh = false, uint32_t DoubleClickMS = 250, uint32_t LongClickMS = 500>
struct button_ex final {
    using type = button_ex;
    constexpr static const uint8_t pin = Pin;
    constexpr static const uint32_t debounce_ms = DebounceMS;
    constexpr static const bool open_high = OpenHigh;
    constexpr static const uint32_t double_click_ms = DoubleClickMS;
    constexpr static const uint32_t long_click_ms = LongClickMS;

   private:
    int m_pressed;
    int m_clicks;
    int m_long_clicks;
    uint32_t m_last_change_ms;
    uint32_t m_last_release_ms;
    uint32_t m_last_press_ms;
    button_ex_callback m_click_cb;
    void* m_click_state;
    button_ex_callback m_double_click_cb;
    void* m_double_click_state;
    button_ex_callback m_long_click_cb;
    void* m_long_click_state;
    button_ex(const button_ex& rhs) = delete;
    button_ex& operator=(const button_ex& rhs) = delete;

   public:
    button_ex(button_ex&& rhs) {
        m_pressed = rhs.m_pressed;
        m_clicks = rhs.m_clicks;
        m_long_clicks = rhs.m_long_clicks;
        m_last_change_ms = rhs.m_last_change_ms;
        m_last_release_ms = rhs.m_last_release_ms;
        m_last_press_ms = rhs.m_last_press_ms;
        m_click_cb = rhs.m_click_cb;
        rhs.m_click_cb = nullptr;
        m_click_state = rhs.m_click_state;
        m_double_click_cb = rhs.m_double_click_cb;
        rhs.m_double_click_cb = nullptr;
        m_double_click_state = rhs.m_double_click_state;
        m_long_click_cb = rhs.m_long_click_cb;
        rhs.m_long_click_cb = nullptr;
        m_long_click_state = rhs.m_long_click_state;
    }
    button_ex& operator=(button_ex&& rhs) {
        m_pressed = rhs.m_pressed;
        m_clicks = rhs.m_clicks;
        m_long_clicks = rhs.m_long_clicks;
        m_last_change_ms = rhs.m_last_change_ms;
        m_last_release_ms = rhs.m_last_release_ms;
        m_last_press_ms = rhs.m_last_press_ms;
        m_click_cb = rhs.m_click_cb;
        rhs.m_click_cb = nullptr;
        m_click_state = rhs.m_click_state;
        m_double_click_cb = rhs.m_double_click_cb;
        rhs.m_double_click_cb = nullptr;
        m_double_click_state = rhs.m_double_click_state;
        m_long_click_cb = rhs.m_long_click_cb;
        rhs.m_long_click_cb = nullptr;
        m_long_click_state = rhs.m_long_click_state;
        return *this;
    }
    button_ex() : m_pressed(-1), m_clicks(0), m_long_clicks(0), m_last_change_ms(0), m_last_release_ms(0), m_last_press_ms(0), m_click_cb(nullptr), m_click_state(nullptr), m_double_click_cb(nullptr), m_double_click_state(nullptr), m_long_click_cb(nullptr), m_long_click_state(nullptr) {
    }
    // initializes the button
    bool initialize() {
        if (m_pressed == -1) {
            m_last_change_ms = 0;
            if (open_high) {
                pinMode(pin, INPUT_PULLUP);
            } else {
                pinMode(pin, INPUT_PULLDOWN);
            }
            m_pressed = 0;
            m_clicks = 0;
            m_long_clicks = 0;
            m_pressed = raw_pressed();
            m_last_press_ms = 0;
            m_last_release_ms = 0;
        }
        return m_pressed != -1;
    }
    // indicates if the button is pressed (no debounce)
    inline bool raw_pressed() {
        initialize();
        return open_high ? !digitalRead(pin) : digitalRead(pin);
    }
    // indicates if the button is pressed
    inline bool pressed() {
        initialize();
        return m_pressed > 0;
    }
    // updates the button (should be called in loops)
    void update() {
        if (!initialize()) {
            return;
        }
        bool pressed = raw_pressed();
        uint32_t ms = millis();
        if (pressed != m_pressed) {
            if (ms - m_last_change_ms >= debounce_ms) {
                uint32_t press_ms,release_ms;
                if (!pressed) {
                    press_ms = ms - m_last_press_ms;
                    m_last_release_ms = ms;
                    m_last_press_ms = 0;
                    if(press_ms>=long_click_ms) {
                        ++m_long_clicks;
                    } else {
                        ++m_clicks;
                    }
                } else {
                    release_ms = ms - m_last_release_ms;
                    m_last_press_ms = ms;
                    m_last_release_ms = 0;
                }
                m_pressed = pressed;
                m_last_change_ms = ms;
            }
            if (!pressed) {
                
                m_last_release_ms = ms;
                
            } else  {
                m_last_press_ms = ms;
            }
        }

        if (m_last_release_ms != 0 && m_last_release_ms + double_click_ms < ms) {
            if (!pressed) {
                while(m_long_clicks!=0) {
                    if (m_long_click_cb != nullptr) {
                        m_long_click_cb(m_long_click_state);
                    }
                    --m_long_clicks;
                }
                switch (m_clicks) {
                    case 0:
                        break;
                    case 1:
                        if (m_click_cb != nullptr) {
                            m_click_cb(m_click_state);
                        }
                        break;
                    default:
                        if (m_double_click_cb != nullptr) {
                            m_double_click_cb(m_double_click_state);
                        }
                        m_clicks = 0;
                        break;
                }
                m_last_release_ms = 0;
                m_clicks = 0;
            }
        }   
    }
    // sets the callback on click 
    void on_click(button_ex_callback cb, void* state = nullptr) {
        m_click_cb = cb;
        m_click_state = state;
    }
    // sets the callback on double click
    void on_double_click(button_ex_callback cb, void* state = nullptr) {
        m_double_click_cb = cb;
        m_double_click_state = state;
    }
    // sets the callback on long click
    void on_long_click(button_ex_callback cb, void* state = nullptr) {
        m_long_click_cb = cb;
        m_long_click_state = state;
    }
};
}  // namespace arduino