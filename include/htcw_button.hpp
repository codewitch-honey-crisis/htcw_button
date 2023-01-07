#pragma once
#include <Arduino.h>
#include <htcw_data.hpp>
namespace arduino {
typedef void (*button_callback)(bool pressed, void* state);
template <uint8_t Pin, uint32_t DebounceMS = 10, bool OpenHigh = false>
class button {
   public:
    using type = button;
    constexpr static const uint8_t pin = Pin;
    constexpr static const uint32_t debounce_ms = DebounceMS;
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
    inline void callback(button_callback callback, void* state = nullptr) {
        m_callback = callback;
        m_state = state;
    }
    inline bool raw_pressed() {
        initialize();
        return open_high ? !digitalRead(pin) : digitalRead(pin);
    }
    inline bool pressed() {
        initialize();
        return m_pressed == 1;
    }
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

typedef void (*button_ex_on_click_callback)(int clicks,void* state);
typedef void (*button_ex_on_long_click_callback)(void* state);
template <uint8_t Pin, uint32_t DebounceMS = 10, bool OpenHigh = false, bool UseInterrupt = false,uint32_t DoubleClickMS = 250, uint32_t LongClickMS = 500, size_t Events = 32>
struct button_ex final {
    using type = button_ex;
    constexpr static const uint8_t pin = Pin;
    constexpr static const uint32_t debounce_ms = DebounceMS;
    constexpr static const bool open_high = OpenHigh;
    constexpr static const bool use_interrupt = UseInterrupt;
    constexpr static const uint32_t double_click_ms = DoubleClickMS;
    constexpr static const uint32_t long_click_ms = LongClickMS;
    constexpr static const size_t events_size = Events;
private:
    static type* m_this;
    typedef struct event_entry {
        uint32_t ms;
        int state;
    } event_entry_t;
    using event_buffer_t = data::circular_buffer<event_entry,events_size>;
    int m_pressed;
    uint32_t m_last_change_ms;
    event_buffer_t m_events;
    button_ex_on_click_callback m_on_click;
    void* m_on_click_state;
    button_ex_on_long_click_callback m_on_long_click;
    void* m_on_long_click_state;
    #ifdef ESP32
    IRAM_ATTR 
    #endif
    static void process_change() {
        uint32_t ms = millis();
        bool pressed = m_this->raw_pressed();
        if (pressed != m_this->m_pressed) {
            if (ms - m_this->m_last_change_ms >= debounce_ms) {
                if(!m_this->m_events.full()) {
                    m_this->m_events.put({ms,pressed});
                    m_this->m_pressed = pressed;
                    m_this->m_last_change_ms = ms;
                }
            }
        }
    }
public:
    button_ex() {
        m_pressed = -1;
        m_last_change_ms = 0;
        m_on_click = nullptr;
        m_on_click_state = nullptr;
        m_on_long_click = nullptr;
        m_on_long_click_state = nullptr;
    }
    bool initialize() {
        if(m_pressed==-1) {
            if (open_high) {
                pinMode(pin, INPUT_PULLUP);
            } else {
                pinMode(pin, INPUT_PULLDOWN);
            }
            m_this = this;
            if(use_interrupt) {
                attachInterrupt(digitalPinToInterrupt(pin),process_change,CHANGE);
            }
            m_pressed = open_high ? !digitalRead(pin) : digitalRead(pin);
            m_last_change_ms = 0;
        }
        return m_pressed!=-1;
    }
    void deinitialize() {
        if(m_pressed!=-1) {
            if(use_interrupt) {
                detachInterrupt(digitalPinToInterrupt(pin));
            }
            m_pressed=-1;
        }
    }
    bool raw_pressed() {
        initialize();
        return open_high ? !digitalRead(pin) : digitalRead(pin);
    }
    void update() {
        if(!initialize()) {
            return;
        }
        if(!use_interrupt) {
            process_change();
        }
        if(m_pressed==1) {
            return;
        }
        if(m_last_change_ms!=0 && !m_events.empty() && millis()-m_last_change_ms>= double_click_ms) {
            event_entry_t ev;
            event_entry_t ev_next;
            uint32_t press_ms=0;
            int state = 0;
            int clicks = 0;
            int longp = 0;
            int done = 0;
            while(!done) {
                switch(state) {
                case 0:
                    if(!m_events.get(&ev)) {
                        done = true;
                        break;
                    }
                    if(ev.state==1) {
                        // pressed
                        state = 1;
                        break;
                    } else {
                        // released
                        while(ev.state!=1) {
                            if(!m_events.get(&ev)) {
                                done = true;
                                break;
                            }
                            // pressed
                            state = 1;
                        }
                        break;
                    }
                case 1: // press state
                    ++clicks;
                    press_ms = ev.ms;
                    while(ev.state!=0) {
                        if(!m_events.get(&ev)) {
                            done = true;
                            break;
                        }
                        state = 2;
                    }
                    break;
                case 2: // release state
                    longp = !!(m_on_long_click && ev.ms-press_ms>=long_click_ms);
                    if(!m_events.get(&ev)) {
                        // flush the clicks
                        if(m_on_click) {
                            if(clicks>longp) {
                                m_on_click(clicks-longp,m_on_click_state);
                            }
                        }
                        if(longp) {
                            m_on_long_click(m_on_long_click_state);
                        }
                        done = true;
                        break;
                    }
                    state = 1;
                    break;
                }
            }
        }
    }
    void on_click(button_ex_on_click_callback callback, void* state = nullptr) {
        m_on_click = callback;
        m_on_click_state = state;
    }
    void on_long_click(button_ex_on_long_click_callback callback, void* state = nullptr) {
        m_on_long_click = callback;
        m_on_long_click_state = state;
    }
};
template <uint8_t Pin, uint32_t DebounceMS , bool OpenHigh , bool UseInterrupt ,uint32_t DoubleClickMS , uint32_t LongClickMS , size_t Events>
button_ex<Pin, DebounceMS, OpenHigh, UseInterrupt, DoubleClickMS, LongClickMS, Events>*
button_ex<Pin, DebounceMS, OpenHigh, UseInterrupt, DoubleClickMS, LongClickMS, Events>::m_this = nullptr;

}  // namespace arduino