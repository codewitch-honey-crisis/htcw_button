#pragma once
#include <Arduino.h>
namespace arduino {
typedef void(*button_callback)(bool pressed, void* state);
template<uint8_t Pin,uint32_t DebounceMS = 10,bool ClosedHigh = false>
class button {
public:
    using type = button;
    constexpr static const uint8_t pin = Pin;
    constexpr static const uint32_t debounce_ms = DebounceMS;
    constexpr static const bool closed_high = ClosedHigh;
private:
    int m_pressed;
    button_callback m_callback;
    void* m_state;
    uint32_t m_last_change_ms;
    button(const button& rhs)=delete;
    button& operator=(const button& rhs)=delete;
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
        if(m_pressed==-1) {
            m_last_change_ms = 0;
            if(closed_high) {
                pinMode(pin,INPUT_PULLUP);
            } else {
                pinMode(pin,INPUT_PULLDOWN);
            }
            m_pressed = 0;
            m_pressed = raw_pressed();
        }
        return m_pressed!=-1;
    }
    inline void callback(button_callback callback, void* state=nullptr) {
        m_callback = callback;
        m_state = state;
    }
    inline bool raw_pressed() { initialize(); return closed_high?!digitalRead(pin):digitalRead(pin);}
    inline bool pressed() { initialize(); return m_pressed; }
    void update() {
        bool pressed = raw_pressed();
        if(pressed!=m_pressed) {
            uint32_t ms = millis();
            if(ms-m_last_change_ms>=debounce_ms) {    
                if(m_callback!=nullptr) {
                    m_callback(pressed,m_state);
                }
                m_pressed = pressed;
                m_last_change_ms=ms;
            }
        }
    }
};
}