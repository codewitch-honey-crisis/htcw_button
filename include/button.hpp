#pragma once
#include <Arduino.h>
#include <htcw_data.hpp>
namespace arduino {
typedef void (*button_on_pressed_changed)(bool pressed, void* state);
class button {
public:
    virtual bool initialized() const=0;
    virtual void initialize()=0;
    virtual void deinitialize()=0;
    virtual bool pressed()=0;
    virtual void update()=0;
    virtual void on_pressed_changed(button_on_pressed_changed callback,void* state=nullptr)=0;
};

class basic_button final : public button {
    uint8_t m_pin;
    uint32_t m_debounce_ms;
    bool m_open_high;
    int m_pressed;
    button_on_pressed_changed m_on_pressed_changed;
    void* m_on_pressed_changed_state;
    uint32_t m_last_change_ms;
    basic_button(const basic_button& rhs) = delete;
    basic_button& operator=(const basic_button& rhs) = delete;
    void do_move(basic_button& rhs);
   public:
    basic_button(basic_button&& rhs);
    basic_button& operator=(basic_button&& rhs);
    basic_button(uint8_t pin, uint32_t debounce_ms=10,bool open_high = false);
    virtual bool initialized() const;
    virtual void initialize();
    virtual void deinitialize();
    virtual void on_pressed_changed(button_on_pressed_changed callback, void* state = nullptr);
    bool raw_pressed();
    virtual bool pressed();
    virtual void update();
};
template <uint8_t Pin, uint32_t DebounceMS = 10, bool OpenHigh = false, size_t Events = 8>
class int_button final : public button {
public:
    using type = int_button;
    constexpr static const uint8_t pin = Pin;
    constexpr static const uint32_t debounce_ms = DebounceMS;
    constexpr static const bool open_high = OpenHigh;
    constexpr static const size_t events_size = Events;
private:
    static type* m_this;
    typedef struct event_entry {
        uint32_t ms;
        int state;
    } event_entry_t;
    using event_buffer_t = data::circular_buffer<event_entry,events_size>;
    volatile int m_pressed;
    button_on_pressed_changed m_on_pressed_changed;
    void* m_on_pressed_changed_state;
    uint32_t m_last_change_ms;
    event_buffer_t m_events;
    int_button(const int_button& rhs) = delete;
    int_button& operator=(const int_button& rhs) = delete;
    static 
#if defined(IRAM_ATTR)
    IRAM_ATTR
#endif
    void process_change() {
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
    void do_move(int_button& rhs) {
        m_this = this;
        m_pressed = rhs.m_pressed;
        rhs.m_pressed = -1;
        m_on_pressed_changed = rhs.m_on_pressed_changed;
        rhs.m_on_pressed_changed = nullptr;
        m_on_pressed_changed_state = rhs.m_on_pressed_changed_state;
        m_last_change_ms = rhs.m_last_change_ms;
        while(!rhs.m_events.empty()) {
            event_entry_t e;
            rhs.m_events.get(&e);
            m_events.put(e);
        }
    }
   public:
    int_button(int_button&& rhs) {
        do_move(rhs);
    }
    int_button& operator=(int_button&& rhs) {
        do_move(rhs);
        return *this;
    }
    int_button() : m_pressed(-1) {

    } 
    virtual bool initialized() const {
        return m_pressed!=-1;
    }
    virtual void initialize() {
        if(!initialized()) {
            m_this = this;
            if(open_high) {
#if defined(INPUT_PULLUP)
                pinMode(pin, INPUT_PULLUP);
#else
                pinMode(pin, INPUT);
                digitalWrite(pin,HIGH);
#endif
            } else {
#if defined(INPUT_PULLDOWN)
                pinMode(pin, INPUT_PULLDOWN);
#else
                pinMode(pin, INPUT);
                digitalWrite(pin,LOW);
#endif
            }
            m_pressed = raw_pressed();
            attachInterrupt(digitalPinToInterrupt(pin),process_change,CHANGE);
        }
    }
    virtual void deinitialize() {
        if(initialized()) {
            detachInterrupt(digitalPinToInterrupt(pin));
            m_events.clear();   
            m_pressed = -1;
        }

    }
    virtual void on_pressed_changed(button_on_pressed_changed callback, void* state = nullptr) {
        m_on_pressed_changed = callback;
        m_on_pressed_changed_state = state;
    }
    bool raw_pressed() {
        return open_high?!digitalRead(pin):digitalRead(pin);
    }
    virtual bool pressed() { return m_pressed; }
    virtual void update() {
        if(m_on_pressed_changed==nullptr) {
            return;
        }
        while(!m_events.empty()) {
            event_entry_t e;
            m_events.get(&e);
            m_on_pressed_changed(e.state,m_on_pressed_changed_state);
        }
    }
};
template <uint8_t Pin, uint32_t DebounceMS , bool OpenHigh, size_t Events>
int_button<Pin, DebounceMS, OpenHigh, Events>*
int_button<Pin, DebounceMS, OpenHigh, Events>::m_this = nullptr;

typedef void (*multi_button_on_click)(int clicks,void* state);
typedef void (*multi_button_on_long_click)(void* state);
namespace multi_button_helpers {
    // implement std::move to limit dependencies on the STL, which may not be there
    template< class T > struct remove_reference      { typedef T type; };
    template< class T > struct remove_reference<T&>  { typedef T type; };
    template< class T > struct remove_reference<T&&> { typedef T type; };
    template <typename T>
    typename remove_reference<T>::type&& semantic_move(T&& arg) {
        return static_cast<typename remove_reference<T>::type&&>(arg);
    }
}
template<uint32_t DoubleClickMS = 250, uint32_t LongClickMS = 500, size_t Events = 32>
class multi_button_ex final : public button {
public:
    using type = multi_button_ex;
    constexpr static const uint32_t double_click_ms = DoubleClickMS;
    constexpr static const uint32_t long_click_ms = LongClickMS;
    constexpr static const size_t events_size = Events;
private:
    button& m_button;
    button_on_pressed_changed m_on_press_changed;
    void* m_on_press_changed_state;
    multi_button_on_click m_on_click;
    void* m_on_click_state;
    multi_button_on_long_click m_on_long_click;
    void* m_on_long_click_state;
    uint32_t m_last_change_ms;
    typedef struct event_entry {
        uint32_t ms;
        int state;
    } event_entry_t;
    using event_buffer_t = data::circular_buffer<event_entry,events_size>;
    event_buffer_t m_events;
    static void process_change(bool pressed, void* state) {
        multi_button_ex* pthis = (multi_button_ex*)state;
         if(!pthis->m_events.full()) {
            event_entry_t e;
            e.ms = millis();
            pthis->m_last_change_ms = e.ms;
            e.state = pressed;
            pthis->m_events.put(e);
        }
        if(pthis->m_on_press_changed!=nullptr) {
            pthis->m_on_press_changed(pressed,pthis->m_on_press_changed_state);
        }
    }
    void do_move(multi_button_ex& rhs) {
        m_button = multi_button_helpers::semantic_move(rhs.m_button);
        m_on_press_changed =rhs.m_on_press_changed;
        m_on_press_changed_state = rhs.m_on_press_changed_state;
        m_on_click = rhs.m_on_click;
        m_on_click_state = rhs.m_on_click_state;
        m_on_long_click = rhs.m_on_long_click;
        m_on_long_click_state = rhs.m_on_long_click_state;
        m_last_change_ms = rhs.m_last_change_ms;
        while(!rhs.m_events.empty()) {
            event_entry_t e;
            rhs.m_events.get(&e);
            m_events.put(e);
        }
    }
    multi_button_ex(const multi_button_ex& rhs)=delete;
    multi_button_ex& operator=(const multi_button_ex& rhs)=delete;
public:
    multi_button_ex(button& inner_button) : m_button(inner_button) {

    };
    multi_button_ex(multi_button_ex&& rhs) {
        do_move(rhs);
    }
    multi_button_ex& operator=(multi_button_ex&& rhs) {
        do_move(rhs);
        return *this;
    }
    virtual bool initialized() const {
        return m_button.initialized();
    }
    virtual void initialize() {
        if(!initialized()) {
            m_last_change_ms = 0;
            m_button.initialize();
            m_button.on_pressed_changed(process_change,this);
        }
    }
    virtual void deinitialize() {
        if(initialized()) {
            return m_button.deinitialize();
        }
    }
    virtual bool pressed() {
        return m_button.pressed();
    }
    virtual void on_pressed_changed(button_on_pressed_changed callback,void* state=nullptr) {
        m_on_press_changed = callback;
        m_on_press_changed_state = state;
    }
    virtual void update() {
        initialize();
        m_button.update();
        if(m_button.pressed()==1) {
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
    void on_click(multi_button_on_click callback, void* state = nullptr) {
        m_on_click = callback;
        m_on_click_state = state;
    }
    void on_long_click(multi_button_on_long_click callback, void* state = nullptr) {
        m_on_long_click = callback;
        m_on_long_click_state = state;
    }
};
using multi_button = multi_button_ex<>;

}  // namespace arduino