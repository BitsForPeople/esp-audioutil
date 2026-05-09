#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace freertos {
    
    class TickInterval {
        public:
            constexpr TickInterval(void) = default;
            constexpr TickInterval(TickType_t interval) :
                m_tstart {0},
                m_interval {interval}
            {

            }

            constexpr bool expired(void) const {
                return ticksLeft() == 0;
            }

            constexpr TickInterval& setInterval(TickType_t interval) {
                m_interval = interval;
                return *this;
            }
            constexpr TickType_t getInterval(void) const {
                return m_interval;
            }

            constexpr TickType_t getTStart(void) const {
                return m_tstart;
            }

            TickInterval& restart(void) {
                m_tstart = xTaskGetTickCount();
                return *this;
            }

            TickType_t ticksLeft(void) const {
                const TickType_t now = xTaskGetTickCount();
                const TickType_t dt = now - m_tstart;
                return m_interval - ((dt < m_interval) ? dt : m_interval);
                // if(dt < m_interval) {
                //     return m_interval - dt;
                // } else {
                //     return 0;
                // }
            }

            TickInterval& expireNow(void) {
                m_tstart = xTaskGetTickCount() - m_interval;
                return *this;
            }

        private:
            TickType_t m_tstart {0};
            TickType_t m_interval {0};
    };
}