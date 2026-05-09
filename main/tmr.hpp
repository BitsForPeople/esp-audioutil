#pragma once
#include <cstdint>

#include <esp_cpu.h>

// template<typename T>
// inline T __attribute__((always_inline)) _v(T value) {
//     asm volatile ("" : "+r" (value));
//     return value;
// }
// template<typename T>
// inline void __attribute__((always_inline)) _consume(T value) {
//     asm volatile ("":: "r" (value));
// }

class Tmr {
    public:
        constexpr Tmr() noexcept = default;
        inline void start() noexcept {
            t_start = esp_cpu_get_cycle_count();
        }

        inline uint32_t stop() const noexcept {
            return esp_cpu_get_cycle_count() - t_start;
        }

        inline uint32_t stop(uint32_t& t) const noexcept {
            const uint32_t dt = stop();
            t = (t < dt) ? t : dt;
            return t;
        }
        
    private:
        uint32_t t_start {0};
};