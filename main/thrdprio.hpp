#pragma once
#include <freertos/FreeRTOS.h>
#include <cstdint>

namespace thrd {

    static constexpr std::size_t MIN_STACK_SIZE = configMINIMAL_STACK_SIZE;
    static constexpr std::size_t DEFAULT_STACK_SIZE = MIN_STACK_SIZE + 2048; 

    struct Prio {
        static constexpr UBaseType_t HIGHEST     = configMAX_PRIORITIES-1;    
        static constexpr UBaseType_t HIGH        = (3*HIGHEST) / 4;  
        static constexpr UBaseType_t MEDIUM      = (2*HIGHEST) / 4;                  
        static constexpr UBaseType_t LOW         = (1*HIGHEST) / 4;
        static constexpr UBaseType_t LOWEST      = 0; 
        static constexpr UBaseType_t IDLE        = LOWEST;



        template<uint32_t SCALE>
        requires (SCALE >= 1)
        static constexpr UBaseType_t base(const uint32_t prioValue) noexcept {
            // assert(prioValue <= SCALE);
            return ((prioValue * HIGHEST) + (SCALE/2)) / SCALE;
        }

        template<uint32_t PRIO>
        requires (PRIO <= 10)
        static constexpr UBaseType_t base10 = base<10>(PRIO);

        struct PrioVal {
            uint32_t value; // The unscaled priority value
            constexpr operator uint32_t() const noexcept {
                return base<10>(this->value);      
            }
        };    

        PrioVal of10;

        constexpr Prio(const uint32_t prioValue) noexcept :
            of10 {prioValue}
        {
            
        }

        constexpr operator UBaseType_t() const noexcept {
            return of10.value;
        }

        template<uint8_t SCALE>
        constexpr UBaseType_t of() const noexcept {
            return this->of(SCALE);
        }

        constexpr UBaseType_t of(const uint32_t scale) const noexcept {
            return ((this->of10.value * HIGHEST) + (scale/2)) / scale;
        }
    
    };   

    static constexpr Prio prio(const uint32_t value) noexcept {
        return Prio {value};
    }
    
} // namespace thrd