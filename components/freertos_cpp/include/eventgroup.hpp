#pragma once
#include <limits>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

namespace freertos {

    class EventGroup {
        public:
        static constexpr unsigned EVT_BITS_RESERVED = 8; // Used internally by FreeRTOS.
        static constexpr unsigned EVT_BIT_CNT = (sizeof(EventBits_t)*8) - EVT_BITS_RESERVED;
        // (configUSE_16_BIT_TICKS) ? 8 : 24;
        static constexpr EventBits_t ALL_BITS = std::numeric_limits<EventBits_t>::max() >> EVT_BITS_RESERVED;
        
        EventGroup(void) {
            hdl = xEventGroupCreateStatic(&mem);
        }
        EventGroup(const EventGroup&) = delete;
        EventGroup(EventGroup&&) = delete;
        EventGroup& operator =(const EventGroup&) = delete;
        EventGroup& operator =(EventGroup&&) = delete;

        EventBits_t setBits(const EventBits_t bits) {
            return xEventGroupSetBits(hdl,bits);
        }

        EventBits_t clearBits(const EventBits_t bits = ALL_BITS) {
            return xEventGroupClearBits(hdl,bits);
        }

        EventGroupHandle_t handle(void) const {
            return hdl;
        }

        EventBits_t getBits(void) {
            return xEventGroupGetBits(hdl);
        }

        EventBits_t waitForBits(const EventBits_t bits, const TickType_t maxWait = portMAX_DELAY) {
            return xEventGroupWaitBits(hdl,bits,pdTRUE,pdTRUE,maxWait);
        }

        EventBits_t waitForAnyBit(const EventBits_t bits = ALL_BITS,
            const TickType_t maxWait = portMAX_DELAY) {
            return xEventGroupWaitBits(hdl,bits,pdTRUE,pdFALSE,maxWait);
        }

        EventBits_t sync(const EventBits_t bitsToSet,
            const EventBits_t bitsToWaitFor,
            const TickType_t maxWait = portMAX_DELAY ) {
            return xEventGroupSync(hdl, bitsToSet, bitsToWaitFor, maxWait);
        }

        private:
        StaticEventGroup_t mem {};
        EventGroupHandle_t hdl;
    };

} // namespace freertos