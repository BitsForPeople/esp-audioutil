#pragma once
#include <cstdint>
#include <type_traits>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esp_error.hpp"

namespace freertos {

    template<typename E, std::size_t MAX_LEN> // , std::enable_if_t<std::is_trivially_copyable_v<E>, bool> = true>
    requires requires { std::is_trivially_copyable_v<E>; }
    class Queue {

        public:

        using item_t = E;
        static constexpr std::size_t capacity = MAX_LEN;

         Queue() :
            m_handle {xQueueCreateStatic(capacity, sizeof(item_t), queue_mem, &static_queue)} {
                if(m_handle == nullptr) {
                    esp::err::doThrow(esp::err::esp_error {ESP_FAIL, "Error intializing queue."});
                }
            }

         ~Queue() noexcept {
            if(m_handle != nullptr) [[likely]] {
                vQueueDelete(m_handle);
            }
        }

        inline bool enqueue(const item_t& item, const TickType_t maxwait = portMAX_DELAY ) noexcept {
            return xQueueSendToBack(m_handle, &item, maxwait) != pdFAIL;
        }

        inline bool enqueueISR(const item_t& item, BaseType_t* const shouldSwitch) noexcept {
            return xQueueSendToBackFromISR(m_handle,&item,shouldSwitch) != pdFAIL;
        }

        // inline bool enqueueOverwrite(const item_t& item) noexcept {
        //     return xQueueOverwrite(m_handle, &item) != pdFAIL;
        // }

        // inline bool enqueueOverwriteISR(const item_t& item, BaseType_t* const shouldSwitch) noexcept {
        //     return xQueueOverwriteFromISR(m_handle, &item, shouldSwitch) != pdFAIL;
        // }

        inline bool dequeue(item_t& item, const TickType_t maxwait = portMAX_DELAY) noexcept {
            return xQueueReceive(m_handle, &item, maxwait) != pdFALSE;
        }

        inline bool dequeueISR(item_t& item, BaseType_t& shouldYield) noexcept {
            return dequeueISR(item,&shouldYield);
        }
        
        inline bool dequeueISR(item_t& item, BaseType_t* const shouldYield) noexcept {
            if ( xQueueIsQueueEmptyFromISR(m_handle) != pdFALSE ) {
                return xQueueReceiveFromISR(m_handle, &item, shouldYield) != pdFALSE;
            } else {
                return false;
            }
        }

        inline void reset() noexcept {
            xQueueReset(m_handle);
        }

        private:
            QueueHandle_t m_handle;
            StaticQueue_t static_queue;
            uint8_t queue_mem[capacity*sizeof(item_t)]; // No alignment required, data is memcopied in and out

    };  

} // namespace freertos