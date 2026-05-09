#pragma once
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


namespace freertos {

    class Lck {
        public:
            static Lck take(SemaphoreHandle_t mux, const TickType_t maxWait = portMAX_DELAY) {
                if(mux) {
                    const bool acq = xSemaphoreTake(mux,maxWait) != pdFALSE;
                    return Lck {mux,acq};
                } else {
                    return Lck {};
                }
            }

            constexpr Lck() = default;
            
            constexpr Lck(SemaphoreHandle_t mux, bool acquired) : mux {mux}, acq {acquired && (mux != nullptr)} {

            }

            constexpr Lck(Lck&& other) : mux {other.mux}, acq {other.acq} {
                other.mux = nullptr;
                other.acq = false;
            }

            constexpr Lck(const Lck&) = delete;

            constexpr Lck& operator =(const Lck&) = delete;
            constexpr Lck& operator =(Lck&& other) {
                this->release();

                this->mux = other.mux;
                this->acq = other.acq;

                other.mux = nullptr;
                other.acq = false;

                return *this;
            }

            constexpr explicit operator bool() const {
                return acquired();
            }

            constexpr bool acquired() const {
                return acq;
            }

            void release() {
                if(acq) {
                    if(mux) {
                        xSemaphoreGive(mux);
                    }
                    this->acq = false;
                }
            }

            ~Lck() {
                if(acq && mux) {
                    xSemaphoreGive(mux);
                }
            }
        private:
            SemaphoreHandle_t mux {nullptr};
            bool acq {false};
    };

    class Mutex {
        public:
            Mutex() {
                handle = xSemaphoreCreateMutexStatic(&mutexMem);        
            }

            Mutex(const Mutex&) = delete;
            Mutex(Mutex&&) = delete;

            SemaphoreHandle_t getHandle(void) const {
                return this->handle;
            }

            operator SemaphoreHandle_t(void) const {
                return getHandle();
            }

            [[nodiscard]]
            Lck acquire(const TickType_t maxWait = portMAX_DELAY) {
                return Lck::take(handle,maxWait);
            }


            bool take(const TickType_t maxWait = portMAX_DELAY) {
                return xSemaphoreTake(handle,maxWait);
            }

            bool give(void) {
                return xSemaphoreGive(handle);
            }

            template<typename F, typename ... Args>
            auto perform(F&& op, Args&&...args) {
                Lck lck {acquire()};
                return std::invoke(op,std::forward<Args>(args)...);
            }


            bool takeFromISR(void) {
                BaseType_t dummy;
                return xSemaphoreTakeFromISR(handle, &dummy);
            }

            bool giveFromISR(BaseType_t& should_yield) {
                return xSemaphoreGiveFromISR(handle,&should_yield);
            }

        private:
            SemaphoreHandle_t handle {nullptr};
            StaticSemaphore_t mutexMem {};
    };



}