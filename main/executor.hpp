#pragma once
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/idf_additions.h"

#include <functional>
#include <algorithm>
#include <atomic>
#include <array>

#include "eventgroup.hpp"
#include "queue.hpp"

#include "thrdprio.hpp"


namespace executor {


    template<typename T>
    concept job_type = requires(T& j) {{j.execute()};} && std::is_trivially_copyable_v<T>;


    template<job_type JOB_t, std::size_t QUEUE_LEN = 8, unsigned TASKS = configNUMBER_OF_CORES>
    requires (QUEUE_LEN > 0 && 0 < TASKS && TASKS <= 32)
    class SimpleExecutor {
        public:
            static constexpr unsigned NUM_TASKS = TASKS;
            static constexpr unsigned NUM_CORES = configNUMBER_OF_CORES;
            static constexpr unsigned MAX_JOBS_IN_FLIGHT = QUEUE_LEN + NUM_TASKS;
        private:
            static constexpr union {
                    uint32_t u32 {0};
                    JOB_t job;
                } DUMMY {};

            std::array<TaskHandle_t,NUM_TASKS> tasks {};
            std::atomic<bool> terminateFlag {false};
            freertos::Queue<JOB_t,QUEUE_LEN> queue {};
        public:


            SimpleExecutor(void) = default;
            SimpleExecutor(const SimpleExecutor&) = delete;
            SimpleExecutor(SimpleExecutor&&) = delete;

            bool start(const UBaseType_t taskPrio = thrd::prio(5).of(10), const std::size_t stackSZ = (5*1024)) {
                asm ("":"+m" (tasks));
                for(TaskHandle_t& h : tasks) {
                    if(h != nullptr) {
                        return false; // Already/still running!
                    }
                }

                { 
                    bool allGood = true;
                    
                    for(unsigned i = 0; allGood && (i < TASKS); ++i) {
                        TaskHandle_t& h = tasks[i];

                        allGood &= xTaskCreatePinnedToCore(
                                &SimpleExecutor::exec_task,
                                "exec_task",
                                stackSZ,
                                (void*)this,
                                taskPrio,
                                &h,
                                i % NUM_CORES
                            ) == pdPASS;
                    }

                    if(!allGood) {
                        terminate();
                    }

                    return allGood;
                }
            }



            bool enqueue(const JOB_t& job, TickType_t maxWait = portMAX_DELAY) {
                return queue.enqueue(job, maxWait);
            }

            void terminate(void) {
                terminateFlag = true;

                queue.reset();

                asm ("":"+m" (tasks));
                for(TaskHandle_t& h : tasks) {
                    if(h != nullptr) {
                        // These dummy jobs will never actually be executed because terminateFlag is already set.
                        // However, we must provide _some_ data to 'wake up' tasks waiting on the queue.
                        enqueue( DUMMY.job, portMAX_DELAY );    
                    }
                }
                asm ("":"+m" (tasks));
                for(TaskHandle_t& h : tasks) {
                    while(h != nullptr) {
                        vTaskDelay(1);
                        asm("":"=m" (h));
                    }
                }

                terminateFlag = false;

                queue.reset();
            }

        protected:
            virtual void onJobFinished(JOB_t&& j) noexcept {

            }
        private:

            // struct Finisher {
            //     JOB_t& job;
            //     SimpleExecutor& ex;
            //     constexpr Finisher(SimpleExecutor& ex, JOB_t& job) :
            //         job {job},
            //         ex {ex} {

            //     }

            //     ~Finisher(void) {
            //         ex.onJobFinished(std::move(job));
            //     }
            // };

            void runInTask(void) {
                JOB_t j;
                while( queue.dequeue(j, portMAX_DELAY) && !terminateFlag ) {
                    j.execute();
                    onJobFinished(std::move(j));
                }
            }

            void removeTask(TaskHandle_t t) {
                asm ("":"+m" (tasks));
                for(TaskHandle_t& h : tasks) {
                    if(h == t) {
                        h = nullptr;
                        asm(""::"m" (h));
                        break;
                    }
                }
            }

            static void exec_task(void* arg) {
                SimpleExecutor& ex = *(SimpleExecutor*)arg;
                ex.runInTask();
                ex.removeTask(xTaskGetCurrentTaskHandle());
                vTaskDelete(nullptr);
                __builtin_unreachable();
            }

    };

} // namespace executor