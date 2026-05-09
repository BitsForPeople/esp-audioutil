#include <stdio.h>

#include <cstdint>
#include <array>
#include <algorithm>
#include <bit>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_rom_sys.h>
#include <esp_random.h>
#include <esp_log.h>

#include "tmr.hpp"

#include "audioutil.hpp"
#include "sintable.hpp"

#include "executor.hpp"
#include "thrdprio.hpp"



#define NI __attribute__((noinline))

static constexpr const char* TAG = "main";

/**
 * @brief "Optimization barrier" for the compiler.
 * "Consumes" the given value and "produces" an identical value.
 * 
 * @tparam T 
 */
template<typename T>
static inline T __attribute__((always_inline)) _v(T x) noexcept {
    asm volatile ("": "+r" (x));
    return x;
}

/**
 * @brief "Optimization barrier": consumes+produces data from/to the given array.
 * 
 * @tparam S 
 * @tparam X 
 * @param ch 
 */
template<typename S, std::size_t X>
static inline void touch(std::array<S,X>& arr) {
    asm volatile ("":"+m" (arr));
}


static constexpr uint32_t BUF_SAMPLE_CNT = 4800;

// For testing, use a defined alignment so that we can "simulate" any mis-alignment as needed:
alignas(16) static std::array<int16_t, BUF_SAMPLE_CNT> ch16_1 {};
alignas(16) static std::array<int16_t, BUF_SAMPLE_CNT> ch16_2 {};

alignas(16) static std::array<int32_t, BUF_SAMPLE_CNT> ch32 {};


template<std::size_t L, typename E, std::size_t N>
requires (N != std::dynamic_extent && L <= N)
static std::span<E,N-L> trimFirst(const std::span<E,N>& s) {
    // return std::span<E,N-L> {s.data()+L,N-L};
    return s.template last<N-L>();
}

template<std::size_t L, typename E, std::size_t N>
requires (N != std::dynamic_extent && L <= N)
static std::span<E,N-L> trimLast(const std::span<E,N>& s) {
    // return std::span<E,N-L> {s.data(),N-L};
    return s.template first<N-L>();
}


template<std::size_t OFF, typename E, std::size_t N>
requires (OFF < N)
static std::span<E,N-OFF-1> misalignedBy(std::array<E,N>& arr) {
    return std::span<E,N-OFF-1> {&arr[OFF],N-OFF-1};
}


template<typename S, std::size_t X>
static void initChannel(std::array<S,X>& ch, const int32_t& offset = 0) {
    int16_t* const s16 = (int16_t*)ch.data();
    const std::size_t CNT16 = (ch.size() * sizeof(S)) / sizeof(int16_t);
    for(unsigned i = 0; i < CNT16; ++i) {
        s16[i] = i + offset;
    }
}

template<typename S, std::size_t X, typename V>
static constexpr void fillChannel(std::array<S,X>& ch, const V& value) {
    for(S& s : ch) {
        s = value;
    }
}

template<typename S, std::size_t X>
static constexpr void clearChannel(std::array<S,X>& ch) {
    fillChannel(ch,0);
}

static constexpr uint32_t min(const uint32_t a, const uint32_t b) {
    return (a<b) ? a : b;
}

static constexpr uint32_t max(const uint32_t a, const uint32_t b) {
    return (a>b) ? a : b;
}

template<typename S, std::size_t X, typename F>
static bool verify(const std::span<S,X>& ch, const F& fexp) {
    bool ok = true;
    uint32_t ix;
    for(ix = 0; ok && ix < ch.size(); ++ix) {
        const S exp = std::invoke(fexp,ix);
        ok = (ch[ix] == exp);
    }
    if(!ok) {
        ESP_LOGE(TAG, "Verify failed @ ix=%" PRIu32, ix-1);
        const uint32_t first = ix - min(ix,10);
        const uint32_t last = ix + min((ch.size() - ix), 10);
        for(uint32_t i = first; i < last; ++i ) {
            ESP_LOGE(TAG, "%" PRIu32 ": exp=%" PRId32 ", act=%" PRId32,
                i,
                (int32_t)std::invoke(fexp,i),
                (int32_t)ch[i]
                );
        }
    }
    return ok;
}


template<typename S, std::size_t X, typename F>
static bool verify(const std::array<S,X>& ch, const F& fexp) {
    return verify( std::span {ch}, fexp);
}



/**
 * @brief Short delay to let the system finish any background activities (logging...) before doing timing measurements.
 * 
 */
static void dly(void) {
    vTaskDelay(50/portTICK_PERIOD_MS);
}


static void test_stereoToMono(void) {

    auto& in = ch16_1;
    auto& out = ch16_2;
    initChannel(in);
    clearChannel(out);

    const auto func = [](const uint32_t ix) -> int16_t {
        return (ix*2 + (ix*2+1))/2;
    };

    constexpr uint32_t CNT = BUF_SAMPLE_CNT / 2;

    const std::span<int16_t,CNT> result {out.data(),CNT};

    Tmr tmr {};
    uint32_t t = -1;

    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::Utils<arch::SoC::OTHER>::stereoToMono(in.data(), out.data(), _v(CNT));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::stereoToMono (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        CNT,
        t);

    if(verify(result, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }

    dly();



    clearChannel(out);
    t = -1;

    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::stereoToMono(in.data(), out.data(), _v(CNT));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::stereoToMono (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        CNT,
        t);

    if(verify(result, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }

    dly();

}

static void test_monoToStereo(void) {

    auto& in = ch16_1;
    auto& out = ch32;
    initChannel(in);
    clearChannel(out);

    const auto func = [](const uint32_t ix) -> int32_t {
        return (((uint32_t)ix << 16) | ix);
    };

    Tmr tmr {};
    uint32_t t = -1;


    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::Utils<arch::SoC::OTHER>::monoToStereo(in.data(), out.data(), _v(BUF_SAMPLE_CNT));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::monoToStereo (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        BUF_SAMPLE_CNT,
        t);

    if(verify(out, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }



    dly();

    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::monoToStereo(in.data(), out.data(), _v(BUF_SAMPLE_CNT));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::monoToStereo (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        BUF_SAMPLE_CNT,
        t);

    if(verify(out, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }



    dly();

    static constexpr int32_t FILL_VALUE = -1;

    fillChannel(out,FILL_VALUE);

    // Offset both input and output to make them mis-aligned:
    static constexpr uint32_t OFF_IN = 3;
    static constexpr uint32_t OFF_OUT = 1;
    // static constexpr uint32_t CNT = BUF_SAMPLE_CNT - max(OFF_IN,OFF_OUT) - 1;

    // const int16_t* const in_off = &in[OFF_IN];
    // int32_t* const out_off = &out[OFF_OUT];

    std::span in_off = trimFirst<OFF_IN>(trimLast<1>(std::span {in}));
    std::span out_off = trimFirst<OFF_OUT>(trimLast<1>(std::span {out}));

    const uint32_t cnt = min(in_off.size(),out_off.size());


    t = -1;

    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::monoToStereo(in_off.data(), out_off.data(), _v(cnt));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::monoToStereo (%" PRIu32 " samples, double-unaligned) : %" PRIu32 " cycles",
        cnt,
        t);

    const bool ok = verify(out, [&func,&cnt](const uint32_t ix) -> int32_t {
        if(ix < OFF_OUT || ix >= (OFF_OUT+cnt)) {
            return FILL_VALUE;
        } else {
            const uint32_t in_ix = ix - OFF_OUT + OFF_IN;
            return func(in_ix);
        }
    });

    if(ok) {
        ESP_LOGI(TAG, "Verified good.");
    }

    dly();

}

static void test_reduce32to16(void) {

    auto& in = ch32;
    auto& out = ch16_1;
    
    for(uint32_t i = 0; i < BUF_SAMPLE_CNT; ++i) {
        in[i] = (i << 16);
    }
    clearChannel(out);

    static constexpr uint32_t GAIN = audio::fp::asQ16(1.5f);

    const auto func = [](const uint32_t ix) -> int32_t {
        return ((uint64_t)(ix << 16) * GAIN) >> (16+16);
    };


    Tmr tmr {};

    uint32_t t = -1;
    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::Utils<arch::SoC::OTHER>::reduce32to16(in.data(), out.data(), _v(BUF_SAMPLE_CNT), _v(GAIN));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::reduce32to16 (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        BUF_SAMPLE_CNT,
        t);

    if(verify(out, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }


    dly();

    t = -1;
    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::reduce32to16(in.data(), out.data(), _v(BUF_SAMPLE_CNT), _v(GAIN));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::reduce32to16 (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        BUF_SAMPLE_CNT,
            t);

    if(verify(ch16_1, func)) {
        ESP_LOGI(TAG, "Verified good.");
    }


    static constexpr int32_t FILL_VALUE = -1;
    fillChannel(out,FILL_VALUE);

    // Offset both input and output to make them mis-aligned:
    static constexpr uint32_t OFF_IN = 1;
    static constexpr uint32_t OFF_OUT = 3;
    static constexpr uint32_t CNT = BUF_SAMPLE_CNT - max(OFF_IN,OFF_OUT) - 1;

    const int32_t* const in_off = &in[OFF_IN];
    int16_t* const out_off = &out[OFF_OUT];

    t = -1;
    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::reduce32to16(in_off, out_off, _v(CNT), _v(GAIN));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::reduce32to16 (%" PRIu32 " samples, double-unaligned) : %" PRIu32 " cycles",
        CNT,
        t);

    const bool ok = verify(out, [&func](const uint32_t ix) -> int32_t {
        if(ix < OFF_OUT || ix >= (OFF_OUT+CNT)) {
            return FILL_VALUE;
        } else {
            const uint32_t in_ix = ix - OFF_OUT + OFF_IN;
            return func(in_ix);
        }
    });

    if(ok) {
        ESP_LOGI(TAG, "Verified good.");
    }

}

static void test_linearRamp(void) {
    constexpr uint32_t CNT = BUF_SAMPLE_CNT;
    constexpr unsigned FRAC_BITS = 31;
    auto& in = ch16_1;
    auto& out = ch16_2;

    fillChannel(in, 10000);
    clearChannel(out);

    const uint32_t volStart = audio::fp::asQ<FRAC_BITS>(0.5f);
    const uint32_t volEnd = audio::fp::asQ<FRAC_BITS>(0.75f);
    const int32_t volStep = (volEnd - volStart)/CNT;

    Tmr tmr {};

    uint32_t t = -1;
    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::Utils<arch::SoC::OTHER>::linearRamp<FRAC_BITS>(in.data(), out.data(), _v(CNT), _v(volStart), _v(volStep));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::linearRamp (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        CNT,
        t);

    dly();

    t = -1;
    for(unsigned i = 0; i < _v(3); ++i) {
        touch(in);
        tmr.start();
        audio::utils::linearRamp<FRAC_BITS>(in.data(), out.data(), _v(CNT), _v(volStart), _v(volStep));
        tmr.stop(t);
        touch(out);
    }

    ESP_LOGI(TAG, "audio::utils::linearRamp (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
        CNT,
        t);


    // const uint32_t mid = CNT/2;

    // for(uint32_t i = mid-10; i < mid+10; ++i) {
    //     ESP_LOGI(TAG, "env[%" PRIu32 "]: %" PRId16, i, out[i]);
    // }

}


static void test_applyGain(const uint32_t gainFP, const uint32_t frac_bits) {
    constexpr uint32_t CNT = BUF_SAMPLE_CNT;

    auto& in = ch16_1;
    auto& out = ch16_2;

    initChannel(in);
    clearChannel(out);

    ESP_LOGI(TAG, "applyGain %.2f", (float)gainFP / (1 << frac_bits));

    const auto func = [gainFP,frac_bits](const uint32_t ix) -> int32_t {
        return audio::utils::clamps<16>((ix*gainFP) >> frac_bits);
    };

    dly();

    Tmr tmr {};

    uint32_t t = -1;
    {

        for(unsigned i = 0; i < _v(3); ++i) {
            touch(in);
            tmr.start();
            audio::Utils<arch::SoC::OTHER>::applyGain(in.data(), out.data(), _v(CNT), _v(gainFP), _v(frac_bits));
            tmr.stop(t);
            touch(out);
        }

        ESP_LOGI(TAG, "audio::utils::applyGain (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
            CNT,
            t);

        if(verify(out, func)) {
            ESP_LOGI(TAG, "Verified good.");
        }

    
        clearChannel(out);
        dly();

        t = -1;
        for(unsigned i = 0; i < _v(3); ++i) {
            touch(in);
            tmr.start();
            audio::utils::applyGain(in.data(), out.data(), _v(CNT), _v(gainFP), _v(frac_bits));
            tmr.stop(t);
            touch(out);
        }

        ESP_LOGI(TAG, "audio::utils::applyGain (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
            CNT,
            t);

        if(verify(out, func)) {
            ESP_LOGI(TAG, "Verified good.");
        }

    }

    {

        static constexpr int32_t FILL_VALUE = -1;

        fillChannel(out,FILL_VALUE);

        // Offset both input and output to make them mis-aligned:
        static constexpr uint32_t OFF_IN = 3;
        static constexpr uint32_t OFF_OUT = 1;

        std::span in_off = trimFirst<OFF_IN>(trimLast<1>(std::span {in}));
        std::span out_off = trimFirst<OFF_OUT>(trimLast<1>(std::span {out}));

        const uint32_t cnt = min(in_off.size(),out_off.size());

        t = -1;
        for(unsigned i = 0; i < _v(3); ++i) {
            touch(in);
            tmr.start();
            audio::utils::applyGain(in_off.data(), out_off.data(), _v(cnt), _v(gainFP), _v(frac_bits));
            tmr.stop(t);
            touch(out);
        }

        ESP_LOGI(TAG, "audio::utils::applyGain (%" PRIu32 " samples, double-unaligned) : %" PRIu32 " cycles",
            cnt,
            t);

        const auto f2 = [cnt,&func](const uint32_t ix) -> int32_t {
            if(ix < OFF_OUT || ix >= (OFF_OUT+cnt)) {
                return FILL_VALUE;
            } else {
                const uint32_t in_ix = ix - OFF_OUT + OFF_IN;
                return func(in_ix);
            }
        };

        if(verify(out, f2)) {
            ESP_LOGI(TAG, "Verified good.");
        }
    }


    // const uint32_t mid = CNT/2;

    // for(uint32_t i = mid-10; i < mid+10; ++i) {
    //     ESP_LOGI(TAG, "gain[%" PRIu32 "]: %" PRId16, i, out[i]);
    // }

}


constexpr audio::QuarterSinTable<4096> SIN {};

struct Job {
    uint32_t id {};
    constexpr Job(void) = default;
    constexpr Job(const uint32_t id) : id{id} {

    }

    void execute(void) {
        // const uint32_t cid = xt_utils_get_core_id();
        const uint32_t time = 1000 + (id%3) * 500; // Vary the time this job will by busy
        // ESP_LOGI(TAG, "Job %" PRIu32 " on core %" PRIu32 " (%" PRIu32 "us)", id,cid,time);
        esp_rom_delay_us(time);
    }
};

template<std::size_t QUEUE_LEN = 8>
requires(executor::SimpleExecutor<Job,QUEUE_LEN>::MAX_JOBS_IN_FLIGHT <= freertos::EventGroup::EVT_BIT_CNT)
class Executor : public executor::SimpleExecutor<Job,QUEUE_LEN> {
    private:
    public:
        freertos::EventGroup evtGrp {};
        std::array<volatile uint32_t,executor::SimpleExecutor<Job,QUEUE_LEN>::NUM_CORES> jobCnt {};
    protected:
        void onJobFinished(Job&& j) noexcept override final {
            volatile uint32_t& cnt = jobCnt[xt_utils_get_core_id()];
            cnt = cnt + 1;
            evtGrp.setBits(1u << (j.id % evtGrp.EVT_BIT_CNT));
        }
};

static Executor<16> exec {};

static inline void test_executor(void) {

    constexpr unsigned JOB_CNT = 32;

    ESP_LOGI(TAG, "Test job executor executing %u jobs using %u task(s) on %u core(s).",
        JOB_CNT, exec.NUM_TASKS, exec.NUM_CORES);

    if(!exec.start(thrd::prio(3).of(10))) {
        ESP_LOGE(TAG, "Failed to start executor!");
    } else {
        exec.evtGrp.clearBits();

        uint32_t jobsFinished {0};
        
        for(uint32_t i = 0; i < JOB_CNT; ++i) {
            while(!exec.enqueue(Job {i},0)) {
                // queue full. Wait for one or more jobs to complete, then try again.
                uint32_t bits = exec.evtGrp.waitForAnyBit();
                jobsFinished += std::popcount(bits);
                // Logging here *really* messes up the current core's job performance...
                // ESP_LOGI(TAG, "%" PRIu32 " jobs finished.", jobsFinished);
            }
        }

        // ESP_LOGI(TAG, "All jobs enqueued.");

        while(jobsFinished < JOB_CNT) {
            uint32_t bits = exec.evtGrp.waitForAnyBit();
            jobsFinished += std::popcount(bits);
            // ESP_LOGI(TAG, "%" PRIu32 " jobs finished.", jobsFinished);
        }

        ESP_LOGI(TAG, "All jobs finished.");

        exec.terminate();

        for(unsigned c = 0; c < exec.jobCnt.size(); ++c) {
            ESP_LOGI(TAG, "Jobs finished on core #%u: %" PRIu32, c, exec.jobCnt[c]);
        }
    }

}

static inline void main_task(void*) {
    vTaskDelay(1000/portTICK_PERIOD_MS);

    test_executor();

    dly();

    test_stereoToMono();

    dly();

    test_monoToStereo();

    dly();

    test_reduce32to16();

    dly();

    test_linearRamp();

    dly();

    test_applyGain(audio::fp::asQ<12>(0.9f),12); // Stay within 15 significant bits to avoid rounding differences.

    dly();

    test_applyGain(audio::fp::asQ<12>(1.1f),12); // Stay within 15 significant bits to avoid rounding differences.

    vTaskDelete(nullptr);
}

extern "C" void app_main(void) {

    // Default priority for the main task is only 1! 
    // Run main task with higher-than-default priority:
    xTaskCreate(main_task,
        "main_task",
        CONFIG_ESP_MAIN_TASK_STACK_SIZE+512,
        nullptr,
        thrd::prio(6).of(10),
        nullptr);


}