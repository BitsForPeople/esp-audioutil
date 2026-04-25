#include <stdio.h>

#include <cstdint>
#include <array>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "tmr.hpp"

#include "simdutil.hpp"
#include "audioutil.hpp"


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


static constexpr uint32_t BUF_SAMPLE_CNT = 1024;

// For testing, use a defined alignment so that we can "simulate" any mis-alignment as needed:
alignas(16) static std::array<int16_t, BUF_SAMPLE_CNT> ch16_1 {};

alignas(16) static std::array<int32_t, BUF_SAMPLE_CNT> ch32 {};


template<typename S, std::size_t X>
static void initChannel(std::array<S,X>& ch, const int32_t& offset = 0) {
    int16_t* const s16 = (int16_t*)ch.data();
    const std::size_t CNT16 = (ch.size() * sizeof(int16_t)) / sizeof(S);
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
static bool verify(const std::array<S,X>& ch, const F& fexp) {
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

/**
 * @brief Short delay to let the system finish any background activities (logging...) before doing timing measurements.
 * 
 */
static void dly(void) {
    vTaskDelay(50/portTICK_PERIOD_MS);
}



extern "C" void app_main(void) {
    vTaskDelay(1000/portTICK_PERIOD_MS);

    {
        initChannel(ch16_1);
        // initChannel(ch16_2);
        clearChannel(ch32);

        const auto func = [](const uint32_t ix) -> int32_t {
            return (((uint32_t)ix << 16) | ix);
        };

        bool ok;

        Tmr tmr {};
        uint32_t t = -1;


        for(unsigned i = 0; i < _v(3); ++i) {
            touch(ch16_1);
            tmr.start();
            audio::Utils<arch::SoC::OTHER>::monoToStereo(ch16_1.data(), ch32.data(), _v(BUF_SAMPLE_CNT));
            t = min(t,tmr.stop());
            touch(ch32);
        }

        ok = verify(ch32, func);

        ESP_LOGI(TAG, "audio::utils::monoToStereo (scalar, %" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
            BUF_SAMPLE_CNT,
            t);
        if(ok) {
            ESP_LOGI(TAG, "Verified good.");
        }



        dly();

        for(unsigned i = 0; i < _v(3); ++i) {
            touch(ch16_1);
            tmr.start();
            audio::utils::monoToStereo(ch16_1.data(), ch32.data(), _v(BUF_SAMPLE_CNT));
            t = min(t,tmr.stop());
            touch(ch32);
        }

        ok = verify(ch32, func);

        ESP_LOGI(TAG, "audio::utils::monoToStereo (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
            BUF_SAMPLE_CNT,
            t);
        if(ok) {
            ESP_LOGI(TAG, "Verified good.");
        }



        dly();

        static constexpr int32_t FILL_VALUE = -1;

        fillChannel(ch32,FILL_VALUE);

        // Offset both input and output to make them mis-aligned:
        static constexpr uint32_t OFF_IN = 3;
        static constexpr uint32_t OFF_OUT = 1;
        static constexpr uint32_t CNT = BUF_SAMPLE_CNT - 2*(max(OFF_IN,OFF_OUT));

        const int16_t* const in16 = &ch16_1[OFF_IN];
        int32_t* const out32 = &ch32[OFF_OUT];

        t = -1;

        for(unsigned i = 0; i < _v(3); ++i) {
            touch(ch16_1);
            tmr.start();
            audio::utils::monoToStereo(in16, out32, _v(CNT));
            t = min(t,tmr.stop());
            touch(ch32);
        }

        ok = verify(ch32, [&func](const uint32_t ix) -> int32_t {
            if(ix < OFF_OUT || ix >= (OFF_OUT+CNT)) {
                return FILL_VALUE;
            } else {
                const uint32_t in_ix = ix - OFF_OUT + OFF_IN;
                return func(in_ix);
            }
        });

        ESP_LOGI(TAG, "audio::utils::monoToStereo (%" PRIu32 " samples, double-unaligned) : %" PRIu32 " cycles",
            CNT,
            t);
        if(ok) {
            ESP_LOGI(TAG, "Verified good.");
        }


    }

    {

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


        dly();

        bool ok;
        Tmr tmr {};

        uint32_t t = -1;
        for(unsigned i = 0; i < _v(3); ++i) {
            touch(in);
            tmr.start();
            audio::Utils<arch::SoC::OTHER>::reduce32to16(in.data(), out.data(), _v(BUF_SAMPLE_CNT), _v(GAIN));
            t = min(t,tmr.stop());
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
            t = min(t,tmr.stop());
            touch(out);
        }

        ESP_LOGI(TAG, "audio::utils::reduce32to16 (%" PRIu32 " samples, double-aligned) : %" PRIu32 " cycles",
            BUF_SAMPLE_CNT,
             t);

        if(verify(ch16_1, func)) {
            ESP_LOGI(TAG, "Verified good.");
        }


        static constexpr int32_t FILL_VALUE = -1;
        fillChannel(ch16_1,FILL_VALUE);

        // Offset both input and output to make them mis-aligned:
        static constexpr uint32_t OFF_IN = 1;
        static constexpr uint32_t OFF_OUT = 3;
        static constexpr uint32_t CNT = BUF_SAMPLE_CNT - 2*(max(OFF_IN,OFF_OUT));

        const int32_t* const in32 = &in[OFF_IN];
        int16_t* const out16 = &out[OFF_OUT];

        t = -1;
        for(unsigned i = 0; i < _v(3); ++i) {
            touch(in);
            tmr.start();
            audio::utils::reduce32to16(in32, out16, _v(CNT), _v(GAIN));
            t = min(t,tmr.stop());
            touch(out);
        }

        ESP_LOGI(TAG, "audio::utils::reduce32to16 (%" PRIu32 " samples, double-unaligned) : %" PRIu32 " cycles",
            CNT,
            t);

        ok = verify(out, [&func](const uint32_t ix) -> int32_t {
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

}