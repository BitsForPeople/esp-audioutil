#pragma once
/*
 * Copyright 2026, <https://github.com/BitsForPeople>
 */
#include <cstdint>
#include <cmath> // ldexpf
#include <algorithm> // std::min
#include <concepts>
#include <limits>
#include <span>
#include <cstring>

#include "arch.hpp"
#include "simdutil.hpp"

#include <esp_log.h>

namespace audio {

    namespace fp {

        template<unsigned FRAC_BITS>
        requires (FRAC_BITS < 32)
        static inline constexpr uint32_t asQ(const float f) noexcept {
            if (std::is_constant_evaluated()) {
                return (uint32_t)(f * ((uint32_t)1 << FRAC_BITS));
            } else {
                return (uint32_t)std::ldexpf(f,FRAC_BITS); // might not be constexpr prior to C++23
            }
        }

        static inline constexpr uint32_t asQ16(const float f) noexcept {
            return asQ<16>(f);
        }

    }

    static constexpr uint32_t UNITY_GAIN = fp::asQ16(1.0f);

    /**
     * @brief Basic C++ implementation of the utility functions as fallback when we don't have specialized variants
     * for our current target SoC.
     *
     * @tparam SOC The target SoC for which we're building.
     */
    template<arch::SoC SOC>
    struct Utils {

        /**
         * @brief Clears a memory range to all-0.
         *
         * @tparam D
         * @tparam X
         *
         * @param mem std::span of the memory to clear
         */
        template<typename D, std::size_t X>
        requires (!std::is_const_v<D>)
        static inline void clr(const std::span<D,X>& mem) {
            std::memset(mem.data(), 0, mem.size_bytes());
        }
        /**
         * @brief Takes \p sampleCnt 32-bit samples from \p input, multiplies each by the \p gainQ16 (16.16), and
         * stores the upper 16 bits of each resulting sample to \p output.
         *
         * @param input 32-bit input samples
         * @param output  16-bit output samples
         * @param sampleCnt number of samples to process
         * @param gainQ16 amplification to apply in 16.16 fixed-point.
         */
        static inline void reduce32to16(const int32_t* input, int16_t* output, const uint32_t sampleCnt, const uint32_t gainQ16) {
            int16_t* const out_end = output + sampleCnt;
            if(gainQ16 == UNITY_GAIN) {
                const int16_t* in16 = (const int16_t*)input + 1;
                while(output < out_end) {
                    *output = *in16;
                    in16 += 2;
                    output += 1;
                }
                // Boring, portable ANSI C:
                // while(output < out_end) {
                //     *output = *input >> 16;
                //     input += 1;
                //     output += 1;
                // }
            } else {
                while(output < out_end) {
                    *output = simdutil::impl::Impl<SOC>::template clamp<16>(
                        ((int64_t)(*input) * gainQ16) >> (16+16)
                    );
                    input += 1;
                    output += 1;
                }
            }
        }

        /**
         * @brief Takes \p sampleCnt 32-bit samples from \p input, multiplies each by the \p gainQ16 (16.16), and
         * stores the upper \p significantBits bits of each resulting sample as a sample in \p output.
         *
         * @param input 32-bit input samples
         * @param output  16-bit output samples
         * @param sampleCnt number of samples to process
         * @param gainQ16 amplification to apply in 16.16 fixed-point.
         * @param significantBits number of significant bits to keep (1...16)
         */
        static inline void reduce32to16(const int32_t* input, int16_t* output, const uint32_t sampleCnt, const uint32_t gainQ16, const uint32_t significantBits) {
            if(significantBits >= 16) {
                reduce32to16(input,output,sampleCnt,gainQ16);
            } else {
                const uint32_t mask = 0xffffffffu << (16-significantBits);
                int16_t* const out_end = output + sampleCnt;
                if(gainQ16 == UNITY_GAIN) {
                    const int16_t* in16 = (const int16_t*)input + 1;
                    while(output < out_end) {
                        *output = *in16 & mask;
                        in16 += 2;
                        output += 1;
                    }
                    // Boring, portable ANSI C:
                    // while(output < out_end) {
                    //     *output = (*input >> 16) & mask;
                    //     input += 1;
                    //     output += 1;
                    // }
                } else {
                    while(output < out_end) {
                        *output = simdutil::impl::Impl<SOC>::template clamp<16>(
                            (((int64_t)(*input) * gainQ16) >> (16+16))
                         ) & mask;
                        input += 1;
                        output += 1;
                    }
                }
            }
        }

        /**
         * @brief Produces \p sampleCnt output samples by reading 2* \p sampleCnt samples from \p input
         * and averaging each pair of them into one output sample.
         *
         * @param input
         * @param output
         * @param sampleCnt
         */
        static inline void stereoToMono(const int16_t* input, int16_t* output, const uint32_t sampleCnt) {
            int16_t* const out_end = output + sampleCnt;
            while(output < out_end) {
                output[0] = (input[0] + input[1]) / 2;
                input += 2;
                output += 1;
            }
        }

        /**
         * @brief Reads \p sampleCnt samples from \p input, duplicates each of them, and writes the 2* \p sampleCnt
         * samples to \p output
         *
         * @param input
         * @param output
         * @param sampleCnt
         */
        template<typename OUT_t>
        requires (sizeof(OUT_t) == 2*sizeof(int16_t) && alignof(OUT_t) >= 2*alignof(int16_t))
        // [[gnu::noinline]]
        static inline void monoToStereo(const int16_t* input, OUT_t* output, const uint32_t sampleCnt) {
            const int16_t* const in_end = input + sampleCnt;
            int16_t* out16 = (int16_t*)output;
            while(input < in_end) {
                out16[0] = input[0];
                out16[1] = input[0];
                input += 1;
                out16 += 2;
            }
        }

        /**
         * @brief Applies a gain/amplification factor to the \p input and stores the result
         * into \p output. The output samples are clamped/saturated to \c int16_t.
         * 
         * @param input input sample data
         * @param output output sample data
         * @param sampleCnt number of samples to process
         * @param gainFP fixed-point gain factor
         * @param fracBits number of fractional bits of \p gainFP
         */
        static inline void applyGain(
            const int16_t* input,
            int16_t* output,
            const uint32_t sampleCnt,
            uint32_t gainFP,
            uint32_t fracBits) {

            if(gainFP <= ((uint32_t)1<<fracBits)) {
                // gain <= 1.0; no need to saturate:
                const int16_t* const in_end = input + sampleCnt;                
                while(input < in_end) {
                    output[0] = (((int32_t)input[0]) * gainFP) >> fracBits;
                    output += 1;
                    input += 1;
                }
            } else {
                // gain > 1.0; saturate:
                const int16_t* const in_end = input + sampleCnt;                
                while(input < in_end) {
                    output[0] = clamps<16>((((int32_t)input[0]) * gainFP) >> fracBits);
                    output += 1;
                    input += 1;
                }
            }
        }

        /**
         * @brief Limits/saturates/"clamps" the signed \p value to the range of a 
         * signed value of \p BITS bits.
         * 
         * @tparam BITS number of bits (incl. sign bit) of the result value; default: 16
         * 
         * @param value value to clamp
         * @return \p value clamped to the range [ -(1<<(BITS-1)), (1<<(BITS-1))-1 ] 
         */
        template<unsigned BITS = 16>
        requires (BITS >= 1 && BITS <= 32)
        static inline constexpr int32_t clamps(const int32_t value) {
            if constexpr (BITS >= 32) {
                return value;
            } else {
                constexpr int32_t MAX = std::numeric_limits<int32_t>::max() >> (32-BITS);
                constexpr int32_t MIN = (int32_t)(-1)-MAX;
                return std::min(std::max(value,MIN),MAX);
            }
        }

        template<unsigned FRAC_BITS>
        requires (16 <= FRAC_BITS && FRAC_BITS <= 31)
        [[gnu::noinline]]
        static inline void linearRamp(const int16_t* input, int16_t* output, const uint32_t sampleCnt, int32_t volStartFP, int32_t volStepFP) {
            if(volStepFP != 0) {
                uint32_t currVol = volStartFP;
                for(uint32_t i = 0; i < sampleCnt; ++i) {
                    output[i] = clamps<16>(((int32_t)input[i] * currVol) >> FRAC_BITS);
                    currVol += volStepFP;
                }
            } else {
                applyGain(input,output,sampleCnt,volStartFP,FRAC_BITS);
            }
        }
    };

    /**
     * @brief Specialized implementation of the utility functions using the ESP32-S3's SIMD instructions.
     *
     */
    template<>
    struct Utils<arch::SoC::ESP32> : public Utils<arch::SoC::OTHER> {

        /**
         * @brief Limits/saturates/"clamps" the signed \p value to the range of a 
         * signed value of \p BITS bits.
         * 
         * @tparam BITS number of bits (incl. sign bit) of the result value; default: 16
         * 
         * @param value value to clamp
         * @return \p value clamped to the range [-(1<<(BITS-1)),(1<<(BITS-1)-1)[ 
         */
        template<unsigned BITS = 16>
        requires (BITS >= 1 && BITS <= 31)
        static inline constexpr int32_t clamps(const int32_t value) {
            if (std::is_constant_evaluated() ||
                !arch::XT_CLAMPS ||
                BITS < 8 ||
                BITS > 23 ||
                simdutil::ctime::known(value)) {
                return Utils<arch::SoC::OTHER>::clamps(value);
            } else {
                int32_t r;
                asm (
                    "CLAMPS %[r], %[value], %[bits]"
                    : [r] "=r" (r)
                    : [value] "r" (value),
                      [bits] "n" (BITS-1)
                );
                return r;
            }
        }

    };

    /**
     * @brief Specialized implementation of the utility functions using the ESP32-S3's SIMD instructions.
     *
     */
    template<>
    struct Utils<arch::SoC::ESP32_S3> : public Utils<arch::SoC::ESP32> {

        static constexpr const char* TAG = "audio::Utils";

        using simd = simdutil::Util<arch::SoC::ESP32_S3>;

        static constexpr std::size_t VEC_LEN_BYTES = simd::VEC_LEN_BYTES;


        /* Forms a memory reference to \c bcnt bytes pointed to by \c ptr */
        #define MEM_BYTES(ptr,bcnt) (*((std::remove_reference_t<decltype(*ptr)>(*)[(bcnt) / sizeof(std::remove_reference_t<decltype(*ptr)>)])ptr))

        /* Forms a memory reference to \c cnt elements of the array pointed to by \c ptr */
        #define MEM_ELEMS(ptr,cnt) (*(std::remove_reference_t<decltype(*ptr)>(*)[cnt])ptr)


        /**
         * @brief Clears a memory range to all-0.
         *
         * @tparam D
         * @tparam X
         *
         * @param mem std::span of the memory to clear
         */
        template<typename D, std::size_t X>
        requires (!std::is_const_v<D>)
        static inline void clr(const std::span<D,X>& mem) {

            uint32_t byteCnt = mem.size_bytes();
            D* dst = mem.data();
            if(byteCnt >= VEC_LEN_BYTES) {
                {
                    uint32_t off = ((uintptr_t)dst + 15) & ~0xf;
                    if(off != (uintptr_t)dst) {
                        off = off - (uintptr_t)dst;
                        off = std::min(off,byteCnt);
                        byteCnt -= off;
                        clrmem_shrt(dst,off);
                    }
                }
                asm (
                    "EE.ZERO.Q q0" "\n"
                    "LOOPNEZ %[cnt], .Lend_%=" "\n"
                        "EE.VST.128.IP q0, %[dst], 16" "\n"
                    ".Lend_%=:"
                    : [dst] "+r" (dst),
                      "=m" (MEM_BYTES(dst,byteCnt))
                    : [cnt] "r" (byteCnt / VEC_LEN_BYTES)
                );
            }
            clrmem_shrt(dst, byteCnt);

        }

        /**
         * @brief
         *
         * @param input
         * @param output must be 2-byte aligned!
         * @param sampleCnt
         * @param gainQ16
         */
        // [[gnu::noinline]]
        static inline void reduce32to16(const int32_t* input, int16_t* output, uint32_t sampleCnt, uint32_t gainQ16 = UNITY_GAIN) {

            using IN_t = std::remove_reference_t<decltype(*input)>;
            using OUT_t = std::remove_reference_t<decltype(*output)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 2.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 4.


            // q7[1] := (gain << shift)
            const uint32_t shift = loadGainIntoQ<7>(gainQ16)-16;

            static constexpr unsigned Q_OUT = 0;
            static constexpr unsigned Q_TMP = 1;

            static constexpr std::size_t VEC_LEN_BYTES = simd::VEC_LEN_BYTES;


            const auto f1 = [&input, &shift](const uint32_t outByteCnt) {
                // Number of output elements we produce (== number of input elements we'll consume):
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;
                asm volatile (

                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], -(2*16)" "\n"  // restore input pointer

                    "EE.ZERO.QACC" "\n"

                    "EE.SRC.Q.QUP q1, q0, q1" "\n" // q0 := q1
                    "EE.SRC.Q.QUP q2, q0, q2" "\n" // q0 := q2

                    "EE.VUNZIP.16 q1, q2" "\n" // lower 16 bits -> q1, upper 16 bits -> q2
                    "EE.VSMULAS.S16.QACC q2, q7, 1" "\n" // QACC[n] := q2[n] * q7[1]
                    "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n" // qout[n] := QACC[n] >> shift

                    :
                    : [input] "r" (input),
                      [shift] "r" (shift),
                      "m" (MEM_BYTES(input,inByteCnt)),
                      [out] "n" (Q_OUT)
                );
                // input += elemCnt;
                simdutil::ptrs::incp(input,inByteCnt);
            };

            const auto floop = [&input, &shift](OUT_t*& output, const uint32_t outByteCnt) {
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;
                asm volatile (
                    // We consume 32 bytes of input per iteration,
                    // and we need to keep an additional 16 bytes around for alignment:
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], 16" "\n"

                    "LOOPNEZ %[cnt], .Lend_%=" "\n"
                        "EE.ZERO.QACC" "\n"

                        // Align input:
                        "EE.SRC.Q.QUP q1, q0, q1" "\n" // q0 := q1
                        "EE.SRC.Q.QUP q2, q0, q2" "\n" // q0 := q2

                        // 32 bytes of aligned input now in (q2:q1), next unaligned input in q0
                        "EE.VUNZIP.16 q1, q2" "\n" // lower 16 bits -> q1, upper 16 bits -> q2
                        "EE.VSMULAS.S16.QACC.LD.INCP q1, %[input], q2, q7, 1" "\n" // QACC[n] := q2[n] * q7[1]

                        "EE.VLD.128.IP q2, %[input], 16" "\n" // load next input

                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n" // q3[n] = QACC[n] >> shift
                        "EE.VST.128.IP q3, %[output], 16" "\n" // write q3 to output
                    ".Lend_%=:"

                    : [input] "+r" (input),
                      [output] "+r" (output),
                      "=m" (MEM_BYTES(output, outByteCnt))
                    : [shift] "r" (shift),
                      [cnt] "r" (outByteCnt/VEC_LEN_BYTES),
                      "m" (MEM_BYTES(input,inByteCnt))
                );
                // Re-adjust input for the number of bytes we 'overread':
                simdutil::ptrs::incp(input,-3*VEC_LEN_BYTES);
            };


            const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

            simd::process<Q_OUT, Q_TMP>(f1, floop, output, outByteCnt);
        }


        /**
         * @brief
         *
         * @param input
         * @param output must be 2-byte aligned!
         * @param sampleCnt
         * @param gainQ16
         * @param significantBits
         */
        // [[gnu::noinline]]
        static inline void reduce32to16(const int32_t* input, int16_t* output, const uint32_t sampleCnt, uint32_t gainQ16, const uint32_t significantBits) {

            using IN_t = std::remove_reference_t<decltype(*input)>;
            using OUT_t = std::remove_reference_t<decltype(*output)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 2.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 4.


            static constexpr unsigned Q_OUT = 0;
            static constexpr unsigned Q_TMP = 1;

            static constexpr unsigned Q_MASK = 6;
            static constexpr unsigned Q_GAIN = 7;
            /*
                Step 1: Set up q7[1] with 15 significant bits from gainQ16
            */

            const uint32_t shift = loadGainIntoQ<Q_GAIN>(gainQ16)-16; // implicit right-shift by 16

            /*
                Step 2: Set up q6 with the bit mask
            */

            loadMaskIntoQ<Q_MASK>(significantBits);



            /*
                "Step" 3: Define the processing functions
            */



            const auto f1 = [&input, &shift](const uint32_t outByteCnt) {
                // Number of output elements we produce (== number of input elements we'll consume):
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;
                asm volatile (

                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], -(2*16)" "\n"  // restore input pointer

                    "EE.ZERO.QACC" "\n"

                    "EE.SRC.Q.QUP q1, q0, q1" "\n" // q0 := q1
                    "EE.SRC.Q.QUP q2, q0, q2" "\n" // q0 := q2

                    "EE.VUNZIP.16 q1, q2" "\n" // lower 16 bits -> q1, upper 16 bits -> q2
                    "EE.VSMULAS.S16.QACC q2, q%[gain], 1" "\n" // QACC[n] := q2[n] * q7[1]
                    "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n" // qout[n] := QACC[n] >> shift

                    "EE.ANDQ q%[out], q%[out], q%[mask]" "\n" // qout[n] := qout[n] & mask

                    :
                    : [input] "r" (input),
                      [shift] "r" (shift),
                      "m" (MEM_BYTES(input,inByteCnt)), // we consume one int32_t for every int16_t we produce.
                      [out] "n" (Q_OUT),
                      [gain] "n" (Q_GAIN),
                      [mask] "n" (Q_MASK)
                );
                simdutil::ptrs::incp(input,inByteCnt);
            };

            const auto floop = [&input, &shift](int16_t*& output, const uint32_t outByteCnt) {
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;
                asm volatile (
                    // We consume 32 bytes of input per iteration,
                    // and we need to keep an additional 16 bytes around for alignment:
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], 16" "\n"

                    "LOOPNEZ %[cnt], .Lend_%=" "\n"
                        "EE.ZERO.QACC" "\n"

                        // Align input:
                        "EE.SRC.Q.QUP q1, q0, q1" "\n" // q0 := q1
                        "EE.SRC.Q.QUP q2, q0, q2" "\n" // q0 := q2

                        // 32 bytes of aligned input now in (q2:q1), next unaligned input in q0
                        "EE.VUNZIP.16 q1, q2" "\n" // lower 16 bits -> q1, upper 16 bits -> q2
                        "EE.VSMULAS.S16.QACC.LD.INCP q1, %[input], q2, q%[gain], 1" "\n" // QACC[n] := q2[n] * q7[1]

                        "EE.VLD.128.IP q2, %[input], 16" "\n" // load next input

                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n" // q3[n] = QACC[n] >> shift

                        "EE.ANDQ q3, q3, q%[mask]" "\n" // q3[n] := q3[n] & mask

                        "EE.VST.128.IP q3, %[output], 16" "\n" // write q3 to output
                    ".Lend_%=:"

                    : [input] "+r" (input),
                      [output] "+r" (output),
                      "=m" (MEM_BYTES(output,outByteCnt))
                    : [shift] "r" (shift),
                      [cnt] "r" (outByteCnt/VEC_LEN_BYTES),
                      "m" (MEM_BYTES(input,inByteCnt)),
                      [gain] "n" (Q_GAIN),
                      [mask] "n" (Q_MASK)
                );
                // Re-adjust input for the number of bytes we 'overread':
                simdutil::ptrs::incp(input,-3*VEC_LEN_BYTES);
            };


            /*
                Step 4: Execute the processing
            */
            const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

            simd::process<Q_OUT, Q_TMP>(f1, floop, output, outByteCnt);

        }

        /**
         * @brief Produces \p sampleCnt output samples by reading 2* \p sampleCnt samples from \p input
         * and averaging each pair of them into one output sample.
         *
         * @param input
         * @param output
         * @param sampleCnt
         */
        [[gnu::noinline]]
        static inline void stereoToMono(const int16_t* input, int16_t* output, const uint32_t sampleCnt) {
            static constexpr unsigned Q_OUT = 0;
            static constexpr unsigned Q_TMP = 1;

            // q7[0] := 1
            asm volatile (
                "EE.MOVI.32.Q q7, %[one], 0" "\n"
                :
                : [one] "r" (1)
            );


            const auto f1 = [&input](const uint32_t outByteCnt) {
                asm volatile (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], -(2*16)" "\n" // restore input

                        "EE.SRC.Q.QUP q1, q0, q1" "\n"
                        "EE.SRC.Q.QUP q2, q0, q2" "\n"

                        "EE.VUNZIP.16 q1, q2" "\n"

                        "EE.MOV.S16.QACC q1" "\n" // QACC[n] := q1[n]
                        "EE.VSMULAS.S16.QACC q2, q7, 0" "\n" // QACC[n] += q2[n] * q7[0]

                        "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n"
                    :
                    : [input] "r" (input),
                      [shift] "r" (1),
                      "m" (MEM_BYTES(input, outByteCnt*2)),
                      [out] "n" (Q_OUT)
                );

                simdutil::ptrs::incp(input, outByteCnt*2);

            };

            const auto floop = [&input](int16_t*& output, const uint32_t outByteCnt) {
                const uint32_t inByteCnt = 2*outByteCnt;
                asm volatile (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], 16" "\n"

                    "LOOPNEZ %[cnt], .Lend_%=" "\n"

                        "EE.SRC.Q.QUP q1, q0, q1" "\n"
                        "EE.SRC.Q.QUP q2, q0, q2" "\n"

                        "EE.VUNZIP.16 q1, q2" "\n"

                        "EE.MOV.S16.QACC q1" "\n" // QACC[n] := q1[n]
                        "EE.VSMULAS.S16.QACC.LD.INCP q1, %[input], q2, q7, 0" "\n" // QACC[n] += q2[n] * q7[0]

                        "EE.VLD.128.IP q2, %[input], 16" "\n"

                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n" // q3[n] := QACC[n] >> 1

                        "EE.VST.128.IP q3, %[output], 16" "\n"
                    ".Lend_%=:" "\n"
                    : [input] "+r" (input),
                      [output] "+r" (output),
                      "=m" (MEM_BYTES(output, outByteCnt))
                    : [cnt] "r" (outByteCnt / VEC_LEN_BYTES),
                      [shift] "r" (1),
                      "m" (MEM_BYTES(input, inByteCnt))
                );
                simdutil::ptrs::incp(input, -(3*VEC_LEN_BYTES));
            };

            const uint32_t outByteCnt = sampleCnt * sizeof(int16_t);

            simdutil::process<Q_OUT, Q_TMP>(f1, floop, output, outByteCnt);
        }

        /**
         * @brief Reads \p sampleCnt samples from \p input, duplicates each of them, and writes the 2* \p sampleCnt
         * samples to \p output
         *
         * @param input
         * @param output must be \e 4-byte aligned!
         * @param sampleCnt
         */
        template<typename OUT_t>
        requires (sizeof(OUT_t) == 2*sizeof(int16_t) && alignof(OUT_t) >= 2*alignof(int16_t))
        // [[gnu::noinline]]
        static inline void monoToStereo(const int16_t* input, OUT_t* output, uint32_t sampleCnt) {

            using IN_t = std::remove_reference_t<decltype(*input)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 4.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 2.

            static constexpr unsigned Q_OUT = 2;
            static constexpr unsigned Q_TMP = 3;

            const auto f1 = [&input](const uint32_t outByteCnt) {
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;
                asm volatile (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], -16" "\n"

                    "EE.SRC.Q.QUP q%[out], q0, q1" "\n"
                    "MV.QR q%[tmp], q%[out]" "\n"
                    "EE.VZIP.16 q%[out], q%[tmp]" "\n"
                    :
                    : [input] "r" (input),
                      "m" (MEM_BYTES(input,inByteCnt)),
                      [out] "n" (Q_OUT),
                      [tmp] "n" (Q_TMP)
                );
                simdutil::ptrs::incp(input, inByteCnt);
            };

            const auto floop = [&input](OUT_t*& output, const uint32_t outByteCnt) {
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;

                if(((uintptr_t)input % VEC_LEN_BYTES) == 0) {
                    // Input is already aligned; full steam ahead:
                    // Do 32-byte chunks of input in the loop to avoid pipeline stalls:
                    asm volatile (
                        "LOOPNEZ %[cnt], .Lend_%=" "\n"
                            "EE.VLDHBC.16.INCP q0, q1, %[input]" "\n"
                            "EE.VLDHBC.16.INCP q2, q3, %[input]" "\n"

                            "EE.VST.128.IP q0, %[output], 16" "\n"
                            "EE.VST.128.IP q1, %[output], 16" "\n"

                            "EE.VST.128.IP q2, %[output], 16" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"
                        ".Lend_%=:" "\n"
                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt))
                        : [cnt] "r" (outByteCnt / (4*VEC_LEN_BYTES)),
                          "m" (MEM_BYTES(input,inByteCnt))
                    );

                    if(outByteCnt & (2*VEC_LEN_BYTES)) {
                        // Have at least two more full vectors of output to produce:
                        asm volatile (
                            "EE.VLDHBC.16.INCP q0, q1, %[input]" "\n"
                            // Pipeline stall for 1 clock cycle here.
                            "EE.VST.128.IP q0, %[output], 16" "\n"
                            "EE.VST.128.IP q1, %[output], 16" "\n"
                            : [input] "+r" (input),
                              [output] "+r" (output),
                              "=m" (MEM_BYTES(output,outByteCnt))
                            : "m" (MEM_BYTES(input,inByteCnt))
                        );
                    }

                    if(outByteCnt & (1*VEC_LEN_BYTES)) {
                        // One more full vector of output to go...
                        asm volatile (
                            "EE.VLDHBC.16.INCP q0, q1, %[input]" "\n" // Increments input by one vector!
                            "EE.VST.128.IP q0, %[output], 16" "\n"
                            : [input] "+r" (input),
                              [output] "+r" (output),
                              "=m" (MEM_BYTES(output,VEC_LEN_BYTES))
                            : "m" (MEM_BYTES(input, (VEC_LEN_BYTES/2)))
                        );
                        // We consumed only an additional half vector of input to produce a full vector of output.
                        simdutil::ptrs::incp(input,-(VEC_LEN_BYTES/2));
                    }


                } else {

                    // Input is not aligned; must do alignment:
                    asm volatile (
                        "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                        "EE.VLD.128.IP q1, %[input], 16" "\n"

                        "LOOPNEZ %[cnt], .Lend_%=" "\n"
                            "EE.SRC.Q.QUP q2, q0, q1" "\n"

                            "EE.VLD.128.IP q1, %[input], 16" "\n"

                            "MV.QR q3, q2" "\n"
                            "EE.VZIP.16 q2, q3" "\n"

                            "EE.VST.128.IP q2, %[output], 16" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"

                        ".Lend_%=:" "\n"
                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output, outByteCnt))
                        : [cnt] "r" (outByteCnt / (2*VEC_LEN_BYTES)),
                          "m" (MEM_BYTES(input, inByteCnt))
                    );


                    // Re-adjust input for the number of bytes we 'overread':
                    simdutil::ptrs::incp(input,-2*VEC_LEN_BYTES);

                    if(outByteCnt & VEC_LEN_BYTES) {
                        // One more full vector of output to go...
                        asm volatile (
                            "EE.SRC.Q.QUP q2, q0, q1" "\n"

                            "MV.QR q3, q2" "\n"
                            "EE.VZIP.16 q2, q3" "\n"

                            "EE.VST.128.IP q2, %[output], 16" "\n"
                            : [output] "+r" (output),
                              "=m" (MEM_BYTES(output,VEC_LEN_BYTES))
                            :
                        );
                        // Now we consumed an additional half vector of input to produce a full vector of output.
                        simdutil::ptrs::incp(input,(VEC_LEN_BYTES * IN_ELEM_SZ)/OUT_ELEM_SZ);
                    }
                }
            };

            const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

            simd::process<Q_OUT, Q_TMP>(f1, floop, output, outByteCnt);

        }

        /**
         * @brief Applies a gain and \e truncates the output to \c int16_t; this is safe to to
         * when the \c gain is <= 1.0 and is about 2x as fast as the \e saturating variant below.
         * 
         * @param input 
         * @param output 
         * @param sampleCnt 
         * @param gainFP 
         * @param fracBits 
         * 
         * @see applyGain_sat()
         * @see applyGain()
         */
        static inline void applyGain_trunc(
            const int16_t* input,
            int16_t* output,
            const uint32_t sampleCnt,
            uint32_t gainFP,
            uint32_t fracBits) {

            using IN_t = std::remove_reference_t<decltype(*input)>;
            using OUT_t = std::remove_reference_t<decltype(*output)>;

            static_assert(sizeof(IN_t) == sizeof(OUT_t));

            constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t);

            constexpr unsigned Q_OUT = 0;
            constexpr unsigned Q_TMP = 1;
            constexpr unsigned Q_GAIN = 7;

            const uint32_t shift = bcastFPIntoQ<Q_GAIN>(gainFP,fracBits);

            const auto f1 = [&input,&shift](const uint32_t outByteCnt) {

                asm volatile (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], -16" "\n"

                    "WSR.SAR %[shift]" "\n"

                        "EE.SRC.Q.QUP q1, q0, q1" "\n"

                        "EE.VMUL.S16 q%[out], q1, q%[gain]" "\n"

                    : [input] "+r" (input)
                    : "m" (MEM_BYTES(input,outByteCnt)),
                      [shift] "r" (shift),
                      [out] "n" (Q_OUT),
                      [gain] "n" (Q_GAIN)
                );

                simdutil::ptrs::incp(input, outByteCnt);

            };

            const auto floop = [&input,&shift](int16_t*& output, uint32_t outByteCnt) {

                asm (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"

                    "WSR.SAR %[shift]" "\n"

                    "EE.SRC.Q.QUP q1, q0, q1" "\n"

                    "EE.VMUL.S16.LD.INCP q1, %[input], q3, q1, q%[gain]" "\n"

                    // 1 result in q3, next unaligned input in (q1:q0)
                    : [input] "+r" (input)
                    : [gain] "n" (Q_GAIN),
                      [shift] "r" (shift)
                );

                if(simdutil::ctime::maybe(hwdiv<3*VEC_LEN_BYTES>(outByteCnt) != 0)) {
                    asm (
                        "WSR.SAR %[shift]" "\n"

                        "LOOPNEZ %[cnt], .Lend_%=" "\n"

                            "EE.SRC.Q.LD.IP q2, %[input], 16, q0, q1" "\n"
                            "EE.VMUL.S16.ST.INCP q3, %[output], q3, q0, q%[gain]" "\n"

                            "EE.SRC.Q.LD.IP q0, %[input], 16, q1, q2" "\n"
                            "EE.VMUL.S16.ST.INCP q3, %[output], q3, q1, q%[gain]" "\n"

                            "EE.SRC.Q.LD.IP q1, %[input], 16, q2, q0" "\n"
                            "EE.VMUL.S16.ST.INCP q3, %[output], q3, q2, q%[gain]" "\n"

                        ".Lend_%=:" "\n"
                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt))
                        : [cnt] "r" (hwdiv<3*VEC_LEN_BYTES>(outByteCnt)),
                          "m" (MEM_BYTES(input,outByteCnt)),
                          [shift] "r" (shift),
                          [gain] "n" (Q_GAIN)
                    );

                    outByteCnt = hwmod<3*VEC_LEN_BYTES>(outByteCnt);
                }

                if(simdutil::ctime::maybe((outByteCnt/VEC_LEN_BYTES) != 0)) {
                    asm (
                        "WSR.SAR %[shift]" "\n"

                        "LOOPNEZ %[cnt], .Lend_%=" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"
                            "EE.SRC.Q.QUP q1, q0, q1" "\n"
                            "EE.VMUL.S16.LD.INCP q1, %[input], q3, q1, q%[gain]" "\n"
                        ".Lend_%=:" "\n"
                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt))
                        : [cnt] "r" (outByteCnt/VEC_LEN_BYTES),
                          "m" (MEM_BYTES(input,outByteCnt)),
                          [shift] "r" (shift),
                          [gain] "n" (Q_GAIN)
                    );
                }

                simdutil::ptrs::incp(input, -(3*VEC_LEN_BYTES));

            };

            const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

            simd::process<Q_OUT,Q_TMP>(f1, floop, output, outByteCnt);

        }


        /**
         * @brief Applies a gain and \e saturates the output to \c int16_t; this is always safe to to
         * but slower than the \e truncating variant above and unnecessary when \c gain <= 1.0 
         * 
         * @param input 
         * @param output 
         * @param sampleCnt 
         * @param gainFP 
         * @param fracBits 
         * 
         * @see applyGain_trunc()
         * @see applyGain()
         */
        static inline void applyGain_sat(
            const int16_t* input,
            int16_t* output,
            const uint32_t sampleCnt,
            uint32_t gainFP,
            uint32_t fracBits) {

            using IN_t = std::remove_reference_t<decltype(*input)>;
            using OUT_t = std::remove_reference_t<decltype(*output)>;

            static_assert(sizeof(IN_t) == sizeof(OUT_t));

            constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t);

            constexpr unsigned Q_OUT = 0;
            constexpr unsigned Q_TMP = 1;
            constexpr unsigned Q_GAIN = 7;

            const uint32_t shift = bcastFPIntoQ<Q_GAIN>(gainFP,fracBits);

            const auto f1 = [](const uint32_t outByteCnt, IN_t*& input, const uint32_t shift) {

                asm volatile (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], -16" "\n"

                    "EE.ZERO.QACC" "\n"
                    "EE.SRC.Q q0, q0, q1" "\n"
                    "EE.VMULAS.S16.QACC q0, q%[gain]" "\n"

                    // "ADD %[input], %[input], %[byteCnt]" "\n"

                    "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n"
                    : 
                    : [input] "r" (input),
                      "m" (MEM_BYTES(input,outByteCnt)),
                      [shift] "r" (shift),
                      [out] "n" (Q_OUT),
                      [gain] "n" (Q_GAIN)
                    //   [byteCnt] "r" (outByteCnt)
                );

                simdutil::ptrs::incp(input, outByteCnt);

            };

            const auto floop = [](OUT_t*& output, uint32_t outByteCnt, IN_t*& input, const uint32_t shift) {

                asm (
                    "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                    "EE.VLD.128.IP q1, %[input], 16" "\n"
                    "EE.VLD.128.IP q2, %[input], 16" "\n"

                    "EE.SRC.Q q0, q0, q1" "\n"
                    "EE.ZERO.QACC" "\n"
                    "EE.VMULAS.S16.QACC.LD.IP.QUP q0, %[input], 16, q0, q%[gain], q1, q2" "\n"
                    : [input] "+r" (input)
                    : "m" (MEM_BYTES(input,outByteCnt)),
                      [gain] "n" (Q_GAIN)
                );
                // input is ahead of output by *4* vectors now.

                // 1 vector of results in QACC, next aligned data in q1, unaligned data in (q0:q2)

                if(hwdiv<3*VEC_LEN_BYTES>(outByteCnt) != 0) [[likely]] {
                    asm (
                        "LOOPNEZ %[cnt], .Lend_%=" "\n"
                            "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n"
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC.LD.IP.QUP q1, %[input], 16, q1, q%[gain], q2, q0" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"

                            "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n"
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC.LD.IP.QUP q2, %[input], 16, q2, q%[gain], q0, q1" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"

                            "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n"
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC.LD.IP.QUP q0, %[input], 16, q0, q%[gain], q1, q2" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"
                        ".Lend_%=:" "\n"
                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt))
                        : [cnt] "r" (hwdiv<3*VEC_LEN_BYTES>(outByteCnt)),
                          [shift] "r" (shift),
                          "m" (MEM_BYTES(input,outByteCnt)),
                          [gain] "n" (Q_GAIN)
                    );

                    outByteCnt = hwmod<3*VEC_LEN_BYTES>(outByteCnt);
                }

                // 1 vector of results in QACC, next aligned data in q1, unaligned data in (q0:q2)

                if(outByteCnt >= VEC_LEN_BYTES) {
                    // Store the result we already have in QACC:
                    asm (
                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n"
                        "EE.VST.128.IP q3, %[output], 16" "\n"

                        "ADDI %[input], %[input], 16" "\n" // Maintain invariant even if not actually loading more data.

                        : [input] "+r" (input),
                          [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt))
                        : "m" (MEM_BYTES(input,outByteCnt)),
                          [shift] "r" (shift)
                    );
                    if(outByteCnt >= 2*VEC_LEN_BYTES) {
                        // Next aligned data is in q1
                        // Next unaligned data is in (q0:q2)
                        asm (
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC q1, q%[gain]" "\n"

                            "ADDI %[input], %[input], 16" "\n" // Maintain invariant even if not actually loading more data.

                            "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n"
                            "EE.VST.128.IP q3, %[output], 16" "\n"
                            : [input] "+r" (input),
                              [output] "+r" (output),
                              "=m" (MEM_BYTES(output,outByteCnt))
                            :
                              "m" (MEM_BYTES(input,outByteCnt)),
                              [shift] "r" (shift),
                              [gain] "n" (Q_GAIN)
                        );
                    }
                }

                simdutil::ptrs::incp(input, -(4*VEC_LEN_BYTES));

            };

            const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

            simd::process<Q_OUT,Q_TMP>(f1, floop, output, outByteCnt, input, shift);

        }

        static inline void applyGain(
            const int16_t* input,
            int16_t* output,
            const uint32_t sampleCnt,
            uint32_t gainFP,
            uint32_t fracBits) {

            if( gainFP > ((uint32_t)1 << fracBits) ) {
                // gain > 1.0
                applyGain_sat(input,output,sampleCnt,gainFP,fracBits);
            } else {
                // gain <= 1.0; no need for saturation.
                applyGain_trunc(input,output,sampleCnt,gainFP,fracBits);
            }

        }


        // TODO Maybe turn FRAC_BITS into a regular function argument. Would need to adjust/truncate shift (reduce resolution) in case frac_bits < 16.
        template<unsigned FRAC_BITS>
        requires (16 <= FRAC_BITS && FRAC_BITS <= 31)
        // [[gnu::noinline]]
        static inline void linearRamp(const int16_t* input, int16_t* output, const uint32_t sampleCnt, int32_t volStartFP, int32_t volStepFP) {

            if(volStepFP != 0) {

                using IN_t = std::remove_reference_t<decltype(*input)>;
                using OUT_t = std::remove_reference_t<decltype(*output)>;

                static_assert(sizeof(IN_t) == sizeof(OUT_t));

                static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 2.

                constexpr unsigned Q_OUT = 0;
                constexpr unsigned Q_TMP = 2;

                constexpr unsigned Q_DUMMY = 3;

                constexpr unsigned Q_VLO = 5;
                constexpr unsigned Q_VHI = 6;
                constexpr unsigned Q_STEP = 7;

                
                uint32_t shift = __builtin_clz((uint32_t)volStartFP);
                if(volStepFP > 0) {
                    shift = __builtin_clz((uint32_t)(volStartFP + sampleCnt*volStepFP));
                }
                shift -= 1; // sign bit must be maintained
                
                const int32_t volStart = volStartFP << shift;
                const int32_t volStep = volStepFP << shift; // ok to left-shift signed value here

                shift = shift + (FRAC_BITS-16);

                // q_step[n] := volStep
                asm volatile (
                    "EE.VLDBC.32 q%[step], %[volstep]" "\n"
                    :
                    : [volstep] "r" (&volStep),
                      "m" (volStep),
                      [step] "n" (Q_STEP)
                );

                // Fill (q_vhi:q_vlo) with [volStart+0*volStep,...,volStart+7*volStep]
                asm volatile (
                    "EE.VLDBC.32 q%[vlo], %[volstart]" "\n" // q_vlo[n] := volStart

                    "MV.QR q%[tmp], q%[step]" "\n" // q_tmp[n] := volStep

                    // q_vlo[n] := q_vlo[n] + n*volStep

                    "EE.SLCI.2Q q%[dummy], q%[tmp], (4-1)" "\n"
                    "EE.VADDS.S32 q%[vlo], q%[vlo], q%[tmp]" "\n"

                    "EE.SLCI.2Q q%[dummy], q%[tmp], (4-1)" "\n"
                    "EE.VADDS.S32 q%[vlo], q%[vlo], q%[tmp]" "\n"

                    "EE.SLCI.2Q q%[dummy], q%[tmp], (4-1)" "\n"
                    "EE.VADDS.S32 q%[vlo], q%[vlo], q%[tmp]" "\n"

                    // q_vhi[n] := q_vlo[n] + 4*volStep
                    "SSAI 2" "\n" // multiply by 4 = shift by 2

                    "EE.VSL.32 q%[vhi], q%[step]" "\n" // q_vhi[n] := 4*volStep

                    "EE.VADDS.S32 q%[vhi], q%[vhi], q%[vlo]" "\n" // q_vhi[n] += q_vlo[n]

                    :
                    : [volstart] "r" (&volStart),
                      "m" (volStart),
                      [dummy] "n" (Q_DUMMY),
                      [tmp] "n" (Q_TMP),
                      [step] "n" (Q_STEP),
                      [vlo] "n" (Q_VLO),
                      [vhi] "n" (Q_VHI)
                );


                // Same as above, just a tiny bit slower:
                // {
                //     // Fill (q6:q5) with [volStart+0*volStep,...,volStart+7*volStep]
                //     int32_t v = volStart;
                //     for(unsigned i = 0; i < VOL_ELEMS; ++i) {
                //         rshiftIntoQPair<6,5>(v);
                //         v += volStep;
                //     }
                // }

                const auto f1 = [&input,&shift](const uint32_t outByteCnt) {
                    const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                    asm (

                        "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                        "EE.VLD.128.IP q1, %[input], -16" "\n"

                        // Needed if truncating:
                        // "WSR.SAR %[shift]" "\n"
                        
                            "EE.VUNZIP.16 q%[vlo], q%[vhi]" "\n" // split 8x32 into 16x16 bits; lower 16 bits -> q5, upper 16 bits -> q6

                            "EE.SRC.Q.QUP q1, q0, q1" "\n"

                            // SATURATING q_out[n] := (q1[n] * q_vhi[n]) >> shift
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC q1, q%[vhi]" "\n"
                            "EE.VZIP.16 q%[vlo], q%[vhi]" "\n" // recombine 16x16 into 8x32 bits.
                            "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n"

                            // TRUNCATING q_out[n] := (q1[n] * q_vhi[n]) >> shift
                            // "EE.VMUL.S16 q%[out], q1, q%[vhi]" "\n" // qout[n] := (q1[n] * qvhi[n]) >> shift
                            // "EE.VZIP.16 q%[vlo], q%[vhi]" "\n" // recombine 16x16 into 8x32 bits.

                        : [input] "+r" (input) // dependency tracking, used but not actually modified
                        : "m" (MEM_BYTES(input,outByteCnt)),
                          [shift] "r" (shift),
                          [out] "n" (Q_OUT),
                          [vlo] "n" (Q_VLO),
                          [vhi] "n" (Q_VHI)
                    );

                    // Increment currVol (q6:q5)[n] by elemCnt*volStep
                    {
                        int32_t step;
                        // Get volStep from q7[0]
                        asm (
                            "EE.MOVI.32.A q7, %[step], 0" "\n"
                            : [step] "=r" (step)
                        );
                        step = step * elemCnt; // 1 <= elemCnt <= 7
                        // Add to currVol (q_vhi:q_vlo)
                        asm (
                            "EE.VLDBC.32 q%[tmp], %[step]" "\n"
                            "EE.VADDS.S32 q%[vlo], q%[vlo], q%[tmp]" "\n" // q_vlo[n] := q_vlo[n] + elemCnt*step;
                            "EE.VADDS.S32 q%[vhi], q%[vhi], q%[tmp]" "\n" // q_vhi[n] := q_vhi[n] + elemCnt*step;
                            : "+r" (input) // dependency tracking only
                            : [step] "r" (&step),
                              "m" (step),
                              [tmp] "n" (Q_TMP),
                              [vlo] "n" (Q_VLO),
                              [vhi] "n" (Q_VHI)
                        );
                    }

                    simdutil::ptrs::incp(input, outByteCnt);
                };

                const auto floop = [&input,&shift](int16_t*& output, const uint32_t outByteCnt) {
                    asm volatile (

                        "EE.LD.128.USAR.IP q0, %[input], 16" "\n"
                        "EE.VLD.128.IP q1, %[input], 16" "\n"

                        // q_step[n] := q_step[n] * 8
                        "SSAI 3" "\n" // multiply by 8 = shift by 3 
                        "EE.VSL.32 q%[step], q%[step]" "\n" // we're processing 8 values per iteration, so the increment per iteration must be 8*volStep

                        // Needed if truncating:
                        // "WSR.SAR %[shift]" "\n"

                        "LOOPNEZ %[cnt], .Lend_%=" "\n"

                            "EE.VUNZIP.16 q%[vlo], q%[vhi]" "\n" // split 8x32 into 16x16 bits; lower 16 bits -> q_vlo, upper 16 bits -> q_vhi

                            "EE.SRC.Q.QUP q1, q0, q1" "\n"

                            // SATURATING q_tmp[n] := (q1[n] * q_vhi[n]) >> shift
                            "EE.ZERO.QACC" "\n"
                            "EE.VMULAS.S16.QACC.LD.IP q1, %[input], 16, q1, q%[vhi]" "\n"
                            "EE.VZIP.16 q%[vlo], q%[vhi]" "\n" // recombine 16x16 into 8x32 bits.
                            "EE.SRCMB.S16.QACC q%[tmp], %[shift], 0" "\n"

                            // TRUNCATING q_tmp[n] := (q1[n] * q_vhi[n]) >> shift
                            // "EE.VMUL.S16.LD.INCP q1, %[input], q%[tmp], q1, q%[vhi]" "\n" // q_tmp[n] := (q1[n] * q_vhi[n]) >> shift
                            // "EE.VZIP.16 q%[vlo], q%[vhi]" "\n" // recombine 16x16 into 8x32 bits.

                            "EE.VADDS.S32 q%[vlo], q%[vlo], q%[step]" "\n" // Add 32-bit volStep (4x)
                            "EE.VADDS.S32.ST.INCP q%[tmp], %[output], q%[vhi], q%[vhi], q%[step]" "\n" // Add 32-bit volStep (4x)

                        ".Lend_%=:"
                        : [output] "+r" (output),
                          "=m" (MEM_BYTES(output,outByteCnt)),
                          [input] "+r" (input)
                        : [cnt] "r" (outByteCnt / VEC_LEN_BYTES),
                          "m" (MEM_BYTES(input,outByteCnt)),
                          [shift] "r" (shift),

                          [step] "n" (Q_STEP),
                          [vlo] "n" (Q_VLO),
                          [vhi] "n" (Q_VHI),
                          [tmp] "n" (Q_TMP)
                    );

                    simdutil::ptrs::incp(input, -(2*VEC_LEN_BYTES));
                };

                const uint32_t outByteCnt = sampleCnt * OUT_ELEM_SZ;

                simd::process<Q_OUT, Q_TMP>(f1, floop, output, outByteCnt);
            } else [[unlikely]] {
                // volStep is 0
                applyGain(input,output,sampleCnt,volStartFP,FRAC_BITS);
            }
        }

        /**
         * @brief Shifts Q-register \p Q_REG right (down) by \p BYTECNT bytes while
         * inserting the lowest \p BYTECNT bytes of \p data into the upper-most
         * bytes of \p Q_REG.
         *
         * @tparam BYTECNT number of bytes to shift by/insert (0...4)
         * @tparam Q_REG Q-register to shift
         * @tparam Q_TMP temporary Q-register (clobbered)
         *
         * @param data data to shift into the upper bytes of \p Q_REG
         */
        template<std::size_t BYTECNT, unsigned Q_REG, unsigned Q_TMP>
        requires (BYTECNT <= 4 && Q_REG < 8 && Q_TMP < 8 && Q_REG != Q_TMP)
        static inline void rshiftBytesIntoQ(const uint32_t data) {
            if constexpr (BYTECNT != 0) {
                asm volatile (
                    "EE.MOVI.32.Q q%[tmp], %[data], 0" "\n"
                    "EE.SRCI.2Q q%[tmp], q%[reg], (%[bcnt]-1)" "\n"
                    :
                    : [data] "r" (data),
                      [reg] "n" (Q_REG),
                      [tmp] "n" (Q_TMP),
                      [bcnt] "n" (BYTECNT)
                );
            }
        }

        template<unsigned Q_HI, unsigned Q_LO>
        requires (Q_HI < 8 && Q_LO < 8 && Q_LO != Q_HI)
        static inline void rshiftIntoQPair(const uint32_t data) {
            asm volatile (
                "EE.SRCI.2Q q%[hi], q%[lo], (%[bcnt]-1)" "\n"
                "EE.MOVI.32.Q q%[hi], %[data], 3" "\n"
                :
                : [data] "r" (data),
                    [hi] "n" (Q_HI),
                    [lo] "n" (Q_LO),
                    [bcnt] "n" (4)
            );
        }


        /**
         * @brief Shifts the Q-register pair ( \p Q_HI, \p Q_LO ) right (down) by \p bytecnt bytes
         *
         * @tparam Q_HI
         * @tparam Q_LO
         *
         * @param bytecnt number of bytes to shift by
         */
        template<unsigned Q_HI, unsigned Q_LO>
        requires (Q_HI < 8 && Q_LO < 8 && Q_HI != Q_LO)
        static inline void rshiftQ(const uint32_t bytecnt) {
            asm volatile (
                "EE.SRCXXP.2Q q%[hi], q%[lo], %[bcnt], %[zero]" "\n"
                :
                : [bcnt] "r" (bytecnt-1),
                  [zero] "r" (0),
                  [hi] "n" (Q_HI), [lo] "n" (Q_LO)
            );
        }

        private:

            template<typename P>
            static constexpr uint32_t elemCnt(P* ptr, const uint32_t numBytes) {
                return numBytes / sizeof(std::remove_reference_t<decltype(*ptr)>);
            }

            template<uint32_t D>
            static inline constexpr uint32_t hwdiv(const uint32_t x) {
                if (std::is_constant_evaluated() || simdutil::ctime::ctc(x/D)) {
                    return x/D;
                } else {
                    uint32_t d = D;
                    asm ("":"+r"(d));
                    const uint32_t result = x/d;
                    [[assume( result == (x/D) )]];
                    return result;
                }
            }

            template<uint32_t D>
            static inline constexpr uint32_t hwmod(const uint32_t x) {
                if (std::is_constant_evaluated() || simdutil::ctime::ctc(x%D)) {
                    return x%D;
                } else {
                    uint32_t d = D;
                    asm ("":"+r"(d));
                    const uint32_t result = x%d;
                    [[assume( result == (x%D) )]];
                    return result;
                }
            }

            template<typename D>
            static inline void clrmem_shrt(D*& ptr, const uint32_t byteCnt) {
                if(byteCnt & 8) {
                    simdutil::ptrs::put<uint64_t>(ptr,0);
                }
                if(byteCnt & 4) {
                    simdutil::ptrs::put<uint32_t>(ptr,0);
                }
                if(byteCnt & 2) {
                    simdutil::ptrs::put<uint16_t>(ptr,0);
                }
                if(byteCnt & 1) {
                    simdutil::ptrs::put<uint8_t>(ptr,0);
                }
            }

            /**
             * @brief Loads up to 31 significant bits from \p gainQ16 into the lowest int32 lane of Q register \p QR,
             * returns the number of bits the resulting value in \c QR[0] is left-shifted from the original \p gainQ16.
             *
             * @tparam QR Q register to load the scaled gain into
             * @tparam BCAST if \c true broadcast the gain into 8x16 bits of QR, else load the gain into the lowest 1x32 bits of QR
             *
             * @param gainFP
             * @param fracBits
             * @return the scale of the gain in bits
             */
            template<unsigned QR, bool BCAST = false>
            requires (QR < 8)
            static inline uint32_t loadFPIntoQ(uint32_t fp, uint32_t fracBits) {

                // Can actually only use 15 bits for fp (< 32768.0) because it has to fit into an int16_t.
                static constexpr uint32_t G_MAX = ((uint32_t)1 << 31) - 1;

                fp = std::min(fp, G_MAX);

                // Extract 15 significant bits from gainQ16:

                uint32_t shift = __builtin_clz(fp); // shift is the "exponent" (negated)

                // assert( shift >= 1 );

                shift -= 1; // MSB must stay 0.

                if constexpr (BCAST) {
                    // Broadcast the upper 16 bits to QR
                    const uint16_t fp16 = (fp << shift) >> 16; // fp16 is the "mantissa"
                    asm volatile (
                        "EE.VLDBC.16 q%[qr], %[fp16]"
                        : 
                        : [fp16] "r" (&fp16),
                          "m" (fp16),
                          [qr] "n" (QR)
                    );
                    shift -= 16;
                } else {
                    // Load up to 32 bits into the lowest 32-bit lane of QR
                    const uint32_t fp32 = (fp << shift); // fp is the "mantissa"

                    // assert( (fp >> 16) >= 0x4000 && (fp >> 16) < 0x8000);

                    // QR[0] := (int16_t)(fp32 & 0xffff) - don't care.
                    // QR[1] := (int16_t)(fp32 >> 16)
                    asm volatile (
                        "EE.MOVI.32.Q q%[qr], %[fp32], 0"
                        :
                        : [fp32] "r" (fp32),
                          [qr] "n" (QR)
                    );

                }

                return shift + fracBits;

            }

            template<unsigned QR>
            requires (QR < 8)
            static inline uint32_t bcastFPIntoQ(uint32_t fp, uint32_t fracBits) {
                return loadFPIntoQ<QR,true>(fp,fracBits);
            }


            /**
             * @brief Loads up to 31 significant bits from \p gainQ16 into the lowest int32 lane of Q register \p QR,
             * returns the number of fractional bits of the resulting value in \c QR[0].
             *
             * @tparam QR Q register to load the scaled gain into
             *
             * @param gainQ16
             * @return the scale of the gain in bits
             */
            template<unsigned QR>
            requires (QR < 8)
            static inline uint32_t loadGainIntoQ(uint32_t gainQ16) {
                return loadFPIntoQ<QR,false>(gainQ16,16);
            }

            /**
             * @brief Loads a bit mask with only a number of \p significantBits set into
             * every 16-bit lane of Q-register \p QR.
             *
             * @tparam QR Q-register to load the mask into
             *
             * @param significantBits number of bits to set; bits 0...(15-significantBits) will
             * be cleared to 0, any others set to 1.
             */
            template<unsigned QR>
            requires (QR < 8)
            static inline void loadMaskIntoQ(const unsigned significantBits) {
                asm volatile (
                    "EE.ZERO.Q q%[qr]"
                    :
                    : [qr] "n" (QR)
                );
                if(significantBits < 16) {
                    const uint16_t n_mask = 0xffffu >> significantBits;

                    // qr[n] := n_mask
                    asm volatile (
                        "EE.VLDBC.16 q%[qr], %[n_mask]" "\n"
                        :
                        : [n_mask] "r" (&n_mask),
                          "m" (n_mask),
                          [qr] "n" (QR)
                    );
                }

                // qr[n] := ~qr[n]
                asm volatile (
                    "EE.NOTQ q%[qr], q%[qr]"
                    :
                    : [qr] "n" (QR)
                );
            }

            #undef MEM_ELEMS
            #undef MEM_BYTES
    };

    using utils = Utils<arch::SOC>;

} // namespace audio
