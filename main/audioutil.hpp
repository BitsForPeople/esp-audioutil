#pragma once
/*
 * Copyright 2026, <https://github.com/BitsForPeople>
 */
#include <cstdint>
#include <cmath> // ldexpf
#include <algorithm> // std::min
#include <concepts>
#include "arch.hpp"
#include "simdutil.hpp"

namespace audio {

    namespace fp {
        static inline constexpr uint32_t asQ16(const float f) noexcept {
            if (std::is_constant_evaluated()) {
                return (uint32_t)(f * 65536.f);
            } else {
                return (uint32_t)std::ldexpf(f,16); // might not be constexpr prior to C++23
            }
        }

        // template<std::integral I>
        // static constexpr unsigned BITWIDTH = sizeof(I) * 8;

        // template<unsigned FRACBITS = 16, std::signed_integral I = int32_t>
        // requires (!std::is_const_v<I> && BITWIDTH<I> > FRACBITS)
        // struct FP {
        //     using valtype = I;
        //     static constexpr unsigned BITS = BITWIDTH<valtype>;
        //     static constexpr unsigned FRAC_BITS = FRACBITS;
        //     static constexpr unsigned INT_BITS = BITS - FRAC_BITS;

        //     valtype intval {};
        //     constexpr FP(void) = default;
        //     constexpr FP(const FP&) = deafult;
        //     constexpr FP(FP&&) = default;
        //     constexpr 
        // };
    }

    static constexpr uint32_t UNITY_GAIN = fp::asQ16(1.0f);

    /**
     * @brief Basic C implementation of the utility functions as fallback when we don't have specialized variants
     * for our current target SoC.
     * 
     * @tparam SOC The target SoC for which we're building.
     */
    template<arch::SoC SOC>
    struct Utils {
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
                // Bring, portable ANSI C:
                // while(output < out_end) {
                //     *output = *input >> 16;
                //     input += 1;
                //     output += 1;
                // }
            } else {
                while(output < out_end) {
                    *output = ((int64_t)(*input) * gainQ16) >> (16+16);
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
                        *output = (((int64_t)(*input) * gainQ16) >> (16+16)) & mask;
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
            int16_t* out = (int16_t*)output;
            while(input < in_end) {
                out[0] = input[0];
                out[1] = input[0];
                input += 1;
                out += 2;
            }
        }
    };

    /**
     * @brief Specialized implementation of the utility functions using the ESP32-S3's SIMD instructions.
     * 
     */
    template<>
    struct Utils<arch::SoC::ESP32_S3> : public Utils<arch::SoC::OTHER> {

        static constexpr const char* TAG = "audio::Utils";

        using simd = simdutil::Util<arch::SoC::ESP32_S3>;

        static constexpr std::size_t VEC_LEN_BYTES = simd::VEC_LEN_BYTES;         


        /* Forms a memory reference to \c bcnt bytes pointed to by \c ptr */
        #define MEM_BYTES(ptr,bcnt) (*((std::remove_reference_t<typeof(*ptr)>(*)[(bcnt) / sizeof(std::remove_reference_t<typeof(*ptr)>)])ptr))

        /* Forms a memory reference to \c cnt elements of the array pointed to by \c ptr */
        #define MEM_ELEMS(ptr,cnt) (*(std::remove_reference_t<typeof(*ptr)>(*)[cnt])ptr)


        [[gnu::noinline]]
        static inline void reduce32to16(const int32_t* input, int16_t* output, uint32_t sampleCnt, uint32_t gainQ16 = UNITY_GAIN) {

            using IN_t = std::remove_reference_t<typeof(*input)>;
            using OUT_t = std::remove_reference_t<typeof(*output)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 2.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 4.


            // q7[1] := (gain << shift)
            const uint32_t shift = loadGainIntoQ<7>(gainQ16);

            static constexpr unsigned Q_OUT = 0;
            static constexpr unsigned Q_TMP = 1;

            static constexpr std::size_t VEC_LEN_BYTES = simd::VEC_LEN_BYTES; 

            
            auto f1 = [&input, &shift](const uint32_t outByteCnt) {
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
                    "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n" // qout[n] := QACC[n] >> sh

                    : 
                    : [input] "r" (input),
                      [shift] "r" (shift),
                      "m" (MEM_BYTES(input,inByteCnt)),
                      [out] "n" (Q_OUT)
                );
                // input += elemCnt;
                simdutil::ptrs::incp(input,inByteCnt);
            };

            auto floop = [&input, &shift](OUT_t*& output, const uint32_t outByteCnt) {
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

                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n" // q3[n] = QACC[n] >> sh
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


        [[gnu::noinline]]
        static inline void reduce32to16(const int32_t* input, int16_t* output, const uint32_t sampleCnt, uint32_t gainQ16, const uint32_t significantBits) {
            
            using IN_t = std::remove_reference_t<typeof(*input)>;
            using OUT_t = std::remove_reference_t<typeof(*output)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 2.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 4.


            static constexpr unsigned Q_OUT = 0;
            static constexpr unsigned Q_TMP = 1;

            static constexpr unsigned Q_MASK = 6;
            static constexpr unsigned Q_GAIN = 7;            
            /*
                Step 1: Set up q7[1] with 15 significant bits from gainQ16
            */

            const uint32_t shift = loadGainIntoQ<Q_GAIN>(gainQ16);

            /*
                Step 2: Set up q6 with the bit mask
            */

            loadMaskIntoQ<Q_MASK>(significantBits);



            /*
                "Step" 3: Define the processing functions
            */



            auto f1 = [&input, &shift](const uint32_t outByteCnt) {
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
                    "EE.SRCMB.S16.QACC q%[out], %[shift], 0" "\n" // qout[n] := QACC[n] >> sh

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

            auto floop = [&input, &shift](int16_t*& output, const uint32_t outByteCnt) {
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

                        "EE.SRCMB.S16.QACC q3, %[shift], 0" "\n" // q3[n] = QACC[n] >> sh

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
         * @brief Reads \p sampleCnt samples from \p input, duplicates each of them, and writes the 2* \p sampleCnt
         * samples to \p output
         * 
         * @param input 
         * @param output must be \e 4-byte aligned!
         * @param sampleCnt 
         */
        template<typename OUT_t>
        requires (sizeof(OUT_t) == 2*sizeof(int16_t) && alignof(OUT_t) >= 2*alignof(int16_t))
        [[gnu::noinline]]
        static inline void monoToStereo(const int16_t* input, OUT_t* output, uint32_t sampleCnt) {

            using IN_t = std::remove_reference_t<typeof(*input)>;

            static constexpr std::size_t OUT_ELEM_SZ = sizeof(OUT_t); // 4.
            static constexpr std::size_t IN_ELEM_SZ = sizeof(IN_t); // 2.

            // static constexpr unsigned OUT_TO_IN_RATIO = OUT_ELEM_SZ / IN_ELEM_SZ; // 2.

            // Memory barrier for the compiler:
            asm volatile (""::"m" (MEM_ELEMS(input, sampleCnt)));
            asm volatile ("":"=m" (MEM_ELEMS(output, sampleCnt)));

            static constexpr unsigned Q_OUT = 2;
            static constexpr unsigned Q_TMP = 3;



            auto f1 = [&input](const uint32_t outByteCnt) {
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

            auto floop = [&input](OUT_t*& output, const uint32_t outByteCnt) {
                const uint32_t elemCnt = outByteCnt / OUT_ELEM_SZ;
                const uint32_t inByteCnt = elemCnt * IN_ELEM_SZ;

                if(((uintptr_t)input % VEC_LEN_BYTES) == 0) {
                    // Input is already aligned; full steam ahead:
                    // Do 32-byte chunks of input in the loop to avoid a pipeline stall:
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
                        asm volatile (
                            // "LOOPNEZ %[cnt], .Lend_%=" "\n"
                                "EE.VLDHBC.16.INCP q0, q1, %[input]" "\n"
                                // Pipeline stall for 1 clock cycle here.
                                "EE.VST.128.IP q0, %[output], 16" "\n"
                                "EE.VST.128.IP q1, %[output], 16" "\n"
                            // ".Lend_%=:" "\n"
                            : [input] "+r" (input),
                              [output] "+r" (output),
                              "=m" (MEM_BYTES(output,outByteCnt))
                            : // [cnt] "r" (outByteCnt / (2*VEC_LEN_BYTES)),
                              "m" (MEM_BYTES(input,inByteCnt))
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
         * @brief Shifts Q-register \p Q_REG right (down) by \p BYTECNT bytes while
         * inserting the lowest \p BYTECNT bytes of \p data into the upper-most
         * bytes of \p Q_REG.
         * 
         * @tparam BYTECNT number of bytes to shift by/insert (1...4)
         * @tparam Q_REG Q-register to shift
         * @tparam Q_TMP temporary Q-register (clobbered)
         * 
         * @param data data to shift into the upper bytes of \p Q_REG
         */
        template<std::size_t BYTECNT, unsigned Q_REG, unsigned Q_TMP>
        requires (BYTECNT > 0 && BYTECNT <= 4 && Q_REG < 8 && Q_TMP < 8 && Q_REG != Q_TMP)
        static inline void rshiftBytesIntoQ(const uint32_t data) {
            asm volatile (
                "EE.MOVI.32.Q q%[tmp], %[data], 0" "\n"
                "EE.SRCI.2Q q%[tmp], q%[tmp], (%[bcnt]-1)" "\n"
                :
                : [data] "r" (data),
                  [reg] "n" (Q_REG),
                  [tmp] "n" (Q_TMP),
                  [bcnt] "n" (BYTECNT)
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
                return numBytes / sizeof(std::remove_reference_t<typeof(*ptr)>);
            }
            
            /**
             * @brief Loads up to 31 significant bits from \p gainQ16 into the lowest int32 lane of Q register \p QR,
             * returns the number of bits the resulting value in \c QR[0] is left-shifted from the original \p gainQ16.
             * 
             * @tparam QR Q register to load the scaled gain into
             * 
             * @param gainQ16 
             * @return the scale of the gain in bits 
             */
            template<unsigned QR>
            requires (QR < 8)
            static inline uint32_t loadGainIntoQ(uint32_t gainQ16) {

                // Can actually only use 15 bits for gain (< 32768.0) because it has to fit into an int16_t.
                static constexpr uint32_t G_MAX = ((uint32_t)1 << 31) - 1;

                gainQ16 = std::min(gainQ16, G_MAX);

                // Extract 15 significant bits from gainQ16:

                uint32_t shift = __builtin_clz(gainQ16); // shift is the "exponent" (negated)

                // assert( shift >= 1 );

                shift -= 1; // MSB must stay 0.
                {
                    const uint32_t g32 = (gainQ16 << shift); // g32 is the "mantissa"

                    // QR[0] := (int16_t)(g32 & 0xffff) - don't care.
                    // QR[1] := (int16_t)(g32 >> 16)
                    asm volatile (
                        "EE.MOVI.32.Q q%[qr], %[g32], 0"
                        :
                        : [g32] "r" (g32),
                          [qr] "n" (QR)
                    );

                }

                return shift;

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

                    // q6[n] := n_mask
                    asm volatile (
                        "EE.VLDBC.16 q%[qr], %[n_mask]" "\n"
                        :
                        : [n_mask] "r" (&n_mask),
                          "m" (n_mask),
                          [qr] "n" (QR)
                    );
                }

                // q6[n] := ~q6[n]
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