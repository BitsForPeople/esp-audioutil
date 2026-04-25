#pragma once
/*
 * Copyright 2025, <https://github.com/BitsForPeople>
 */

 #include <cstdint>
 #include <type_traits>
 #include <functional>
 #include "arch.hpp"

//  #include <esp_log.h>

 namespace simdutil {

    /**
     * @brief Compile-time helper functions.
     * 
     */
    namespace ctime {
        template<typename E>
        static inline constexpr bool ctc(E expr) noexcept {
            return __builtin_constant_p(expr);
        }

        static inline constexpr bool known(const bool cond) noexcept {
            return ctc(cond) && cond;
        }

        static inline constexpr bool maybe(const bool cond) noexcept {
            return !known(!cond);
        }
    } // namespace ctime 

    /**
     * @brief Pointer helper functions.
     * 
     */
    namespace ptrs {

        template<typename T, typename P>
        requires (!std::is_const_v<P> || std::is_const_v<T>)
        static inline T& as(P* ptr) {
            return *(T*)ptr;
        }

        template<typename P>
        static inline P* pplus(P* const ptr, const signed inc) {
            return (P*)((uintptr_t)ptr + inc);
        }

        template<typename P>
        static inline void incp(P*& ptr, const signed inc) {
            ptr = pplus(ptr,inc);
        }

        template<typename T, typename P>
        static inline void put(P*& ptr, const T& value) {
            as<T>(ptr) = value;
            incp(ptr,sizeof(T));
        }

    };

    namespace impl {

        template<arch::SoC SOC>
        struct Impl {

            /**
             * @brief Clamps/saturates the given signed value \p v to 
             * a signed value of \p BITS bits (incl. sign bit).
             * 
             * 
             * @tparam BITS number of bits to clamp to; default: 16
             * @param v value to saturate
             * @return \p v saturated to the range -(1<<(BITS-1))...+(1<<(BITS-1))-1
             */
            template<unsigned BITS = 16>
            requires (BITS >= (7+1) && BITS <= (22+1))
            static inline constexpr int32_t clamp(const int32_t v) {
                static constexpr int32_t MIN = -(1<<(BITS-1));
                static constexpr int32_t MAX = -1 - MIN;
                return std::min(std::max(v,MIN),MAX);
            }

        };

        template<>
        struct Impl<arch::SoC::ESP32> : public Impl<arch::SoC::OTHER> {
            // The inline assembly in this function prevents some gcc optimizations, sometimes causing a net loss in performance.
            template<unsigned BITS = 16>
            requires (BITS >= (7+1) && BITS <= (22+1))
            static inline constexpr int32_t clamp_(const int32_t v) {
                static constexpr int32_t MIN = -(1<<(BITS-1));
                static constexpr int32_t MAX = -MIN - 1;
                if(std::is_constant_evaluated() || ctime::known(v)) {
                    return Impl<arch::SoC::OTHER>::clamp(v);
                } else {
                    int32_t r;
                    asm (
                        "CLAMPS %[r], %[v], %[bits]"
                        : [r] "=r" (r)
                        : [v] "r" (v),
                          [bits] "n" (BITS-1)
                    );
                    [[assume(r >= MIN)]];
                    [[assume(r <= MAX)]];
                    [[assume((r & ~(((uint32_t)1<<BITS)-1)) == 0)]];
                    return r;
                }
            } 
        };

        template<>
        struct Impl<arch::SoC::ESP32_S3> : public Impl<arch::SoC::ESP32> {
            /**
             * @brief Writes the lower 0...15 bytes from Q register \p QR0 to memory at \p dst.
             * \p QR1 is used as a temporary, and both \p QR0 and \p QR1 are clobbered.
             * 
             * @tparam QR0 
             * @tparam QR1 
             * @param dst 
             * @param numBytes 
             */
            template<unsigned QR0, unsigned QR1>
            requires (QR0 < 8 && QR1 < 8 && QR0 != QR1)
            static inline void writeUnaligned(void* dst, const unsigned numBytes, uint32_t& dummy) {

                uint32_t tmp; // used both as a temporary and for dependency tracking.
                asm ("":"=r"(tmp): "m" (dummy)); // tie tmp to dummy

                if(numBytes & 1) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    if(ctime::maybe((numBytes & 2) || (numBytes & 4) || (numBytes & 8))) {
                        asm (
                            "EE.SRCI.2Q q%[qr1], q%[qr0], 1-1" "\n"
                            : "+r" (tmp)
                            : [qr0] "n" (QR0), [qr1] "n" (QR1)
                        );  
                    }
                    ptrs::put<uint8_t>(dst,tmp);
                }
                if(numBytes & 2) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    if(ctime::maybe((numBytes & 4) || (numBytes & 8))) {
                        asm (
                            "EE.SRCI.2Q q%[qr1], q%[qr0], 2-1" "\n"
                            : "+r" (tmp)
                            : [qr0] "n" (QR0), [qr1] "n" (QR1)
                        );  
                    }
                    ptrs::put<uint16_t>(dst,tmp);
                }
                if(numBytes & 4) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    if(ctime::maybe(numBytes & 8)) {
                        asm (
                            "EE.SRCI.2Q q%[qr1], q%[qr0], 4-1" "\n"
                            : "+r" (tmp)
                            : [qr0] "n" (QR0), [qr1] "n" (QR1)
                        );  
                    }
                    ptrs::put<uint32_t>(dst,tmp);
                }
                if(numBytes & 8) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : 
                        [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    ptrs::put<uint32_t>(dst,tmp);
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 1" "\n"
                        : 
                        [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    ptrs::put<uint32_t>(dst,tmp);
                }

                asm ("":"+m" (dummy):"r" (tmp)); // tie dummy to tmp
            }

            /**
             * @brief Writes the lower 0...15 bytes from Q register \p QR0 to memory at \p dst.
             * \p dst \e must point to an at least 8-byte-aligned address.
             * \p QR1 is used as a temporary, and both \p QR0 and \p QR1 are clobbered.
             * 
             * @tparam QR0 
             * @tparam QR1 
             * @param dst 
             * @param numBytes 
             */
            template<unsigned QR0, unsigned QR1>
            requires (QR0 < 8 && QR1 < 8 && QR0 != QR1)
            static inline void writeTail(void* dst, const unsigned numBytes, uint32_t& dummy) {
                // *dst is assumed to be aligned (usually 16-byte, min. 8-byte)
                uint32_t tmp; // used both as a temporary and for dependency tracking.
                asm ("":"=r"(tmp): "m" (dummy)); // tie tmp to dummy

                if(numBytes & 8) {
                    asm (
                        "EE.VST.L.64.IP q%[qr0], %[dst], 8" "\n"
                        "EE.SRCI.2Q q%[qr1], q%[qr0], 8-1" "\n"
                        : [dst] "+r" (dst),
                        "=m" (*(uint64_t*)dst),
                        "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                }
                if(numBytes & 4) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        "EE.SRCI.2Q q%[qr1], q%[qr0], 4-1" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    ptrs::put<uint32_t>(dst,tmp);
                }
                if(numBytes & 2) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    if(ctime::maybe(numBytes & 0x1)) {
                        asm (
                            "EE.SRCI.2Q q%[qr1], q%[qr0], 2-1" "\n"
                            : "+r" (tmp)
                            : [qr0] "n" (QR0), [qr1] "n" (QR1)
                        );  
                    }
                    ptrs::put<uint16_t>(dst,tmp);
                }

                if(numBytes & 1) {
                    asm (
                        "EE.MOVI.32.A q%[qr0], %[tmp], 0" "\n"
                        : [tmp] "+r" (tmp)
                        : [qr0] "n" (QR0), [qr1] "n" (QR1)
                    );
                    ptrs::put<uint8_t>(dst,tmp);
                }
                asm ("":"+m" (dummy):"r" (tmp)); // tie dummy to tmp
            }

            
            template<typename P>
            static inline void splitForAlignment(P* const ptr, const uint32_t& outByteLen, uint32_t& first, uint32_t& rem) {
                rem = outByteLen;
                first = ((uintptr_t)ptr + 15) & ~0xf;
                first = first - (uintptr_t)ptr;
                if(first) {
                    first = std::min(first,rem);
                    rem -= first;
                }
            }

            template<typename P>
            static inline uint32_t getOff(P* const ptr, const uint32_t outByteLen) {
                uint32_t off = ((uintptr_t)ptr + 15) & ~0xf;
                off = off - (uintptr_t)ptr;
                off = std::min(off,outByteLen);
                return off;
            }

            static constexpr std::size_t VEC_LEN_BYTES = 16;

        };
    } // namespace impl

    template<arch::SoC SOC>
    struct Util {
        private:
            // static constexpr const char* TAG = "simd::Util";
            using imp = impl::Impl<SOC>;
        public:

        static constexpr std::size_t VEC_LEN_BYTES = imp::VEC_LEN_BYTES;

        template<typename E> 
        requires (sizeof(E) <= VEC_LEN_BYTES && (VEC_LEN_BYTES % sizeof(E)) == 0)   
        static constexpr std::size_t VEC_ELEM_CNT = VEC_LEN_BYTES / sizeof(E);

        // TODO: Might be cleaner to use a single class instead of two interdependent functors.
        /**
         * @brief Manages SIMD processing with output to potentially unaligned memory.
         * The output in memory is specified via \p dst and \p outByteLen .
         * Depending on the initial alignment of \c *dst and the number of bytes of
         * output to produce this function will
         * 
         *   - initially invoke functor \p f1 to produce one vector of output
         * 
         *   - write to the output as many of the produced bytes as needed to get to 16-byte alignment
         * 
         *   - invoke \p floop to produce as many full vectors of aligned output as possible
         * 
         *   - invoke \p f1 once more to get one final vector of output
         * 
         *   - write the remaining bytes from the final vector to the output
         * 
         * Note: \p f1 is responsible for advancing any and all \e input pointers
         * 
         * Note: \p floop is responsible for advancing any and all \e input pointers and the output pointer \c dst
         * 
         * @tparam Q_OUT Q register in which \p f1 returns its output data
         * @tparam Q_TMP temporary Q register used for alignment (clobbered!)
         * @tparam F1_t type of \p f1 functor
         * @tparam FLOOP_t type of \p floop functor
         * @tparam D data type of the output memory
         * @tparam Args type(s) of any additional arguments to pass to \p f1 and \p floop
         * 
         * @param f1 
         * @param floop 
         * @param dst 
         * @param outByteLen length of the processing in terms of the number of bytes of output to produce
         * @param args any additional arguments to pass to \p f1 and \p floop
         */
        template<unsigned Q_OUT = 0, unsigned Q_TMP = (Q_OUT + 1) % 8, typename F1_t, typename FLOOP_t, typename D, typename ... Args>
        requires (Q_OUT < 8 && Q_TMP < 8 && Q_OUT != Q_TMP &&
            std::invocable<F1_t,uint32_t,Args...> &&
            std::invocable<FLOOP_t,D*&,uint32_t,Args...>)
        static inline void process(F1_t& f1, FLOOP_t& floop, D* dst, uint32_t outByteLen, Args&&... args) {

            if constexpr (!std::is_void_v<D>) {
                [[assume((outByteLen % sizeof(D)) == 0)]];
            }
            uint32_t dummy;
            asm ("":"=m"(dummy));
            {
                // Make is so that *dst is 16-bytes aligned, if it's not already.
                const uint32_t off = imp::getOff(dst,outByteLen);
                if(off != 0) {
                    // Get one vector of output:
                    std::invoke(f1, off, std::forward<Args...>(args)...);
                    if constexpr (!std::is_void_v<D>) {
                        [[assume((off % sizeof(D)) == 0)]];
                    }
                    // Write as many bytes as needed for alignment to *dst:
                    imp::template writeUnaligned<Q_OUT,Q_TMP>(dst,off,dummy);
                    // Adjust dst to reflect the new (aligned) output location:
                    ptrs::incp(dst,off);
                    // Decrement outByteLen by the number of bytes already done:
                    outByteLen -= off;
                }
            }
            // *dst is now 16-bytes aligned:
            // assert( ((uintptr_t)dst % VEC_LEN_BYTES) == 0 );
            if((outByteLen / VEC_LEN_BYTES) != 0) {
                // Process full vectors in a loop: 
                std::invoke(floop, dst, outByteLen, std::forward<Args...>(args)...);
                outByteLen = outByteLen % VEC_LEN_BYTES;
            }

            // assert( outByteLen < VEC_LEN_BYTES );
            if(outByteLen != 0) {
                // Get final vector of output:
                std::invoke(f1, outByteLen, std::forward<Args...>(args)...);

                if constexpr (!std::is_void_v<D>) {
                    [[assume((outByteLen % sizeof(D)) == 0)]];
                }
                /*
                  We assume that *dst is (still) 16-bytes aligned here, i.e. that
                  floop did *not* change the alignment of *dst!
                */
                // Write remaining 1...15 bytes to *dst:
                // gcc 14.2 fails to optimize these branches to simple bbci instructions like it does with writeUnaligned() above :-/
                imp::template writeTail<Q_OUT,Q_TMP>(dst,outByteLen,dummy);
                // imp::template writeUnaligned<Q_OUT, Q_TMP>(dst,outByteLen,dummy);
            }
        }
    };

    template<unsigned Q_OUT = 0, unsigned Q_TMP = (Q_OUT + 1) % 8, typename F1_t, typename LOOP_t, typename D, typename ... Args>
    requires (Q_OUT < 8 && Q_TMP < 8 && Q_OUT != Q_TMP)
    inline void process(F1_t& f1, LOOP_t& floop, D* dst, uint32_t outByteLen, Args&&... args) {
        Util<arch::SOC>::process<Q_OUT, Q_TMP, F1_t, LOOP_t, D, Args...>(f1, floop, dst, outByteLen, std::forward<Args>(args)...);
    }

 } // namespace simdutil