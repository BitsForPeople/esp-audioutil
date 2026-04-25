#pragma once
/*
 * Copyright 2025, <https://github.com/BitsForPeople>
 */

#ifdef __XTENSA__
#include <xtensa/config/core-isa.h>
#endif
#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif
#include <freertos/FreeRTOS.h> // configNUMBER_OF_CORES

namespace arch {

    enum class SoC {
        ESP32,
        ESP32_S3,
        ESP32_C3,
        OTHER
    };

    static inline constexpr SoC SOC =
        #if CONFIG_IDF_TARGET_ESP32S3
            SoC::ESP32_S3;
        #elif CONFIG_IDF_TARGET_ESP32C3
            SoC::ESP32_C3;
        #elif CONFIG_IDF_TARGET_ESP32
            SoC::ESP32;
        #else
            SoC::OTHER;
        #endif   

    static inline constexpr bool ESP32_S3 = SOC == SoC::ESP32_S3;

    static inline constexpr bool ESP32_C3 = SOC == SoC::ESP32_C3;

    static inline constexpr bool ESP32_OTHER = SOC == SoC::OTHER;


    enum class Arch {
        XTENSA,
        RISCV,
        OTHER
    };

    static inline constexpr Arch ARCH =
        #if __XTENSA__
            Arch::XTENSA;
        #elif __riscv
            Arch::RISCV;
        #else
            Arch::OTHER;
        #endif 


    /**
     * @brief Are we running on a RISC-V architecture?
     * 
     */
    static inline constexpr bool IS_RISCV = ARCH == Arch::RISCV;

    /**
     * @brief Are we running on an Xtensa architecture?
     * 
     */
    static inline constexpr bool IS_XTENSA = ARCH == Arch::XTENSA;

    static inline constexpr bool IS_OTHER = ARCH == Arch::OTHER;    


    static inline constexpr bool MULTICORE =
        #if configNUMBER_OF_CORES > 1
            true;
        #else
            false;
        #endif

    static inline constexpr bool SINGLECORE = !MULTICORE;

    /**
     * @brief Do we have the Xtensas' atomic compare and set?
     * 
     */
    static inline constexpr bool XT_CAS = IS_XTENSA &&
        #if XCHAL_HAVE_S32C1I
            true;
        #else
            false;
        #endif

    /**
     * @brief Do we have the Xtensas' CLAMPS instruction for saturating?
     * 
     */
    static inline constexpr bool XT_CLAMPS = IS_XTENSA &&
        #if XCHAL_HAVE_CLAMPS
            true;
        #else
            false;
        #endif

    /**
     * @brief Do we have the Xtensas' memory ordering loads/stores?
     * 
     */
    static inline constexpr bool XT_ACQ_REL = IS_XTENSA &&
        #if XCHAL_HAVE_RELEASE_SYNC
            true;
        #else
            false;
        #endif

    /**
     * @brief Do we have the RISC-V extension ("A") for atomic memory accesses?
     * 
     */
    static inline constexpr bool RV_ATOMICS = IS_RISCV &&
        #ifdef __riscv_a
            true;
        #else
            false;
        #endif

    /**
     * @brief Do we have instructions for atomic memory accesses?
     * 
     */
    static inline constexpr bool ATOMICS =
        (XT_CAS && XT_ACQ_REL) ||
        RV_ATOMICS;

    /**
     * @brief Do we have Xtensa's zero-overhead loops?
     * 
     */
    static inline constexpr bool XT_LOOPS = IS_XTENSA &&
        #if XCHAL_HAVE_LOOPS
            true;
        #else
            false;
        #endif

                
    /**
     * @brief Does __builtin_clz() map to a hardware instruction?
     * 
     */
    static constexpr bool HW_CLZ =
        #if XCHAL_HAVE_NSA || defined(__riscv_zbb)
            true;
        #else
            false;
        #endif

    /**
     * @brief Does __builtin_ctz() map to a hardware instruction?
     * 
     */
    static constexpr bool HW_CTZ =
        #ifdef __riscv_zbb
            true;
        #else
            false;
        #endif

    /**
     * @brief Do we have hardware min/max operations?
     * 
     */
    static constexpr bool HW_MINMAX =
        #if XCHAL_HAVE_MINMAX
            true;
        #else
            false;
        #endif
 

}