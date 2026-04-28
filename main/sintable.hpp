#pragma once
#include <cstdint>
#include <array>
#include <cmath>

namespace audio {

    /**
     * @brief Lookup table for sine holding only 1/4 wave and mapping all lookups
     * to this.
     * The table itself is static const and should end up in flash instead of RAM.
     * 
     * @tparam SZ number of values for the \e full sine wave
     */
    template<std::size_t SZ>
    requires ((SZ & (SZ-1)) == 0 && SZ >= 4) // restricting to powers of 2 also makes sure the sine wave wraps around cleanly at 2^32
    class QuarterSinTable {
        static inline constexpr std::size_t TABLE_LEN = SZ/4;

        using table_t = std::array<int16_t,TABLE_LEN>;

        private:
            static inline constexpr table_t makeTable(void) {
                constexpr float PI = 3.14159265358979323846f;
                table_t arr {};
                for(std::size_t i = 0; i < TABLE_LEN; ++i) {
                    arr[i] = (int16_t)(sinf(i * ((2*PI/SZ)/4)) * 32767.0f);
                }
                return arr;
            }
        public:
        static inline constexpr table_t table = makeTable();

        /**
         * @brief 
         * 
         * @return Number of entries corresponding to one full sine wave, i.e. \c SZ 
         */
        static inline constexpr std::size_t size(void) noexcept {
            return SZ;
        }

        /**
         * @brief Get the value corresponding to <tt>sin( i * ((2*PI) / SZ) )</tt>.
         * 
         * @param i index to get the sine value for; any value allowed.
         * @return <tt>(int16_t)sin( i * ((2*PI) / SZ) )</tt>
         */
        inline constexpr int16_t operator[](const uint32_t i) const noexcept {
            
            uint32_t p = i % TABLE_LEN;

            if( (i % (2*TABLE_LEN)) >= TABLE_LEN) {
                // Odd quadrant, reversed order:
                p = (TABLE_LEN-1)-p;
            }

            int32_t s = table[p];

            if( (i % (4*TABLE_LEN)) >= (2*TABLE_LEN) ) {
                // Second half, negative values:
                s = -s;
            }

            return s;

        }

    };

    /**
     * @brief Lookup table for sine holding all values for a full wave. Uses 4x the memory
     * (and cache!) of the QuarterSinTable, so may or may not be a little faster on access.
     * The table itself is static const and should end up in flash instead of RAM.
     * 
     * @tparam SZ number of values for the \e full sine wave
     */
    template<std::size_t SZ>
    requires ((SZ & (SZ-1)) == 0 && SZ >= 4)
    class FullSinTable {

        using table_t = std::array<int16_t,SZ>;

        private:
            static inline constexpr table_t makeTable(void) {
                const QuarterSinTable<SZ> st {};
                table_t arr {};
                for(std::size_t i = 0; i < SZ; ++i) {
                    arr[i] = st[i];
                }
                return arr;
            }
        public:

        static constexpr table_t table = makeTable();

        /**
         * @brief 
         * 
         * @return Number of entries corresponding to one full sine wave, i.e. \c SZ 
         */
        static inline constexpr std::size_t size(void) noexcept {
            return SZ;
        }

        /**
         * @brief Get the value corresponding to <tt>sin( i * ((2*PI) / SZ) )</tt>.
         * 
         * @param i index to get the sine value for; any value allowed.
         * @return <tt>(int16_t)sin( i * ((2*PI) / SZ) )</tt>
         */
        inline constexpr int16_t operator[](const uint32_t i) const {
            return table[i%SZ];
        }

    };

} // namespace audio
