#ifndef FFT_TYPES_H
#define FFT_TYPES_H

#include "Fixed_point.h"

namespace Navtech::Unit {

    using dB = float;

    class FFT_8bit {
    public:
        using Fixed_8bit = Utility::Fixed<8, 1>;

        constexpr FFT_8bit() = default;

        constexpr FFT_8bit(std::uint8_t data) : value { Fixed_8bit::from_raw(data) }
        {
        }

        constexpr dB to_dB() const
        {
            return static_cast<dB>(value.to_float());
        }

        constexpr std::uint8_t raw() const
        {
            return value.raw();
        }

        FFT_8bit& operator=(const Unit::dB& rhs)
        {
            value = rhs;
            return *this;
        }

        static FFT_8bit* overlay_at(std::uint8_t* const addr)                { return reinterpret_cast<FFT_8bit*>(addr); }
        static const FFT_8bit* overlay_at(const std::uint8_t* const addr)    { return reinterpret_cast<const FFT_8bit*>(addr); }


    private:
        Fixed_8bit value { };
    };


    class FFT_16bit {
    public:
        using Fixed_16bit = Utility::Fixed<16, 8>;

        constexpr FFT_16bit() = default;

        constexpr FFT_16bit(std::uint16_t data) : value { Fixed_16bit::from_raw(data) }
        {
        }

        constexpr dB to_dB() const
        {    
            return static_cast<dB>(value.to_float() * scale_factor);
        }

        constexpr std::uint16_t raw() const
        {
            return value.raw();
        }

        FFT_16bit& operator=(const Unit::dB& rhs)
        {
            value = static_cast<float>(rhs) / scale_factor;
            return *this;
        }

        static FFT_16bit* overlay_at(std::uint8_t* const addr)                { return reinterpret_cast<FFT_16bit*>(addr); }
        static const FFT_16bit* overlay_at(const std::uint8_t* const addr)    { return reinterpret_cast<const FFT_16bit*>(addr); }

    private:
        Fixed_16bit value { };

        // 16-bit FFT data is quantized differently to 8-bit.
        // The dynamic range is the same (96.5dB) but that value
        // gives a full-scale output of 141.5.  Therefore, the
        // fixed-point value has to be re-scaled to give the correct
        // actual dB value.
        //
        constexpr static float full_scale      { 141.5f };
        constexpr static float max_dB          { 96.5f };
        constexpr static float scale_factor    { max_dB / full_scale };
    };


} // namespace Navtech::Unit


constexpr inline Navtech::Unit::dB operator""_dB(unsigned long long val)
{
    return static_cast<Navtech::Unit::dB>(val);
}


constexpr inline Navtech::Unit::dB operator""_dB(long double val)
{
    return static_cast<Navtech::Unit::dB>(val);
}


#endif // FFT_TYPES_H