#include "CFAR_Peak_finder.h"
#include "Vector_maths.h"
#include "Log.h"
#include "float_equality.h"
#include "Centre_of_mass.h"
#include "Shape_finder.h"

namespace Navtech::Navigation {

    CFAR_Peak_finder::CFAR_Peak_finder() : 
       Active { "CFAR Peak Finder "}
    {
    }

    
    // -----------------------------------------------------------------------
    // Public methods
    //
    void CFAR_Peak_finder::configure(
        const Navtech::Networking::Colossus_protocol::TCP::Configuration&       cfg_msg,
        const Navtech::Networking::Colossus_protocol::TCP::Navigation_config    nav_cfg,
        Subresolution_mode                                                      subresolution_mode,
        Buffer_mode                                                             bffr_mode,
        std::size_t                                                             bffr_sz,
        Unit::Bin                                                               guard_sz,
        Peak_mode                                                             peak_type
    ) 
    {
        using namespace Navtech::CFAR;

        buffer_mode = bffr_mode;
        buffer_sz = bffr_sz;
        
        // Set up the CFAR algorithm
        // 
        range_in_bins           = cfg_msg.range_in_bins();
        range_gain              = cfg_msg.range_gain();
        range_offset            = cfg_msg.range_offset();
        range_resolution        = cfg_msg.bin_size() / 10000.0f;

        minimum_bin             = nav_cfg.min_bin_to_operate_on();
        max_peaks               = nav_cfg.max_peaks_per_azimuth();
        min_range               = minimum_bin * range_resolution;
        max_range               = range_in_bins * range_resolution;
        azimuth_samples         = cfg_msg.azimuth_samples();
        steps_per_azimuth       = static_cast<float>(cfg_msg.encoder_size() / azimuth_samples);

        mode                    = subresolution_mode;
        peak_mode               = peak_type;

        if (mode == Subresolution_mode::centre_of_mass_2d) {
            rotation_data.resize(azimuth_samples);
        }

        to_degrees = [this] (float a)  { 
            return fmod(a * 360.0f / static_cast<float>(azimuth_samples) + 360.0f, 360.0f); 
        };
        
        to_metre        = [this] (float b)      { return (b * range_gain * range_resolution) + range_offset; };

        cfar =  Cell_average<float>(
            minimum_bin,
            nav_cfg.bins_to_operate_on(),
            guard_sz, // num guard cells
            nav_cfg.navigation_threshold()
        );

    }


    void CFAR_Peak_finder::find_peaks(
        const Networking::Colossus_protocol::TCP::FFT_data&    fft_msg
    )
    {
        async_call(
            &CFAR_Peak_finder::on_find_peaks,
            this,
            fft_msg.azimuth(),
            fft_msg.to_vector()
        );
    }


    void CFAR_Peak_finder::set_target_callback(std::function<void(const CFAR_Target&)> fn)
    {
        target_callback = std::move(fn);
    }

    
    // ------------------------------------------------------------------------
    // Private members
    //
    void CFAR_Peak_finder::on_find_peaks(            
        Unit::Azimuth azimuth,
        const Networking::Message_buffer& fft_data
    )
    {
        using Navtech::Utility::stdout_log;
        using Navtech::Utility::endl;

        auto power_data = fft_to_floats(fft_data);

        auto azimuth_idx = static_cast<Unit::Azimuth>(azimuth / steps_per_azimuth);

        if (buffer_mode != Buffer_mode::off) buffer_data(azimuth_idx, power_data);
        else process_data(azimuth_idx, power_data);       
    }


    std::vector<float> CFAR_Peak_finder::fft_to_floats(const Networking::Message_buffer& fft_data)
    { 
        std::vector<float> power_data(fft_data.size());
                
        std::transform(fft_data.begin(), fft_data.end(), 
                        power_data.begin(), 
                        [] (std::uint8_t f) { return f * 0.5f; });

        return power_data;
    }


    void CFAR_Peak_finder::buffer_data(Unit::Azimuth azi_idx, const std::vector<float>& data)
    {
        // This should only really be used with staring radar
        //
        std::vector<float> adjusted_data { };
        switch (buffer_mode) {
            case Buffer_mode::off:
                // This case should be unreachable, but just in case:
                //
                process_data(azi_idx, data);
                break;
            case Buffer_mode::average:
                buffered_data.emplace_back(data.begin(), data.end());

                if (buffered_data.size() < buffer_sz) return;

                adjusted_data.resize(range_in_bins);

                for (std::uint16_t bin { 0 }; bin < range_in_bins; ++bin) {
                    float total { 0.0f };
                    for (auto& azimuth : buffered_data) {
                        float value { static_cast<float>(azimuth[bin]) / 2.0f };
                        total += std::pow(10.0f, value / 10.0f);
                    }
                    adjusted_data[bin] = std::log10(total / buffered_data.size()) * 10.0f;
                }

                buffered_data.clear();
                
                process_data(azi_idx, adjusted_data);
                break;
            case Buffer_mode::max:
                float max { };

                adjusted_data.resize(range_in_bins);
                for (std::uint16_t bin = 0; bin < range_in_bins; bin++) {
                    for (std::size_t counter = 0; counter < buffered_data.size(); counter++) {
                        float value { buffered_data[counter][bin] / 2.0f };
                        if (value > max) { max = value; }
                    }
                    adjusted_data[bin] = max;
                }

                break;
        }

        process_data(azi_idx, adjusted_data);
    }


    void CFAR_Peak_finder::process_data(Unit::Azimuth azi_idx, const std::vector<float>& data)
    {
        auto cfar_points = cfar.process(data);

        // It's possible for the data to be contoured
        //
        cfar_points.resize(range_in_bins);
        auto min_it = cfar_points.cbegin() + minimum_bin;
        auto max_it = select_peak(min_it, cfar_points.end());

        // This is so rows aren't skipped, there's certainly a better way of doing this
        //
        if (Utility::essentially_equal(*max_it, 0.0f) && mode != Subresolution_mode::centre_of_mass_2d) return;

        auto forward_it = max_it;
        auto reverse_it = max_it;

        while (*(forward_it+1) != 0 && forward_it != cfar_points.end()) ++forward_it;
        while (*(reverse_it-1) != 0 && reverse_it != (min_it)) --reverse_it;

        auto window_sz = static_cast<Unit::Bin>(std::distance(reverse_it, forward_it));
        auto peak_bin = static_cast<Unit::Bin>(std::distance(cfar_points.cbegin(), max_it));
        
        float           resolved_bin    { };

        switch (mode) {
            case Subresolution_mode::curve_fit:
                resolved_bin = peak_resolve(data, peak_bin, window_sz);
                send_target(resolved_bin, azi_idx);
                break;
            case Subresolution_mode::centre_of_mass: 
                {
                    auto first_bin = std::distance(cfar_points.cbegin(), reverse_it);
                    auto window_start = data.begin() + first_bin;
                    auto window_end = window_start + window_sz;
                    resolved_bin = first_bin + Utility::centre_of_mass(window_start, window_end);
                    send_target(resolved_bin, azi_idx);
                }
                break;
            case Subresolution_mode::centre_of_mass_2d:
                if (azi_idx < last_azimuth) {
                    counter++;
                    if (counter >= 2) {
                        find_shapes(rotation_data);
                    }
                }

                if (counter >= 1) {
                    std::vector<Unit::dB> reduced_points(cfar_points.size(), 0.0);
                    auto peaks { 0 };
                    for (auto i { minimum_bin }; i < cfar_points.size(); i++) {
                        if (Utility::essentially_equal(cfar_points[i], 0.0f)) continue;
                        reduced_points[i] = cfar_points[i];
                        peaks++;
                        if (peaks >= max_peaks) break;
                    }

                    rotation_data[azi_idx] = reduced_points;
                }

                last_azimuth = azi_idx;
                return;
        }
    }


    std::vector<float>::const_iterator CFAR_Peak_finder::select_peak(
            std::vector<float>::const_iterator begin,
            std::vector<float>::const_iterator end
    ) const
    {
        switch (peak_mode) {
            case (Peak_mode::max) :
                return std::max_element(begin, end);
            case (Peak_mode::first): {
                // Find the first non-zero value
                //
                auto first_non_zero = std::find_if(begin, end, [] (float f) { return f > 0.0f; });
                if (first_non_zero == end) return end;

                // Continue until finished rising 
                //
                for (auto itr { first_non_zero }; itr < end - 1; ++itr) {
                    if (*(itr + 1) > *itr) continue;
                    return itr;
                }

                return end - 1;
            }
        }
        
        return end - 1;
    }


    void CFAR_Peak_finder::send_target(float resolved_bin, float resolved_azimuth)
    {
        auto range = to_metre(resolved_bin);
        auto bearing = to_degrees(resolved_azimuth);

        if (std::isinf(range) || range < min_range || range > max_range) return;

        if (target_callback) {
            target_callback({ bearing, range });
        }
    }


    float CFAR_Peak_finder::peak_resolve(
        const std::vector<float>& data,
        Unit::Bin peak_bin,
        Unit::Bin window_sz
    )
    {   
        if (window_sz == 0) return peak_bin;
        window_sz = (window_sz <= 5) ? 5 : window_sz; 
        const std::uint8_t bins_to_offset   { static_cast<uint8_t>((window_sz - 1) / 2) };
        float x[max_bins_to_operate_on]     { 0.0 };
        float y[max_bins_to_operate_on]     { 0.0 };
        auto index                          { 0 };
        auto startValue                     { 0 - bins_to_offset };

        for (index = 0; index < window_sz; index++) {
            x[index] = static_cast<float>(startValue++);
        }

        auto startBin { peak_bin - bins_to_offset };

        for (index = 0; index < window_sz; index++) {
            y[index] = data[startBin + index];
        }

        float Sx  { };
        float Sx2 { };
        float Sx3 { }; 
        float Sx4 { };
        float x2[max_bins_to_operate_on] { };
        float x3[max_bins_to_operate_on] { };
        float x4[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(x, window_sz, Sx);
        Vector_maths::scalar_square(x, window_sz, Sx2);
        Vector_maths::vector_cube(x, window_sz, x3);
        Vector_maths::scalar_sum(x3, window_sz, Sx3);
        Vector_maths::vector_square(x, window_sz, x2);
        Vector_maths::vector_multiply(x2, x2, window_sz, x4);
        Vector_maths::scalar_sum(x4, window_sz, Sx4);

        float Sy   { }; 
        float Sxy  { };
        float Sx2y { };
        float xy[max_bins_to_operate_on]  { };
        float x2y[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(y, window_sz, Sy);
        Vector_maths::vector_multiply(x, y, window_sz, xy);
        Vector_maths::scalar_sum(xy, window_sz, Sxy);
        Vector_maths::vector_multiply(x2, y, window_sz, x2y);
        Vector_maths::scalar_sum(x2y, window_sz, Sx2y);

        float A[4] { Sx2, Sx3, Sx4, Sx2y };
        float B[4] { Sx, Sx2, Sx3, Sxy };
        float C[4] { (float)window_sz, Sx, Sx2, Sy };

        float F = C[0] / A[0];

        for (index = 0; index <= 3; index++) {
            C[index] = C[index] - (F * A[index]);
        }

        F = B[0] / A[0];

        for (index = 0; index <= 3; index++) {
            B[index] = B[index] - (F * A[index]);
        }

        F = C[1] / B[1];

        for (index = 1; index <= 3; index++) {
            C[index] -= F * B[index];
        }

        float b2  { C[3] / C[2] };
        float b1  { (B[3] - B[2] * b2) / B[1] };

        return -b1 / (2 * b2) + startBin + (float)bins_to_offset;
    }


    void CFAR_Peak_finder::find_shapes(const std::vector<std::vector<float>> rotation_data) {

        using namespace Navtech::Utility;

        Shape_finder<float>    shape_finder { minimum_bin };

        auto shape_centres = shape_finder.find_centres(rotation_data);

        for (auto& centre : shape_centres) {
            send_target(
                centre.first,
                centre.second
            );
        }
    }
} // namespace Navtech::Navigation