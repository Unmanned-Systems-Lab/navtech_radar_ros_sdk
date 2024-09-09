#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>

#include "CFAR_algorithms.h"

using namespace std;
using namespace Navtech;
using namespace Navtech::CFAR;
using namespace Navtech::Unit;


class GivenACFARAlgorithm : public ::testing::Test {
protected:
    GivenACFARAlgorithm() = default;
};


TEST_F(GivenACFARAlgorithm, ConstantLevelBelowThreshold)
{   
    //             
    // ______________________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, AlternatingNoiseBelowThreshold)
{
    //              
    // _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, PeakAboveNoiseLowThreshold)
{
    //               _
    // _-_-_-_-_-_-_- -_-_-_-_-_-_-_-
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 30, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 10.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);

    ASSERT_FLOAT_EQ(output[0].range, 15.0f);
    ASSERT_FLOAT_EQ(output[0].power, 30.0f);
}


TEST_F(GivenACFARAlgorithm, PeakAboveNoiseBelowThreshold)
{
    //               _
    // _-_-_-_-_-_-_- -_-_-_-_-_-_-_-
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 20, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 10.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, Spike)
{
    //              |
    // _____________|________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 90, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);
    ASSERT_FLOAT_EQ(output[0].range, 15.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, NarrowPeak)
{
    //              -
    // ____________- -_______________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 45, 90, 45, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);
    ASSERT_FLOAT_EQ(output[0].range, 13.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, WidePeak)
{
    //             _-_
    // ___________-   -______________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 30, 60, 90, 60, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 3);
    ASSERT_FLOAT_EQ(output[0].range, 12.0f);
    ASSERT_FLOAT_EQ(output[0].power, 60.0f);

    ASSERT_FLOAT_EQ(output[1].range, 13.0f);
    ASSERT_FLOAT_EQ(output[1].power, 90.0f);
    
    ASSERT_FLOAT_EQ(output[2].range, 14.0f);
    ASSERT_FLOAT_EQ(output[2].power, 60.0f);
}


TEST_F(GivenACFARAlgorithm, BroadPeak)
{
    //           __----__
    // ________--        --__________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 30, 60, 90, 90, 90, 60, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 3);
    ASSERT_FLOAT_EQ(output[0].range, 12.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);

    ASSERT_FLOAT_EQ(output[1].range, 13.0f);
    ASSERT_FLOAT_EQ(output[1].power, 90.0f);
    
    ASSERT_FLOAT_EQ(output[2].range, 14.0f);
    ASSERT_FLOAT_EQ(output[2].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakCloseToWindowSize)
{
    //     _-----------------_         
    // ___-                   -______
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 30, 60, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 60, 30, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, DoublePeak)
{
    //            |   |   
    // ___________|---|______________ 
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 90, 30, 30, 90, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 2);

    ASSERT_FLOAT_EQ(output[0].range, 12.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);

    ASSERT_FLOAT_EQ(output[1].range, 15.0f);
    ASSERT_FLOAT_EQ(output[1].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakAtStartOfRange)
{
    // -             
    //  -____________________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 90, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);

    ASSERT_FLOAT_EQ(output[0].range, 0.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakWithinFirstHalfWindow)
{
    //  -             
    // - -___________________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 30, 90, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);

    ASSERT_FLOAT_EQ(output[0].range, 2.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakAtEndOfRange)
{
    //                              -
    // ____________________________-
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 30, 90 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);

    ASSERT_FLOAT_EQ(output[0].range, 29.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakWithinLastHalfWindow)
{
    //                             -
    // ___________________________- -
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 30, 90, 30 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);

    ASSERT_FLOAT_EQ(output[0].range, 28.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}



TEST_F(GivenACFARAlgorithm, ToMetreCallbackWithRangeGain)
{
    //              |
    // _____________|________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 90, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 0 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return (b * 0.175238f * 0.99f) + 0.0f; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);
    ASSERT_FLOAT_EQ(output[0].range, 2.6022843f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}



TEST_F(GivenACFARAlgorithm, NonZeroStartOffset)
{
    //              |
    // _____________|________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 90, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 10 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};
    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);
    ASSERT_FLOAT_EQ(output[0].range, 15.0f);
    ASSERT_FLOAT_EQ(output[0].power, 90.0f);
}


TEST_F(GivenACFARAlgorithm, PeakBeforeMinBin)
{
    // -             
    //  -____________________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 90, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 10 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, WindowSizeGreaterThanMinBin)
{
    // -             
    //  -____________________________
    // ------------------------------
    //                  0                                      10                                      20
    //                  |                                       |                                       |
    vector<dB> input { 90, 30, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 5 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 45.0f };

    CFAR::Cell_average<dB> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 0);
}


TEST_F(GivenACFARAlgorithm, TestWith8bitData)
{
    //              |
    // _____________|________________
    // ------------------------------
    //                      0                                      10                                      20
    //                      |                                       |                                       |
    
    // NOTE: 8-bit FFT data is stored as half-dB values
    //
    vector<uint8_t> input { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 90, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    Bin start        { 10 };
    Bin window_sz    { 11 };
    Bin guard_cells  { 2 };
    dB threshold     { 30.0f };

    CFAR::Cell_average<uint8_t> ca_cfar { start, window_sz, guard_cells, threshold, [](Bin b) { return b; }};

    auto output = ca_cfar.points(input.data(), input.size());

    ASSERT_EQ(output.size(), 1);
    ASSERT_FLOAT_EQ(output[0].range, 15.0f);
    ASSERT_FLOAT_EQ(output[0].power, 45.0f);
}

