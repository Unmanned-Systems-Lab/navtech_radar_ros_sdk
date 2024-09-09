// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------
#ifndef SPHERICAL_COORDINATE_H
#define SPHERICAL_COORDINATE_H

#include "Units.h"

namespace Navtech::Euclidean { struct Coordinate; }

namespace Navtech::Spherical {

    // Bearing measured from polar north
    // Inclination measured from the +ve Z axis
    //
    struct Coordinate {
        Unit::Metre   range         { };
        Unit::Degrees bearing       { };
        Unit::Degrees inclination   { };

        Coordinate() = default;
        Coordinate(Unit::Metre rng, Unit::Degrees bng, Unit::Degrees inc) : 
        range { rng }, 
        bearing { inc.to_float() > 180.0f ? bng.to_float() + 180.0f : bng }, 
        inclination { inc.to_float() > 180.0f ? 360.0f - inc.to_float() : inc }
        { }

        Euclidean::Coordinate to_euclidean() const;

        // Comparison
        //
        bool operator==(const Coordinate& rhs) const;
        bool operator!=(const Coordinate& rhs) const;

        std::string to_string() const;
    };

} // namespace Navtech::Spherical

#endif // SPHERICAL_COORDINATE_H