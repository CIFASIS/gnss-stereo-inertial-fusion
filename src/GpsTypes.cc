/**
* This file is part of GNSS-SI. GNSS-SI is based on ORB-SLAM3.
* Copyright (C) 2023 Javier Cremona, Ernesto Kofman and Taihú Pire, CIFASIS (CONICET-UNR).
* Copyright (C) 2023 Javier Civera, University of Zaragoza.
*
* GNSS-SI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GNSS-SI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with GNSS-SI.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "GpsTypes.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <iostream>

namespace ORB_SLAM3
{

namespace GlobalPosition
{

Eigen::Vector3d GpsMeasurement::getRelativeFromOrigin(const GlobalPosition& origin) const
{        
    try
    {
        const GpsMeasurement& gpsMeasOrigin = dynamic_cast<const GpsMeasurement&>(origin);
        GeographicLib::LocalCartesian gpsOrigin(gpsMeasOrigin.latitude, gpsMeasOrigin.longitude, gpsMeasOrigin.altitude);
        double x,y,z;
        gpsOrigin.Forward(latitude, longitude, altitude, x, y, z);
        Eigen::Vector3d v(x,y,z);                        
        return v + noise; // TODO this sum should be done in a different method
    }
    catch(std::bad_cast exp)
    {
        std::cerr<<"Caught bad cast\n";
    }
    
}

Eigen::MatrixXd GpsMeasurement::getCovariance() const
{        
    return Eigen::MatrixXd(covariance);    
}

Eigen::Vector3d MocapMeasurement::getRelativeFromOrigin(const GlobalPosition& origin) const
{        
    try
    {
        const MocapMeasurement& mocapMeasOrigin = dynamic_cast<const MocapMeasurement&>(origin);
        // This is equivalent to take the translation part of T_origin.inverse() * T_meas
        Eigen::Vector3d v = mocapMeasOrigin.rotation.toRotationMatrix().transpose() * (position - mocapMeasOrigin.position);
        return v + noise;
    }
    catch(std::bad_cast exp)
    {
        std::cerr<<"Caught bad cast\n";
    }
    
}

Eigen::MatrixXd MocapMeasurement::getCovariance() const
{     
    return Eigen::MatrixXd(covariance);
}

}
}
