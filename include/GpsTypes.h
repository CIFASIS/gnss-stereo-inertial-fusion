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

#ifndef GPSTYPES_H
#define GPSTYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

namespace GlobalPosition
{

class GlobalPosition
{
    protected:
        GlobalPosition(){}
        GlobalPosition(double id, double timestamp): id(id), timestamp(timestamp){}

    public:
        virtual ~GlobalPosition(){}
        virtual Eigen::Vector3d getRelativeFromOrigin(const GlobalPosition& origin) const = 0;
        virtual Eigen::MatrixXd getCovariance() const = 0;
        double timestamp;
        int id;

};


class GpsMeasurement : public GlobalPosition
{
public:
    GpsMeasurement(){}
    GpsMeasurement(const int id, const double &latitude, const double &longitude, const double &altitude, const double &timestamp, const Eigen::Matrix3d &covariance, const Eigen::Vector3d noise = Eigen::Vector3d(0.,0.,0.)):
        latitude(latitude), longitude(longitude), altitude(altitude), noise(noise), covariance(covariance), GlobalPosition(id, timestamp){}
        
    double latitude;
    double longitude;
    double altitude;    
    Eigen::Vector3d noise;
    Eigen::Matrix3d covariance;

    Eigen::Vector3d getRelativeFromOrigin(const GlobalPosition& origin) const override;
    Eigen::MatrixXd getCovariance() const override;

};

class MocapMeasurement : public GlobalPosition
{
public:
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW: http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MocapMeasurement(){}
    MocapMeasurement(
        const int id, 
        const double &x, 
        const double &y, 
        const double &z, 
        const double &qx,  
        const double &qy,  
        const double &qz,  
        const double &qw, 
        const double &timestamp,
        const Eigen::Matrix3d &covariance,
        const Eigen::Vector3d noise = Eigen::Vector3d(0.,0.,0.)):
        noise(noise), 
        covariance(covariance),
        GlobalPosition(id, timestamp)
        {
            position = Eigen::Vector3d(x,y,z);
            rotation = Eigen::Quaterniond(qw,qx,qy,qz);
        }
        
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d noise;
    Eigen::Matrix3d covariance;
    Eigen::Vector3d getRelativeFromOrigin(const GlobalPosition& origin) const override;
    Eigen::MatrixXd getCovariance() const override;

};

class Calib
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::make_array(tbg.data(), tbg.size());

        ar & mbIsSet;
    }

public:

    Calib(const Eigen::Vector3f &tbg) : tbg(tbg), mbIsSet(true)
    {}
    Calib() : mbIsSet(false)
    {}

public:
    // Sophus/Eigen implementation
    Eigen::Vector3f tbg;
    bool mbIsSet;

};

}

}

#endif
