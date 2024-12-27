#pragma once

#include <queue>
#include <sensor_msgs/NavSatFix.h>
#include "IngvioParams.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace ingvio
{

    class GPSCtrl
    {
        public:
            GPSCtrl() : _e(0.0), _n(0.0), _u(0.0), _timestamp(0.0) {}
            GPSCtrl(sensor_msgs::NavSatFix::ConstPtr gpsfix_msg, double lat0, double lon0, double alt0){
                _timestamp = gpsfix_msg->header.stamp.toSec();
                // lla to enu
                double lat = gpsfix_msg->latitude;
                double lon = gpsfix_msg->longitude;
                double alt = gpsfix_msg->altitude;

                geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0, _e, _n, _u);

            }
            ~GPSCtrl() = default;

            void geodetic_to_enu(double lat, double lon, double h, double lat0, double lon0, double h0, double& xEast, double& yNorth, double& zUp) 
        {
        const double a = 6378137.0;
        const double b = 6356752.3142;
        const double f = (a - b) / a;
        const double e_sq = f * (2 - f);

        double lamb = lat * M_PI / 180.0;
        double phi = lon * M_PI / 180.0;
        double s = std::sin(lamb);
        double N = a / std::sqrt(1 - e_sq * s * s);

        double sin_lambda = std::sin(lamb);
        double cos_lambda = std::cos(lamb);
        double sin_phi = std::sin(phi);
        double cos_phi = std::cos(phi);

        double x = (h + N) * cos_lambda * cos_phi;
        double y = (h + N) * cos_lambda * sin_phi;
        double z = (h + (1 - e_sq) * N) * sin_lambda;

        double lamb0 = lat0 * M_PI / 180.0;
        double phi0 = lon0 * M_PI / 180.0;
        double s0 = std::sin(lamb0);
        double N0 = a / std::sqrt(1 - e_sq * s0 * s0);

        double sin_lambda0 = std::sin(lamb0);
        double cos_lambda0 = std::cos(lamb0);
        double sin_phi0 = std::sin(phi0);
        double cos_phi0 = std::cos(phi0);

        double x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
        double y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
        double z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;

        double xd = x - x0;
        double yd = y - y0;
        double zd = z - z0;

        double t = -cos_phi0 * xd - sin_phi0 * yd;

        xEast = -sin_phi0 * xd + cos_phi0 * yd;
        yNorth = t * sin_lambda0 + cos_lambda0 * zd;
        zUp = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;
    };
            double _e, _n, _u;
            double _timestamp;

    };

    class GPSFixMeas
    {
        public :
            GPSFixMeas() : _has_origin(false), _lat0(0.0), _lon0(0.0), _alt0(0.0), _max_buffer_size(100),_yaw(0.0), _is_aligned(false) , _align_buffer_size(40), _align_dist_thres(40) {}
            ~GPSFixMeas() = default;

            void inputGPSFix(sensor_msgs::NavSatFix::ConstPtr gpsfix_msg)
            {
                if (!_has_origin)
                {
                    _lat0 = gpsfix_msg->latitude;
                    _lon0 = gpsfix_msg->longitude;
                    _alt0 = gpsfix_msg->altitude;
                    _has_origin = true;
                }
                GPSCtrl gpsfix(gpsfix_msg, _lat0, _lon0, _alt0);
                if (_gpsfix_buffer.size() < _max_buffer_size)
                    _gpsfix_buffer.push(gpsfix);
                else
                {
                    _gpsfix_buffer.pop();
                    _gpsfix_buffer.push(gpsfix);
                    std::cout << "[GPSFixMeas]: GPSFix buffer is full, pop one." << std::endl;
                }
            }

            bool getENUat(double timestamp, Eigen::Vector3d& enu)
            {
                bool flag = false;
                while (!_gpsfix_buffer.empty())
                {
                    auto& gpsfix = _gpsfix_buffer.front();
                    if (gpsfix._timestamp < timestamp - 0.13)
                    {
                        _gpsfix_buffer.pop();
                        continue;
                    }
                    if (gpsfix._timestamp > timestamp + 0.13)
                        break;
                    enu << gpsfix._e, gpsfix._n, gpsfix._u;
                    _gpsfix_buffer.pop();
                    flag = true;
                    break;
                }
                return flag;
            }

            bool isAligned() const
            {
                return _is_aligned;
            }

            void batchAlign(const Eigen::Vector3d& enu, const Eigen::Vector3d& xyz)
            {
                if(_is_aligned)
                    return;
                if(_xyz.size() < _align_buffer_size)
                {
                    _xyz.push_back(xyz);
                    _enu.push_back(enu);
                }
                else
                {
                    _xyz.erase(_xyz.begin());
                    _enu.erase(_enu.begin());
                    _xyz.push_back(xyz);
                    _enu.push_back(enu);

                    std::cout << "[GPSFixMeas]: start align." << std::endl;

                    // xy dist from begin to end
                    double dist = 0.0;
                    Eigen::Vector3d xyz_diff = _xyz[_align_buffer_size - 1] - _xyz[0];
                    dist = xyz_diff.norm();
                    if(dist < _align_dist_thres)
                    {
                        std::cout << "[GPSFixMeas]: GPSFix align distance is too small, waiting and restart ..." << std::endl;
                        std::cout << "[GPSFixMeas]: dist: " << dist << std::endl;
                        // clear
                        _xyz.clear();
                        _enu.clear();
                        _is_aligned = false;
                        return;
                    }

                    //compute translation from enu to xyz
                    double x0 = 0.0, y0 = 0.0;
                    double e0 = 0.0, n0 = 0.0;
                    x0 = _xyz[0].x();
                    y0 = _xyz[0].y();
                    e0 = _enu[0].x();
                    n0 = _enu[0].y();
                    for(int i = 0; i < _align_buffer_size; i++)
                    {
                        _enu[i].x() -= e0;
                        _enu[i].y() -= n0;
                        _xyz[i].x() -= x0;
                        _xyz[i].y() -= y0;
                    }


                    double nume = 0.0, deno = 0.0;
                    for(int i = 0; i < _align_buffer_size; i++)
                    {
                        nume += _xyz[i].x() * _enu[i].y() - _xyz[i].y() * _enu[i].x();
                        deno += _xyz[i].x() * _enu[i].x() + _xyz[i].y() * _enu[i].y();
                    }
                    double yaw = std::atan2(nume, deno);
                    _yaw = yaw;

                    // ROTATE en to xy using yaw
                    for(int i = 0; i < _align_buffer_size; i++)
                    {
                        _enu[i].x() += e0;
                        _enu[i].y() += n0;
                        double x = _enu[i].x() * std::cos(yaw) + _enu[i].y() * std::sin(yaw);
                        double y = -1.0 * _enu[i].x() * std::sin(yaw) + _enu[i].y() * std::cos(yaw);
                        _enu[i].x() = x;
                        _enu[i].y() = y;
                    }
                    //compute anchor point
                    double x = 0.0, y = 0.0, z = 0.0;
                    for(int i = 0; i < _align_buffer_size; i++)
                    {
                        _xyz[i].x() += x0;
                        _xyz[i].y() += y0;
                        x += _xyz[i].x() - _enu[i].x();
                        y += _xyz[i].y() - _enu[i].y();
                        z += _xyz[i].z() - _enu[i].z();
                    }
                    x /= _align_buffer_size;
                    y /= _align_buffer_size;
                    z /= _align_buffer_size;
                    //compute align rmse
                    double rmse = 0.0;
                    for(int i = 0; i < _align_buffer_size; i++)
                    {
                        // xyz rmse
                        rmse += std::pow(_xyz[i].z() - (_enu[i].z() + z), 2);
                        rmse += std::pow(_xyz[i].x() - (_enu[i].x() + x), 2);
                        rmse += std::pow(_xyz[i].y() - (_enu[i].y() + y), 2);
                    }
                    rmse = std::sqrt(rmse / _align_buffer_size);
                    std::cout << "[GPSFixMeas]: GPSFix align rmse: " << rmse << std::endl;
                    _is_aligned = false;
                    _xyz.clear();
                    _enu.clear();
                    std::cout << "[GPSFixMeas]: GPSFix align finished." << std::endl;
                    std::cout << "[GPSFixMeas]: yaw: " << yaw << std::endl;
                    // dist
                    std::cout << "[GPSFixMeas]: dist: " << dist << std::endl;
                    if(rmse < 0.3)
                    {
                        _is_aligned = true;
                        _yaw = yaw;
                        _x0 = x;
                        _y0 = y;
                        _z0 = z;
                    }
                }
            }

            bool getAlignYawxyz(double& yaw, double& x, double& y, double& z)
            {
                if(_is_aligned)
                 {
                    yaw = _yaw;
                    x = _x0;
                    y = _y0;
                    z = _z0;
                    return true;
                }
                return false;
            }
        protected:
            std::queue<GPSCtrl> _gpsfix_buffer;

            bool _has_origin;

            double _lat0, _lon0, _alt0;
            int _max_buffer_size;
            int _align_buffer_size;

            double _yaw;
            double _x0, _y0, _z0;
            bool _is_aligned;

            double _align_dist_thres;
            // xy and en vector
            std::vector<Eigen::Vector3d> _xyz;
            std::vector<Eigen::Vector3d> _enu;
    };
}