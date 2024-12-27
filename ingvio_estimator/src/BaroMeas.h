// #pragma once
// #include <sensor_msgs/FluidPressure.h>
// #include <queue>
// #include "State.h"
// #include "Update.h"
// #include "StateManager.h"
// namespace ingvio
// {

//     class BaroCtrl
//     {
//     public:
//         BaroCtrl() : _pressure(0.0), _timestamp(0.0)
//         {}
        
//         BaroCtrl(const sensor_msgs::FluidPressure::ConstPtr& baro_msg)
//         {
//             _pressure = baro_msg->fluid_pressure;
//             _timestamp = baro_msg->header.stamp.toSec() ;
//             //_temperature = baro_msg->temperature;
//         }
        
//         ~BaroCtrl() = default;
        
//         BaroCtrl operator+(const BaroCtrl& other_baro)
//         {
//             BaroCtrl result;
            
//             result._pressure = this->_pressure + other_baro._pressure;
//             result._timestamp = this->_timestamp + other_baro._timestamp;
            
//             return result;
//         }

//         BaroCtrl& operator=(const BaroCtrl& other_baro)
//         {
//             this->_pressure = other_baro._pressure;
//             this->_timestamp = other_baro._timestamp;
            
//             return *this;
//         }

//         double _pressure;
//         double _timestamp;
//     };

//     class BaroMeas
//     {
//     public:

//         // 构造函数
//         //使用 filter_params
//         BaroMeas(const IngvioParams& filter_params) : _has_baro_set(false), _p0(0.0), _max_baro_buffer_size(filter_params._max_baro_buffer_size)
//         {
//             time_threshold = 0.01;
//         }
//         const bool& isInit() const
//         { return _has_baro_set; }

//         bool getInitPress(double& p0) const
//         { 
//             if (!_has_baro_set)
//             {
//                 p0 = 0;
//                 return false;
//             }
//             else
//             {
//                 p0 = _p0;
//                 return true;
//             }
//         }
//         void storeBaro(const sensor_msgs::FluidPressure::ConstPtr& baro_msg)
//         {
//             BaroCtrl tmp(baro_msg);
//             this->storeBaro(tmp);
//         }

//         void storeBaro(const BaroCtrl& baro_ctrl)
//         {
//             if (_baro_ctrl_buffer.size() > _max_baro_buffer_size)
//         {
//             std::cout << "[BaroMeas]: Exceeding baro max buffer size, throw curr imu ctrl!" << std::endl;
//             return;
//         }
//         else
//             _baro_ctrl_buffer.push(baro_ctrl);
//         }

//         bool baro_init(double t0, double t1)
//         {

//             int cnt = 0;
//             _p0 = 0;
//             while(!_baro_ctrl_buffer.empty())
//             {
//                 double t = _baro_ctrl_buffer.front()._timestamp;
//                 if (t < t0)
//                     {_baro_ctrl_buffer.pop();
//                     continue;}
//                 if (t >= t0 && t <= t1)
//                 {
//                     _p0 += _baro_ctrl_buffer.front()._pressure;
//                     cnt++;
//                     _baro_ctrl_buffer.pop();
//                 }
//                 else
//                 {
//                     break;
//                 }
 
//             }
//             // for(auto& baro_ctrl : _baro_ctrl_buffer)
//             // {
//             //     if (baro_ctrl._timestamp >= t0 && baro_ctrl._timestamp <= t1)
//             //     {
//             //         _p0 += baro_ctrl._pressure;
//             //         cnt++;
//             //     }
//             // }
//             if (cnt > 0)
//             {
//                 _p0 /= (double) cnt;
//                 _has_baro_set = true;
//                 std::cout << "[BaroMeas]: init baro! with cnt = " << cnt <<" with p0 = "<< _p0<<std::endl;
//                 return true;
//             }
//             else
//             {
//                 std::cout << "[BaroMeas]: No baro data in the time interval!" << std::endl;
//                 return false;
//             }
//         }

//         bool getBaroMeasAt(const double& timestamp, double& pressure) 
//         {
//             bool flag = false;
//             if (!_has_baro_set)
//             {
//                 pressure = 0;
//                 return flag;
//             }
//             while(!_baro_ctrl_buffer.empty())
//             {
//                 BaroCtrl& baro_ctrl = _baro_ctrl_buffer.front();
//                 if (baro_ctrl._timestamp < timestamp - time_threshold)
//                 {
//                     _baro_ctrl_buffer.pop();
//                     continue;
//                 }
//                 if(baro_ctrl._timestamp >= timestamp + time_threshold)
//                 {
//                     break;
//                 }
//                 flag = true;
//                 pressure = baro_ctrl._pressure;
//                  _baro_ctrl_buffer.pop();
//                 // std::cout<<"baro buffer size now"<<_baro_ctrl_buffer.size()<<std::endl;
//                 break;
                
//             }
//             return flag;
//         }

//     protected:
//         std::queue<BaroCtrl> _baro_ctrl_buffer;
//         bool _has_baro_set;
//         double _p0;
//         int _max_baro_buffer_size;
//         double time_threshold;
//     };
// }