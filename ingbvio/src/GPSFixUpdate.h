#include "State.h"
#include "Update.h"
#include "IngvioParams.h"
#include "StateManager.h"
#include "GPSFixMeas.h"
#include <iomanip>

namespace ingvio{
    class state;
    class GPSFixUpdate : public UpdateBase
    {
        public: 
        using UpdateBase::UpdateBase;
        GPSFixUpdate(const IngvioParams& params) : UpdateBase(params._chi2_max_dof, params._chi2_thres)
        {
            gps_xy_cov = 5.0;
            gps_z_cov = 10.0;
        }
        virtual ~GPSFixUpdate(){}
        GPSFixUpdate(const GPSFixUpdate&) = delete;
        GPSFixUpdate operator=(const GPSFixUpdate&) = delete;

        void gpsFixUpdate(std::shared_ptr<State> state, const Eigen::Vector3d& gps_meas)
        {
            std::vector<std::shared_ptr<Type>> var_order;
            var_order.push_back(state->_extended_pose);
            int rows = 3;
            int cols = 9;
            Eigen::Vector3d pos = state->_extended_pose->valueTrans1();
            Eigen::MatrixXd H(rows, cols);
            H.setZero();

            Eigen::VectorXd res(rows);
            res.setZero();
            res(0) = gps_meas(0) - pos(0);
            res(1) = gps_meas(1) - pos(1);
            res(2) = gps_meas(2) - pos(2);

            std::cout << "GPS res: " << res.transpose() << std::endl;

            H(0, 1) = pos(2);
            H(0, 2) = -pos(1);
            H(1, 0) = -pos(2);
            H(1, 2) = pos(0);
            H(2, 0) = pos(1);
            H(2, 1) = -pos(0);

            H(0, 3) = 1;
            H(1, 4) = 1;
            H(2, 5) = 1;


            Eigen::MatrixXd R(rows, rows);
            R.setZero();
            R(0, 0) = std::pow(gps_xy_cov, 2.0);
            R(1, 1) = std::pow(gps_xy_cov, 2.0);
            R(2, 2) = std::pow(gps_z_cov, 2.0);

            StateManager::ekfUpdate(state, var_order, H, res, R);
        }



        protected:
        double gps_xy_cov;
        double gps_z_cov;
    };
}