#include "State.h"
#include "Update.h"
#include "IngvioParams.h"
#include "StateManager.h"
#include "BaroMeas.h"
#include <iomanip>

namespace ingvio
{
    class State;
    class BaroUpdate : public UpdateBase
    {   
       public:
       using UpdateBase::UpdateBase;

       BaroUpdate(const IngvioParams& filter_params) :
        UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres)
        {
            _baro_noise_amp = 5.0;
        }

        virtual ~BaroUpdate() {}

        BaroUpdate(const BaroUpdate&) = delete;    

        BaroUpdate operator=(const BaroUpdate&) = delete;

        // void checkp0Status(std::shared_ptr<State> state, std::shared_ptr<BaroMeas> baro_meas)
        // {

        //     if (baro_meas->isInit() && state->_p0 == nullptr)
        //     {
        //         double p0;
        //         if(baro_meas->getInitPress(p0))
        //         { 
        //             p0 = 0.0;
        //             StateManager::addBaroVariable(state, p0, _p0_cov);

        //         }
        //     }
        // }


        void baroUpdate(std::shared_ptr<State> state, const double& baro_meas, double p0)
        {
            std::vector<std::shared_ptr<Type>> var_order;
            var_order.push_back(state->_extended_pose);
            int rows = 1;
            int cols = 9;
            double press = baro_meas;
            Eigen::Vector3d pos = state->_extended_pose->valueTrans1();
            Eigen::MatrixXd H(rows, cols);
            H.setZero();

            Eigen::VectorXd res(rows);
            res.setZero();
            //res(0) = press - p0 - pos(2);
            double h = baro_meas - p0;
            res(0) = h - pos(2);
            std::cout<<"baro res: "<<res(0)<<std::endl;
            //std::cout<<"h0: "<<state->_p0->value()<<std::endl;
            H(0,0) =  pos(1);
            H(0,1) = -1 * pos(0);
            H(0,5) = 1;
            //H(0,9) = 1;
            //var_order.push_back(state->_p0);
            Eigen::MatrixXd R(rows, rows);
            R.setZero();
            R(0, 0) = std::pow(_baro_noise_amp, 2.0);

            StateManager::ekfUpdate(state, var_order, H, res, R);


        }

        protected:

        double _p0_cov = 0.1;
        double _baro_noise_amp = 5.0;

    };
}