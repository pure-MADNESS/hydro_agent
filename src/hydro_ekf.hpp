/*
  _   _           _             _____ _  _______    ____ _               
 | | | |_   _  __| |_ __ ___   | ____| |/ /  ___|  / ___| | __ _ ___ ___ 
 | |_| | | | |/ _` | '__/ _ \  |  _| | ' /| |_    | |   | |/ _` / __/ __|
 |  _  | |_| | (_| | | | (_) | | |___| . \|  _|   | |___| | (_| \__ \__ \
 |_| |_|\__, |\__,_|_|  \___/  |_____|_|\_\_|      \____|_|\__,_|___/___/
        |___/                                                            
*/


#ifndef HYDRO_EKF_HPP
#define HYDRO_EKF_HPP   

#include "ekf.hpp"
#include <cmath>
#include <algorithm>

using namespace Eigen;
using namespace std;


class HydroEKF : public EKF {

    public:

        /**
         * @brief State: [omega, P_max]^T
         *        Measurement: [omega_alternatore]^T
         */
        HydroEKF(double J, double kt, int pairs);

        void set_inputs(double  Q_water_api, double p_now);

        /**         
         * Prediction:                  
         * J * dw/dt = Tau_hydro - Tau_load                      
         */
        VectorXd f(const VectorXd& x, double dt) override;

        MatrixXd F(const VectorXd& x, double dt) override;        

        VectorXd h(const VectorXd& x_pred) override;

        MatrixXd H(const VectorXd& x) override;

     private:

        double _J;          // [kg*m^2]
        double _kt;         // fixed
        double _p_pairs;
        double _Q_water;    // [m^3/s]
        double _p_actual;   // [W]
};


#endif
