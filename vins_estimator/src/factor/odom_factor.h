#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>


// NORA Additional Odometry factor for the Ceres problem with a total distance measurement
class ODOMFactor  
{
    public:
        ODOMFactor(const double _d_measured) : d_measured(_d_measured) {}

        template <typename T>
        bool operator()(const T *const pos_i, const T *const pos_j, T* residuals) const
        {
            const Eigen::Matrix<T, 3, 1> Pi(pos_i[0], pos_i[1], pos_i[2]);
            const Eigen::Matrix<T, 3, 1> Pj(pos_j[0], pos_j[1], pos_j[2]);

            residuals[0] = ((Pj - Pi).norm() - static_cast<T>(d_measured))/static_cast<T>(0.00189);

            // cout << "Dist_calc: " << sqrt(pow(pos_i[0]-pos_j[0],2.0)+pow(pos_i[1]-pos_j[1],2.0)+pow(pos_i[2]-pos_j[2],2.0)) << endl
            //     << "Dist_calc_2: " << (Pi - Pj).norm() << endl
            //     << "Res: " << residuals[0] << endl << endl;
            return true;
        }

        static ceres::CostFunction *Create(const double _d_measured)
        {
            return new ceres::AutoDiffCostFunction<ODOMFactor, 1, 7, 7>(new ODOMFactor(_d_measured));
        }

    private:
        double d_measured;

};


// NORA Additional Odometry factor for the Ceres problem with distance in z direction and distance

class ODOMFactor_z  
{
    public:
        ODOMFactor_z(const double _dz) : dz(_dz) {}

        template <typename T>
        bool operator()(const T *const pos_i, const T *const pos_j, T* residuals) const
        {
            const Eigen::Matrix<T, 3, 1> Pi(pos_i[0], pos_i[1], pos_i[2]);
            const Eigen::Matrix<T, 3, 1> Pj(pos_j[0], pos_j[1], pos_j[2]);

            residuals[0] = (Pj[2]-Pi[2] - static_cast<T>(dz))/static_cast<T>(0.0006);

            // cout << "Calced_z: " << pos_j[2]-pos_i[2] << endl
            //     << "Measured_z: " << dz << endl
                // << "Calced_z_2: " << Pj[2]-Pi[2] << endl
                // << "Res: " << residuals[0] << endl;
            return true;
        }

        static ceres::CostFunction *Create(const double _dz)
        {
            return new ceres::AutoDiffCostFunction<ODOMFactor_z, 1, 7, 7>(new ODOMFactor_z(_dz));
        }

    private:
        double dz;

};
