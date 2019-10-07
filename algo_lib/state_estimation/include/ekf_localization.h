#ifndef PR_IMPL_EKF_IMPLEMENTATION_H
#define PR_IMPL_EKF_IMPLEMENTATION_H

#include <Eigen/Dense>
#include <math.h>
#include <iostream>

#define TSTEP 0.01
#define a1 0
#define a2 0
#define a3 0
#define a4 0


void computeStateTransisitionJacobians(const Eigen::Vector3f mean_pose_tmin1, const Eigen::Vector2f control_input_t,
										 Eigen::Matrix3f &G_t_out, Eigen::Matrix3f &V_t_out);

void computeControlNoise(const Eigen::Vector2f control_input_t, Eigen::Matrix2f M_t_out);


void computePredictedMeanAndCovariance(const Eigen::Vector3f mean_pose_tmin1, const Eigen::Matrix3f covar_pose_tmin1, 
										const Eigen::Vector2f control_input_t, Eigen::Vector3f &pred_mean_pose_t_out, 
										Eigen::Matrix3f &pred_covar_pose_t_out);



#endif