
#include "tcp_client.h"
#include "ekf_localization.h"

void computeStateTransisitionJacobians(const Eigen::Vector3f mean_pose_tmin1, const Eigen::Vector2f control_input_t,
										 Eigen::Matrix3f &G_t_out, Eigen::MatrixXf &V_t_out) {

	float vt = control_input_t(0);
	float wt = control_input_t(1);
	float x_tmin1 = mean_pose_tmin1(0);
	float y_tmin1 = mean_pose_tmin1(1);
	float theta_tmin1 = mean_pose_tmin1(2);

	G_t_out(0, 0) = 1;
	G_t_out(1, 0) = 0;
	G_t_out(2, 0) = 0;

	G_t_out(0, 1) = 0;
	G_t_out(1, 1) = 1;
	G_t_out(2, 1) = 0;

	G_t_out(0, 2) = -(vt/wt)*math::cos(theta_tmin1) + (vt/wt)*math::cos(theta_tmin1 + wt*TSTEP);
	G_t_out(1, 2) = -(vt/wt)*math::sin(theta_tmin1) + (vt/wt)*math::sin(theta_tmin1 + wt*TSTEP);
	G_t_out(2, 2) = 1;

	V_t_out(0,0) = (-sin(theta_tmin1)+sin(theta_tmin1 + wt*TSTEP)) / wt;
	V_t_out(1,0) = (cos(theta_tmin1)-cos(theta_tmin1 + wt*TSTEP)) / wt;
	V_t_out(2,0) = 0;

	V_t_out(0,1) = (vt*(sin(theta_tmin1) - sin(theta_tmin1 + wt*TSTEP)) / (math::pow(wt, 2))) + 
					(vt*(cos(theta_tmin1 + wt*TSTEP)*TSTEP) / (math::pow(wt, 2)));
	V_t_out(1,1) = (-vt*(cos(theta_tmin1) - cos(theta_tmin1 + wt*TSTEP)) / (math::pow(wt, 2))) + 
					(vt*(sin(theta_tmin1 + wt*TSTEP)*TSTEP) / (math::pow(wt, 2)));;
	V_t_out(2,1) = TSTEP;

}

void computeControlNoise(const Eigen::Vector2f control_input_t, Eigen::Matrix2f M_t_out) {

	float vt = control_input_t(0);
	float wt = control_input_t(1);
	
	M_t_out(0,0) = a1*math::pow(vt, 2) + a2*math::pow(wt, 2);
	M_t_out(1,0) = 0;

	M_t_out(0,1) = 0;
	M_t_out(1,1) = a3*math::pow(vt, 2) + a4*math::pow(wt, 2);

}



void computePredictedMeanAndCovariance(const Eigen::Vector3f mean_pose_tmin1, const Eigen::Matrix3f covar_pose_tmin1, 
										const Eigen::Vector2f control_input_t, Eigen::Vector3f &pred_mean_pose_t_out, 
										Eigen::Matrix3f &pred_covar_pose_t_out) {

	Eigen::Matrix3f G_t;
	Eigen:MatrixXf V_t(3,2);
	Eigen::Matrix2f M_t;

	computeStateTransisitionJacobians(mean_pose_tmin1, control_input_t, G_t, V_t);
	computeControlNoise(control_input_t, M_t);

	//Compute Predicted Mean

	pred_mean_pose_t_out(0) = mean_pose_tmin1(0) + (-(vt/wt)*math::sin(theta_tmin1) + (vt/wt)*math::sin(theta_tmin1 + wt*TSTEP));
	pred_mean_pose_t_out(1) = mean_pose_tmin1(1) + ((vt/wt)*math::cos(theta_tmin1) - (vt/wt)*math::cos(theta_tmin1 + wt*TSTEP);) 
	pred_mean_pose_t_out(2) = mean_pose_tmin1(2) + control_input_t(1)*TSTEP;

	//Compute Predicted Covariance

	pred_covar_pose_t_out = G_t*covar_pose_tmin1*G_t.transpose() + V_t*M_t*V_t.transpose();

}





int main() {
	int clientfd = 0;
	init_tcp_client(clientfd);

	char data[24] = "adfasdf";

	send_data(clientfd, data);

	close_tcp_client(clientfd);
}