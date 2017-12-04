#include "stdafx.h"
#include "LEKF2DSample.h"

namespace ekf
{

	LControl2D::LControl2D(int stamp /*= -1*/)
		:LBaseControl<3>(stamp) {}
	
	Eigen::Vector3d LControl2D::PredictStateImpl(const Eigen::Vector3d& last_state, const Eigen::Vector3d& control)
	{
		double last_theta = last_state(2);
		double cost = cos(last_theta);
		double sint = sin(last_theta);
		Eigen::Vector3d res;
		res(0) = control(0) * cost - control(1) * sint + last_state(0);
		res(1) = control(0) * sint + control(1) * cost + last_state(1);
		res(2) = control(2) + last_theta;
		//double theta = control(2) + last_theta;
		//Eigen::Rotation2D<double> r(theta);
		//res(2) = r.smallestAngle();

		return res;
	};


	Eigen::Matrix3d LControl2D::Fx(const Eigen::Vector3d& state)
	{

		Eigen::Matrix3d res = Eigen::Matrix3d::Identity();
		double cost = cos(state(2));
		double sint = sin(state(2));
		double xu = _control(0);
		double yu = _control(1);
		res(0, 2) = -xu * sint - yu * cost;
		res(1, 2) = xu * cost - yu *sint;

		return res;
	}

	LMeasurement2D::LMeasurement2D(int stamp /*= -1*/)
		:LBaseMeasurement<3>(stamp) {}

	Eigen::Vector3d LMeasurement2D::MeasurementFuncion(Eigen::Vector3d state)
	{
		return state;
	}

	Eigen::Matrix3d LMeasurement2D::Linearization(const Eigen::Vector3d & state)
	{

		return Eigen::Matrix3d::Identity();
	}

	Eigen::Vector2d LMeasurementAruco::MeasurementFuncion(Eigen::Vector3d state)
	{
		double cost = cos(state(2));
		double sint = sin(state(2));
// 		double xm = _p_w_m(0);
// 		double ym = _p_w_m(1);
		double x = state(0);
		double y = state(1);
		Eigen::Vector2d res;
		res(0) = xm * cost + ym *sint - x *cost - y*sint;
		res(1) = - xm * sint + ym *cost + x *sint - y*cost;
		return res;
	}

}

