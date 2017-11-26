#include "stdafx.h"
#include "LEKF2DSample.h"

namespace ekf
{

	LControl2D::LControl2D(int stamp /*= -1*/)
		:LBaseControl<3>(stamp) {}
	
	Eigen::Vector3d LControl2D::PredictState(const Eigen::Vector3d& last_state)
	{
		double last_theta = last_state(2);
		double cost = cos(last_theta);
		double sint = sin(last_theta);
		Eigen::Vector3d res;
		res(0) = _control(0) * cost - _control(1) * sint + last_state(0);
		res(1) = _control(0) * sint + _control(1) * cost + last_state(1);
		double theta = _control(2) + last_theta;
		Eigen::Rotation2D<double> r(theta);
		res(2) = r.smallestAngle();

		return res;
	};

	Eigen::Matrix3d LControl2D::Linearization(const Eigen::Vector3d& state)
	{
		return Eigen::Matrix3d::Identity();
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

}

