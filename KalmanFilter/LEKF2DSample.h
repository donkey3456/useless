#pragma once

#include "LEKFBase.h"
// The user has to write the funciton PredictState.
// The user can alse rewrite Linearization to provide theoretical jacobian
namespace ekf
{
	class LControl2D : public LBaseControl<3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		LControl2D(int stamp = -1);
		virtual ~LControl2D() {};
	protected:
		virtual Eigen::Vector3d PredictState(const Eigen::Vector3d& last_state);
		virtual Eigen::Matrix3d Linearization(const Eigen::Vector3d& state);
	};


	class LMeasurement2D : public LBaseMeasurement<3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		LMeasurement2D(int stamp = -1);
		virtual ~LMeasurement2D() {};
	protected:
		virtual Eigen::Vector3d MeasurementFuncion(Eigen::Vector3d);
		virtual Eigen::Matrix3d Linearization(const Eigen::Vector3d& state);
	};

}
