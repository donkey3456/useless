#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <limits>
#include <list>
#include <set>

#define _USE_MATH_DEFINES
#include <math.h>

#include "LLogger.h"


namespace ekf
{
	
	// minimum stamp: -1
	// maximum stamp: 2147483647
	// maximum processing time(stamp starts from 0): 2147483648 / frequency;
	// when stamp equals -1
	// The user should handle the process order by himself.
	class LProcessObj: public LoggableObj
	{
	public:
		typedef std::shared_ptr<LProcessObj> LPtrProcessObj;
		struct ProcessOrderPrior
		{
			bool operator()(const LPtrProcessObj& lhs, const LPtrProcessObj& rhs)
			{
				if (lhs->_stamp == rhs->_stamp)
				{
					return lhs->ProcessOrder() < rhs->ProcessOrder();
				}
				else
				{
					return lhs->_stamp < rhs->_stamp;
				}
			}
		};
	public:
		LProcessObj(int stamp = -1) :_stamp(stamp) {};
		virtual ~LProcessObj() {};
		virtual void SetStamp(int stamp) 
		{
			assert(stamp >= -1);
			_stamp = stamp;
		}
		virtual int ProcessOrder() const = 0;

	protected:
		int _stamp;
	};

	template<int StateDimension>
	class LBaseObject: public LProcessObj
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, StateDimension, 1, Eigen::ColMajor> StateDataType;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateCovarianceType;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateJacobianType;
	public:
		struct GaussDistributionInfo
		{
			GaussDistributionInfo()
			{
				_valid = false; 
				_mean = StateDataType::Zero();
				_covariance = StateCovarianceType::Identity()*1e10;
			}

			bool _valid;
			StateDataType _mean;
			StateCovarianceType _covariance;
		};

	public:
		LBaseObject(int stamp = -1) :LProcessObj(stamp) {};
		virtual ~LBaseObject() {};

		virtual void Update(const GaussDistributionInfo& last_state, GaussDistributionInfo& state) = 0;
	};



	template<int StateDimension>
	class LStateRecorder : public LBaseObject<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		LStateRecorder(int stamp = -1) :
			LBaseObject<StateDimension>(stamp), _updated(false)
		{};
		virtual ~LStateRecorder() {};

		virtual int ProcessOrder() const { return 2; }
		virtual void SetOutdated() { _updated = false; };
		virtual void Update(const GaussDistributionInfo& last_state, GaussDistributionInfo& state)
		{
			state = _state = last_state;
			_updated = true;
		}

		bool Updated() { return _updated; }
		GaussDistributionInfo State() { return _state; }
		StateDataType Mean() { return _state._mean; }
		StateCovarianceType Covariance() { return _state._covariance; }
		bool IsValid() { return _state._valid; }
	protected:
		bool _updated;
		GaussDistributionInfo _state;
	};



	template<int StateDimension>
	class LControl : public LBaseObject<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		LControl(int stamp = -1) :
			LBaseObject<StateDimension>(stamp),_covariance(StateCovarianceType::Identity()){};
		virtual ~LControl() {};

		virtual int ProcessOrder() const { return 0; }
		virtual void SetCovariance(StateCovarianceType c) { _covariance = c; }
		virtual void Update(const GaussDistributionInfo& last_state, GaussDistributionInfo& state)
		{
			state._mean = PredictState(last_state._mean);
			StateJacobianType jacobian = Linearization(last_state._mean);
			state._covariance = jacobian * last_state._covariance * jacobian.transpose() + _covariance;
		}

	protected:
		virtual StateDataType PredictState(const StateDataType&) = 0;
		virtual StateJacobianType Linearization(const StateDataType& state)
		{
			StateJacobianType jacobian;
			StateDataType predicted_state = PredictState(state);
			for (int i = 0; i < StateDimension; i++)
			{
				StateDataType delta = StateDataType::Zero();
				delta(i) = 1e-6;
				StateDataType temp = state + delta;
				StateDataType predicted_state_new = PredictState(temp);
				jacobian.block(0, i, StateDimension, 1) = (predicted_state_new - predicted_state) * 1e6;
			}
			return jacobian;
		}

	protected:
		StateCovarianceType _covariance;
	};

	template<int StateDimension, int ControlDimension = StateDimension>
	class LBaseControl: public LControl<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, ControlDimension, 1, Eigen::ColMajor> ControlDataType;

	public:
		LBaseControl(int stamp = -1) :LControl<StateDimension>(stamp) {};
		virtual ~LBaseControl() {};

		virtual void SetControl(ControlDataType c) { _control = c; }

	protected:
		ControlDataType _control;
	};


	template<int StateDimension>
	class LMeasurement :public LBaseObject<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		LMeasurement(int stamp = -1) :LBaseObject<StateDimension>(stamp){}
		virtual ~LMeasurement() {};
		virtual int ProcessOrder() const { return 1; }
	};


	template<int MeasurementDimension,int StateDimension = MeasurementDimension>
	class LBaseMeasurement:public LMeasurement<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, MeasurementDimension, 1, Eigen::ColMajor> MeasurementDataType;
		typedef Eigen::Matrix<double, MeasurementDimension, MeasurementDimension, Eigen::ColMajor> MeasurementCovarianceType;
		typedef Eigen::Matrix<double, MeasurementDimension, StateDimension, Eigen::ColMajor> MeasurementJacobianType;
		typedef Eigen::Matrix<double, StateDimension, MeasurementDimension, Eigen::ColMajor> GainType;

	public:
		LBaseMeasurement(int stamp = -1): LMeasurement<StateDimension>(stamp){};
		virtual ~LBaseMeasurement() {};

		
		virtual void Update(const GaussDistributionInfo& last_state, GaussDistributionInfo& state)
		{
			MeasurementDataType residual = Residual(last_state._mean);
			MeasurementJacobianType jacobian = Linearization(last_state._mean);
			MeasurementCovarianceType residual_cov = jacobian * last_state._covariance * jacobian.transpose() + _covariance;
			GainType gain = last_state._covariance * jacobian.transpose() * residual_cov.inverse();
			state._mean = last_state._mean + gain * residual;
			state._covariance = last_state._covariance - gain * residual_cov * gain.transpose();
			std::cerr << gain*jacobian << std::endl;
			state._valid = true;
		}

		// check residual, maybe use this to remove outlier
		virtual MeasurementDataType Residual(StateDataType state)
		{
			return _measurement - MeasurementFuncion(state);
		}

		virtual void SetMeasurement(MeasurementDataType m) { _measurement = m; }
		virtual void setMeasurementCovariance(MeasurementCovarianceType c) { _covariance = c; }
	protected:
		virtual MeasurementDataType MeasurementFuncion(StateDataType) = 0;
		virtual MeasurementJacobianType Linearization(const StateDataType& state)
		{
			MeasurementJacobianType jacobian;
			MeasurementDataType measure = MeasurementFuncion(state);
			for (int i = 0; i < StateDimension; i++)
			{
				StateDataType delta = StateDataType::Zero();
				delta(i) = 1e-6;
				StateDataType temp = state + delta;
				MeasurementDataType measure_new = MeasurementFuncion(temp);
				jacobian.block(0, i, MeasurementDimension, 1) = (measure_new - measure) * 1e6;
			}
			return jacobian;
		}
	protected:
		MeasurementDataType _measurement;
		MeasurementCovarianceType _covariance;
	};

	template<int StateDimension>
	class LBaseEKF :public LoggableObj
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, StateDimension, 1, Eigen::ColMajor> StateDataType;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateCovarianceType;
		typedef std::shared_ptr<LBaseObject<StateDimension>> ProcessObjType;
		typedef std::shared_ptr<LControl<StateDimension>> ControlType;
		typedef std::shared_ptr<LStateRecorder<StateDimension>> StateRecorderType;
		typedef std::shared_ptr<LMeasurement<StateDimension>> MeasurementType;
	public:
		LBaseEKF() {}
		virtual ~LBaseEKF() {}


		virtual StateDataType CurrentEstimiate() = 0;
		virtual StateCovarianceType CurrentCovariance() = 0;
		virtual bool Valid() = 0;
		virtual void Update() = 0;
	protected:
		virtual void OutdateState() = 0;


	protected:


	};

	template<int StateDimension>
	class LSimpleEKF : public LBaseEKF<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	public:
		LSimpleEKF() :_state_recorder(new LStateRecorder<StateDimension>) {}
		virtual ~LSimpleEKF() {}

		virtual void Update()
		{
			auto state = _state_recorder->State();
			for (auto control : _controls)
			{
				auto last_state = state;
				control->Update(last_state, state);
			}

			for (auto measurement : _measurements)
			{
				auto last_state = state;
				measurement->Update(last_state, state);
			}

			auto last_state = state;
			_state_recorder->Update(last_state, state);
			_controls.clear();
			_measurements.clear();

		}

		void AddControl(ControlType control)
		{
			_controls.push_back(control);
			OutdateState();
		}
		void AddMeasurement(MeasurementType measure)
		{
			_measurements.push_back(measure);
			OutdateState();
		}

		virtual StateDataType CurrentEstimiate() { return _state_recorder->Mean(); };
		virtual StateCovarianceType CurrentCovariance() { return _state_recorder->Covariance(); }
		virtual bool Valid() { return _state_recorder->IsValid(); }
	protected:
		virtual void OutdateState() { _state_recorder->SetOutdated(); }

	protected:
		StateRecorderType _state_recorder;
		std::list<ControlType> _controls;
		std::list<MeasurementType> _measurements;
	};



	template<int StateDimension>
	class LBufferedEKF
	{

		typedef Eigen::Matrix<double, StateDimension, 1, Eigen::ColMajor> StateDataType;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateCovarianceType;
		typedef std::shared_ptr<LControl<StateDimension>> ControlType;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		LBufferedEKF() {};
		virtual ~LBufferedEKF() {};

		StateDataType GetMean() { return _mean; }
		StateCovarianceType GetCovariance() { return _covariance; }

		void AddControl(ControlType control);
		void AddMeasurement();


	protected:
		StateDataType _mean;
		StateCovarianceType _covariance;


	};







}