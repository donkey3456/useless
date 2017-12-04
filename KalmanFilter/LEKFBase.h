#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <limits>
#include <list>
#include <queue>
#include <set>
#include <mutex>

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
		virtual bool LessPriorThan (const LPtrProcessObj& rhs) const
		{
			if (_stamp == rhs->_stamp)
			{
				return ProcessOrder() > rhs->ProcessOrder();
			}
			else
			{
				return _stamp > rhs->_stamp;
			}
		}
	public:
		LProcessObj(int stamp = -1) :_stamp(stamp) {};
		virtual ~LProcessObj() {};
		virtual void SetStamp(int stamp) 
		{
			assert(stamp >= -1);
			_stamp = stamp;
		}
		int Stamp() { return _stamp; }
	protected:
		virtual int ProcessOrder() const = 0; // overloaded in LControl & LMeasurement
		int _stamp;
	};


	template<int StateDimension>
	class LBaseObject: public LProcessObj
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, StateDimension, 1, Eigen::ColMajor> StateVector;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateCovariance;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateJacobian;
	public:
		struct GaussDistribution
		{
			GaussDistribution()
			{
				_mean = StateVector::Zero();
				_covariance = StateCovariance::Identity()*1e10;
			}

			StateVector _mean;
			StateCovariance _covariance;
		};

	public:
		LBaseObject(int stamp = -1) :LProcessObj(stamp),_updated(false) {};
		virtual ~LBaseObject() {};

		void Update(const GaussDistribution& last_state, GaussDistribution& state)
		{
			_pre_state = last_state;
			UpdateImpl();
			state = _post_state;
			_updated = true;
		};		
		bool Updated() { return _updated; }
		GaussDistribution GetPostState() { return _post_state; }

	protected:
		virtual void UpdateImpl() = 0; // overloaded in LControl & LBaseMeasurement

	protected:
		GaussDistribution _pre_state;
		GaussDistribution _post_state;
		bool _updated;
	};

	template<int StateDimension>
	class LStateRecorder :public LBaseObject<StateDimension>
	{
	public:
		LStateRecorder(int stamp = -1):LBaseObject<StateDimension>(stamp) {}

	protected:
		virtual void UpdateImpl() { _post_state = _pre_state; }
		virtual int ProcessOrder() const { return 0; }

	};

	template<int StateDimension>
	class LControl : public LBaseObject<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		LControl(int stamp = -1) :
			LBaseObject<StateDimension>(stamp){};
		virtual ~LControl() {};

	protected:
		virtual int ProcessOrder() const { return 0; }
		virtual void UpdateImpl()
		{
			_post_state._mean = PredictState(_pre_state._mean);
			StateJacobian jacobian = Fx(_pre_state._mean);
			StateCovariance covariance = CalControlCovariance(_pre_state._mean);
			_post_state._covariance = jacobian * _pre_state._covariance * jacobian.transpose() + covariance;
		}
		virtual StateCovariance CalControlCovariance(const StateVector&) = 0; // overloaded in LBaseControl 
		virtual StateVector PredictState(const StateVector&) = 0; // overloaded in LBaseControl 
		virtual StateJacobian Fx(const StateVector& state)
		{
			StateJacobian jacobian;
			StateVector predicted_state = PredictState(state);
			for (int i = 0; i < StateDimension; i++)
			{
				StateVector delta = StateVector::Zero();
				delta(i) = 1e-6;
				StateVector temp = state + delta;
				StateVector predicted_state_new = PredictState(temp);
				jacobian.block(0, i, StateDimension, 1) = (predicted_state_new - predicted_state) * 1e6;
			}
			return jacobian;
		}
	};


	template<int StateDimension, int ControlDimension = StateDimension>
	class LBaseControl: public LControl<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, ControlDimension, 1, Eigen::ColMajor> ControlVector;
		typedef Eigen::Matrix<double, ControlDimension, ControlDimension, Eigen::ColMajor> ControlCovarance;
		typedef Eigen::Matrix<double, StateDimension, ControlDimension, Eigen::ColMajor> ControlJacobian;
	public:
		LBaseControl(int stamp = -1) :LControl<StateDimension>(stamp) {};
		virtual ~LBaseControl() {};

		virtual void SetControl(const ControlVector& c) { _control = c; }
		virtual void SetControlCovariance(const ControlCovarance& c) { _covariance = c; }

	protected:
		virtual StateVector PredictState(const StateVector& state)
		{
			return PredictStateImpl(state, _control);
		}
		virtual StateVector PredictStateImpl(const StateVector& state, const ControlVector& control) = 0; // overloaded by user
		virtual StateCovariance CalControlCovariance(const StateVector& state) 
		{
			ControlJacobian jacobian = Fu(state);
			return jacobian * _covariance * jacobian.transpose();
		}
		virtual ControlJacobian Fu(const StateVector& state)
		{
			ControlCovarance jacobian;
			StateVector predicted_state = PredictState(state);
			for (int i = 0; i < ControlDimension; i++)
			{
				ControlVector delta = ControlVector::Zero();
				delta(i) = 1e-6;
				ControlVector temp = _control + delta;
				StateVector predicted_state_new = PredictStateImpl(state,temp);
				jacobian.block(0, i, StateDimension, 1) = (predicted_state_new - predicted_state) * 1e6;
			}
			return jacobian;
		}
	protected:
		ControlVector _control;
		ControlCovarance _covariance;
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


	template<int StateDimension,int MeasurementDimension = StateDimension>
	class LBaseMeasurement:public LMeasurement<StateDimension>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, MeasurementDimension, 1, Eigen::ColMajor> MeasurementVector;
		typedef Eigen::Matrix<double, MeasurementDimension, MeasurementDimension, Eigen::ColMajor> MeasurementCovariance;
		typedef Eigen::Matrix<double, MeasurementDimension, StateDimension, Eigen::ColMajor> MeasurementJacobian;
		typedef Eigen::Matrix<double, StateDimension, MeasurementDimension, Eigen::ColMajor> Gain;

	public:
		LBaseMeasurement(int stamp = -1): LMeasurement<StateDimension>(stamp){};
		virtual ~LBaseMeasurement() {};

		virtual MeasurementVector Residual(StateVector state)
		{
			return _measurement - MeasurementFuncion(state);
		}
		virtual void SetMeasurement(MeasurementVector m) { _measurement = m; }
		virtual void setMeasurementCovariance(MeasurementCovariance c) { _covariance = c; }

	protected:
		virtual void UpdateImpl()
		{
			MeasurementJacobian jacobian = Linearization(_pre_state._mean);
			MeasurementVector residual = Residual(_pre_state._mean);
			MeasurementCovariance residual_cov = jacobian * _pre_state._covariance * jacobian.transpose() + _covariance;
			Gain gain = _pre_state._covariance * jacobian.transpose() * residual_cov.inverse();
			_post_state._mean = _pre_state._mean + gain * residual;
			//_logger.Err() << (jacobian.transpose() * jacobian).inverse();
			//_post_state._covariance = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose() * _covariance * residual_cov.inverse() * jacobian * _pre_state._covariance;
			_post_state._covariance = (StateCovariance::Identity() - gain * jacobian) * _pre_state._covariance;
		}

		// check residual, maybe use this to remove outlier
		virtual MeasurementVector MeasurementFuncion(StateVector) = 0; // overloaded by user
		virtual MeasurementJacobian Linearization(const StateVector& state)
		{
			MeasurementJacobian jacobian;
			MeasurementVector measure = MeasurementFuncion(state);
			for (int i = 0; i < StateDimension; i++)
			{
				StateVector delta = StateVector::Zero();
				delta(i) = 1e-6;
				StateVector temp = state + delta;
				MeasurementVector measure_new = MeasurementFuncion(temp);
				jacobian.block(0, i, MeasurementDimension, 1) = (measure_new - measure) * 1e6;
			}
			return jacobian;
		}
	protected:
		MeasurementVector _measurement;
		MeasurementCovariance _covariance;
	};

	template<int StateDimension>
	class LEKF :public LoggableObj
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		typedef Eigen::Matrix<double, StateDimension, 1, Eigen::ColMajor> StateVector;
		typedef Eigen::Matrix<double, StateDimension, StateDimension, Eigen::ColMajor> StateCovariance;
		typedef std::shared_ptr<LBaseObject<StateDimension>> ProcessObj;
		typedef std::shared_ptr<LBaseControl<StateDimension>> Control;
		typedef std::shared_ptr<LMeasurement<StateDimension>> Measurement;
	public:
		LEKF() 
		{
			_buffersize = 100;
			auto pobj = make_shared<LStateRecorder<StateDimension>>();
			LStateRecorder<StateDimension>::GaussDistribution state;
			pobj->Update(LStateRecorder<StateDimension>::GaussDistribution(), state);
			_processobjs.push_back(pobj);
			_updated = true;
		}

		virtual ~LEKF() {}

		void SetControlBufferSize(int size) 
		{ 
			if (size<=0)
			{
				_logger.Err() << "Buffer size should larger than 0!!!" << endl;
				return;
			}
			_buffersize = size;
		}

		virtual void AddControl(Control control)
		{
			std::lock_guard<std::mutex> locker(_mutex_objs);
			if (_controls.size() != 0 && control->Stamp() < _controls.back()->Stamp())
			{
				_logger.Err() << "Unexpected order of the controls!!!" << endl;
				return;
			}
			_controls.push(control);
			_processobjs.push_back(control);
			_updated = false;

		}

		virtual void AddMeasurement(Measurement measurement)
		{
			std::lock_guard<std::mutex> locker(_mutex_objs);
			if (_controls.size() != 0 && measurement->Stamp() > _controls.back()->Stamp())
			{
				_logger.Err() << "Unexpected order of the measurements!!!" << endl;
				return;
			}

			// insert new obj
			auto reverse_iter = _processobjs.rbegin();

			while (reverse_iter != _processobjs.rend())
			{
				ProcessObj obj = *reverse_iter;
				if (!obj->LessPriorThan(measurement))
				{
					break;
				}
				reverse_iter++;
			}

			if (reverse_iter == _processobjs.rend())
			{
				_logger.Err() << "The new measurement is outdated!!!" << endl;
				return;
			}
			_processobjs.insert(reverse_iter.base(), measurement);
			_updated = false;
		}

		virtual void Update()
		{
			std::lock_guard<std::mutex> locker(_mutex_objs);

			auto iter = _processobjs.begin();
			ProcessObj obj;
			while (iter != _processobjs.end())
			{
				obj = *iter;
				if (!obj->Updated())
				{
					break;
				}
				iter++;
			}
			iter--;

			obj = *iter;
			auto state = obj->GetPostState();
			iter++;
			while (iter != _processobjs.end())
			{
				obj = *iter;
				auto last_state = state;
				obj->Update(last_state, state);
				iter++;
			}

			if (_controls.size() > _buffersize)
			{
				while (_controls.size() > _buffersize)
				{
					_controls.pop();
				}
				Control control = _controls.front();
				auto iter = std::find(_processobjs.begin(), _processobjs.end(), control);
				if (iter == _processobjs.end())
				{
					_logger.Err() << "Unexpected Error!! Can't find control in process objs." << endl;
					return;
				}

				_processobjs.erase(_processobjs.begin(), --iter);
			}

			_updated = true;
		}

		virtual StateVector CurrentEstimiate()
		{
			if (!_updated)
			{
				Update();
			}
			return _processobjs.back()->GetPostState()._mean;
		}

		virtual StateCovariance CurrentCovariance()
		{
			if (!_updated)
			{
				Update();
			}
			return _processobjs.back()->GetPostState()._covariance;
		}

	protected:
		int _buffersize;
		std::mutex _mutex_objs;
		std::list<ProcessObj> _processobjs;
		std::queue<Control> _controls;
		bool _updated;

	};




}