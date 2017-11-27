// KalmanFilter.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "LEKFBase.h"
#include "LEKF2DSample.h"

using namespace std;
using namespace ekf;
using namespace Eigen;
int main()
{
	LSimpleEKF<3> ekf;

	Matrix3d init_cov = Matrix3d::Zero();
	init_cov(0, 0) = 1e100;
	init_cov(1, 1) = 1e100;
	init_cov(2, 2) = 1e100;
	ekf.SetInitialCovariance(init_cov);
	for (int i = 0; i < 100; i++)
	{
		shared_ptr<LControl2D> controlStraight = make_shared<LControl2D>();
		controlStraight->SetControl(Vector3d(1000, 0, M_PI/50));
		Matrix3d cov = Matrix3d::Zero();
		cov(0, 0) = 100;
		cov(1, 1) = 100;
		cov(2, 2) = 1e-2;
		controlStraight->SetCovariance(cov);
		ekf.AddControl(controlStraight);
	}

	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;
	cerr << ekf.Valid() << endl;

	shared_ptr<LMeasurement2D> pM = make_shared<LMeasurement2D>();
	pM->EnableInfo(true);
	pM->SetMeasurement(Vector3d(10, 10, 0.001));
	Matrix3d cov = Matrix3d::Zero();
	cov(0, 0) = 10;
	cov(1, 1) = 10;
	cov(2, 2) = 1e-4;
	pM->setMeasurementCovariance(cov);
	ekf.AddMeasurement(pM);
	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;
	cerr << ekf.Valid() << endl;

	system("pause");
    return 0;
}

