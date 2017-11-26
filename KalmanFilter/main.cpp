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

	shared_ptr<LControl2D> controlStraight = make_shared<LControl2D>();
	controlStraight->SetControl(Vector3d(1000, 0, 0));
	Matrix3d cov = Matrix3d::Zero();
	cov(0, 0) = 10;
	cov(1, 1) = 10;
	cov(2, 2) = 0.0001;
	controlStraight->SetCovariance(cov);
	ekf.AddControl(controlStraight);
	for (int i = 0; i< 360;i++)
	{
		shared_ptr<LControl2D> controlturn90 = make_shared<LControl2D>();
		controlturn90->SetCovariance(cov);
		controlturn90->SetControl(Vector3d(0, 1000, M_PI * 2 / 360));
		ekf.AddControl(controlturn90);
	}
	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;
	cerr << ekf.Valid() << endl;

	shared_ptr<LMeasurement2D> pM = make_shared<LMeasurement2D>();
	pM->EnableInfo(true);
	pM->SetMeasurement(Vector3d(0, 0, -M_PI_2));
	pM->setMeasurementCovariance(cov);
	ekf.AddMeasurement(pM);
	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;
	cerr << ekf.Valid() << endl;

	system("pause");
    return 0;
}

