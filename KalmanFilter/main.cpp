// KalmanFilter.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "LEKFBase.h"
#include "LEKF2DSample.h"

using namespace std;
using namespace ekf;
using namespace Eigen;


void sample1()
{
	LEKF<3> ekf;


	for (int i = 0; i < 1000; i++)
	{
		shared_ptr<LControl2D> controlStraight = make_shared<LControl2D>();
		controlStraight->SetControl(Vector3d(1000, 0, M_PI/50));
		Matrix3d cov = Matrix3d::Zero();
		cov(0, 0) = 100;
		cov(1, 1) = 100;
		cov(2, 2) = 1e-2;
		controlStraight->SetControlCovariance(cov);
		ekf.AddControl(controlStraight);
	}

	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;

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

	system("pause");
}
void sample2()
{
	LEKF<3> ekf;

	shared_ptr<LMeasurementAruco> pM(new LMeasurementAruco);
		// =make_shared<LMeasurementAruco>();
	pM->SetMeasurement(Vector2d(10, 0));
	pM->SetMarkerPos(Vector2d(0, 0));
	Matrix2d cov = Matrix2d::Zero();
	cov(0, 0) = 100;
	cov(1, 1) = 100;
	pM->setMeasurementCovariance(cov);
	ekf.AddMeasurement(pM);
	ekf.Update();

	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;

	for (int i = 0; i < 300; i++)
	{
		shared_ptr<LControl2D> controlStraight = make_shared<LControl2D>();
		controlStraight->SetControl(Vector3d(10, 0, 0));
		Matrix3d state_cov = Matrix3d::Zero();
		state_cov(0, 0) = 0.01;
		state_cov(1, 1) = 0.01;
		state_cov(2, 2) = 1e-6;
		controlStraight->SetControlCovariance(state_cov);
		ekf.AddControl(controlStraight);
	}

	shared_ptr<LMeasurementAruco> pM2(new LMeasurementAruco);
		//make_shared<LMeasurementAruco>();
	pM2->SetMeasurement(Vector2d(10, 0));
	pM2->SetMarkerPos(Vector2d(0, 3005));
	pM2->setMeasurementCovariance(cov);
	ekf.AddMeasurement(pM2);
	ekf.Update();
	cerr << ekf.CurrentEstimiate() << endl;
	cerr << ekf.CurrentCovariance() << endl;

	system("pause");
}

int main()
{
	sample1();
	sample2();
	return 0;
}