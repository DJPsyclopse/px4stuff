#include "BlockEst.hpp"
#include <matrix/filter.hpp>

BlockEst::BlockEst() :
	BlockAttPosEst(nullptr, "LQGQ"),
	_x(zero<float, n_x, 1>()),
	_P(eye<float, n_x>())
{
}

// update
void BlockEst::update() {
	// base class update, handles data
	BlockAttPosEst::update();
}

// correction
void BlockEst::correctMagAccel(const Vec6 & y, const SMat6 & R) {
	Vec6 yh;
	yh.setZero();
	Vec6 r = y - yh;
	Matrix<float, 6, n_x> C;
	C.setZero(); // TODO

	// kalman correction
	Vector<float, n_x> dx;
	SquareMatrix<float, n_x> dP;
	float beta;
	kalman_correct(_P, C, R, r, dx, dP, beta);
	_x += dx;
	_P += dP;
}

void BlockEst::correctAbsAlt(const Vec1 & y, const SMat1 & R) {
	Vec1 yh;
	yh.setZero();
	Vec1 r = y - yh;
	Matrix<float, 1, n_x> C;
	C.setZero(); // TODO

	// kalman correction
	Vector<float, n_x> dx;
	SquareMatrix<float, n_x> dP;
	float beta;
	kalman_correct(_P, C, R, r, dx, dP, beta);
	_x += dx;
	_P += dP;
}

void BlockEst::correctRelAlt(const Vec1 & y, const SMat1 & R) {
	Vec1 yh;
	yh.setZero(); // TODO
	Vec1 r = y - yh;
	Matrix<float, 1, n_x> C;
	C.setZero(); // TODO

	// kalman correction
	Vector<float, n_x> dx;
	SquareMatrix<float, n_x> dP;
	float beta;
	kalman_correct(_P, C, R, r, dx, dP, beta);
	_x += dx;
	_P += dP;
}

void BlockEst::correctPosVel(const Vec6 & y, const SMat6 & R) {
	Vec6 yh;
	yh.setZero(); // TODO
	Vec6 r = y - yh;
	Matrix<float, 6, n_x> C;
	C.setZero(); // TODO

	// kalman correction
	Vector<float, n_x> dx;
	SquareMatrix<float, n_x> dP;
	float beta;
	kalman_correct(_P, C, R, r, dx, dP, beta);
	_x += dx;
	_P += dP;
}

// prediction
void BlockEst::predict(const Vec6 & y, const SMat6 & R) {
	SquareMatrix<float, n_x> Q;
	Q.setIdentity(); // TODO
	SquareMatrix<float, n_x> A;
	A.setIdentity(); // TODO
	_x += _x; // TODO
	_x.T().print();
	_P += (A*_P + _P*A.T() + Q);
}

