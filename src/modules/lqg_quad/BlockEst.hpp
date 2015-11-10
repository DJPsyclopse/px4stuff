#pragma once

#include <controllib/estimators/BlockAttPosEst.hpp>

class BlockEst : public BlockAttPosEst {
public:
	BlockEst();

	// override the base update function
	void update();

private:
	// correction (required to implement from base)
	void correctMagAccel(const Vec6 & y, const SMat6 & R);
	void correctRelAlt(const Vec1 & y, const SMat1 & R);
	void correctAbsAlt(const Vec1 & y, const SMat1 & R);
	void correctPosVel(const Vec6 & y, const SMat6 & R);

	// prediction (required to implement from base)
	void predict(const Vec6 & y, const SMat6 & Q);

	static const size_t n_x = 10;

	// state
	Vector<float, n_x> _x;

	// covariance
	SquareMatrix<float, n_x> _P;
};
