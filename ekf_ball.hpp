#ifndef EKF_BALL_HPP
#define EKF_BALL_HPP

#include "keisan/matrix/matrix.hpp" // Include the Matrix class in ICHIRO ITS keisan library

namespace keisan
{

class ekf_ball
{
public:
    ekf_ball();

    void init(double x, double y,double v, double theta);
    void predict(double dt);
    void update(const Matrix<2, 1> & z);

    Matrix<2, 1> getPosition() const;
    Matrix<2, 1> getVelocity() const;

    Matrix<4, 1> getstate() const;
    Matrix<4, 4> getcov() const;

private:
    Matrix<4, 1> X_; //state
    Matrix<4, 4> P_; //covarience
    Matrix<4, 4> Q_; //covarience noise
    Matrix<2, 2> R_; //measurement noise

    double normalizeAngle(double a) const; // a = angle in radians
};

}  // namespace keisan

#endif  // EKF_BALL_HPP
