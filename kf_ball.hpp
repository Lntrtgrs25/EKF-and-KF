#ifndef EKF_BALL_HPP
#define EKF_BALL_HPP

#include "keisan/matrix.hpp" // Include the Matrix class in ICHIRO ITS keisan library

namespace keisan
{

class ekf_ball
{
public:
    ekf_ball();

    void init(double x, double y);
    void predict(double dt);
    void update(const Matrix<2, 1> & z);

    Matrix<2, 1> getPosition() const;
    Matrix<2, 1> getVelocity() const;

    Matrix<4, 1> getState() const;
    Matrix<4, 4> getCovariance() const;

private:
    Matrix<4, 1> X_;
    Matrix<4, 4> P_;
    Matrix<4, 4> Q_;
    Matrix<2, 2> R_;
};

}  // namespace keisan

#endif  // EKF_BALL_HPP
