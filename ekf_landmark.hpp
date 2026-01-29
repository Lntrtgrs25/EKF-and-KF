#ifndef EKF_landmark_HPP
#define EKF_landmark_HPP

#include "keisan/matrix.hpp" // Include the Matrix class in ICHIRO ITS keisan library

namespace keisan
{

class ekf_landmark
{
public:
  ekf_landmark();

  void predict();
  void update(
    const Matrix<2, 1> & z, 
    double xr, 
    double yr, 
    double theta);

  Matrix<2, 1> getstate() const;
  Matrix<2, 2> getcov() const;


private:
  Matrix<2, 1> X;
  Matrix<2, 2> P;
  Matrix<2, 2> Q;
  Matrix<2, 2> R;
};

}  // namespace keisan

#endif  // KEISAN__KEISAN_HPP_
