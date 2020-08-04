#include <Eigen/Eigen>

/**
 * Creates an orthonormal basis where the x-vector is given
 * and the y-vector is suggested, but can be changed. Both
 * y and z vectors are written to in this function. We assume
 * that the vector x is normalized when this function is called.
 * 
 * @param x x-axis
 * @param y y-axis
 * @param z z-axis
 * 
 * @return
 */
void makeOrthonormalBasis(const Eigen::Vector3f &x, Eigen::Vector3f *y, Eigen::Vector3f *z);