#include <Eigen/Dense>
#include <iostream>

int main()
{
  Eigen::Matrix2d a;
  a << 2, 1, 1, 3;
  const Eigen::Vector2d x = a.ldlt().solve(Eigen::Vector2d(3, 5));
  std::cout << "Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "."
            << EIGEN_MINOR_VERSION << " solved x = " << x.transpose() << std::endl;
  return 0;
}
