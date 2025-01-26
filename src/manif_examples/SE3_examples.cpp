#include <iostream>
#include <manif/SE3.h>
//#include <manif/ceres/ceres.h>

using namespace manif;

void SE3Data() {
  manif::SE3d pose6dof;

  pose6dof = pose6dof.Identity();
  std::cout << "SE3d pose6dof:" << pose6dof << "\n";

  std::cout << "pose6dof.Dim:" << pose6dof.Dim << "\n";
  std::cout << "pose6dof.DoF:" << pose6dof.DoF << "\n";

  std::cout << "pose6dof.coeffs():\n" << pose6dof.coeffs() << "\n";

  pose6dof[0] = 1;
  std::cout << "pose6dof:"
            << "x,y,z " << pose6dof.x() << "," << pose6dof.y() << ","
            << pose6dof.z() << ","
            << "\n";

  std::cout << "SE3d pose6dof.inverse():\n" << pose6dof.inverse() << "\n";

  pose6dof.quat();

  pose6dof.translation();
  pose6dof.transform();

  pose6dof = pose6dof.Random();

  std::cout << "pose6dof[0]:" << pose6dof[0] << "\n";
  std::cout << "pose6dof[1]:" << pose6dof[1] << "\n";
  std::cout << "pose6dof[2]:" << pose6dof[2] << "\n";

  return;
}

void SE3Jacobian() {

  SE3d X = SE3d::Random();
  SE3Tangentd w = SE3Tangentd::Random();

  std::cout << "Tangent w:\n"
            << w.coeffs().transpose() << "\n\n"; // Print as row vector

  std::cout << "w.exp().coeffs():\n" << w.exp().coeffs() << "\n\n";

  std::cout << "w.exp().x():\n" << w.exp().x() << "\n\n";

  std::cout << "w.exp().quat().coeffs():\n"
            << w.exp().quat().coeffs() << "\n\n";

  SE3d::Jacobian J_o_x, J_o_w;

  auto X_plus_w = X.plus(w, J_o_x, J_o_w);

  std::cout << "Original SE3d X:\n" << X << "\n\n";

  std::cout << "SE3d X (as 1x7): " << X.data() << std::endl;

  Eigen::Matrix4d T = X.transform();
  std::cout << "Transformation matrix:\n" << T << std::endl;

  std::cout << "Resulting SE3d X_plus_w:\n" << X_plus_w << "\n\n";

  std::cout << "Jacobian J_o_x:\n" << J_o_x << "\n\n";
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "Jacobian J_o_x:\n" << J_o_x.format(CleanFmt) << "\n";
  std::cout << "Jacobian J_o_w:\n" << J_o_w << "\n\n";
}

int main() {
  SE3Data();
  //  SE3Jacobian();
}
