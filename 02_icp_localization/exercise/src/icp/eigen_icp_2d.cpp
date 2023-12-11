#include "eigen_icp_2d.h"

#include <iostream>

#include "Eigen/Cholesky"
#include "Eigen/Geometry"
#include "rotations.h"

using namespace std;

ICP::ICP(const ContainerType& fixed_, const ContainerType& moving_,
         int min_points_in_leaf)
    : _fixed(fixed_),
      _moving(moving_),
      _kd_tree(_fixed.begin(), _fixed.end(), min_points_in_leaf) {}

void ICP::computeCorrespondences() {
  _correspondences.resize(_moving.size());
  int k = 0;
  for (const auto& m : _moving) {
    const auto& mt = _X * m;
    auto ft = _kd_tree.bestMatchFast(mt, _ball_radius);
    if (!ft) continue;
    _correspondences[k]._fixed = *ft;
    _correspondences[k]._moving = mt;
    ++k;
  }
  _correspondences.resize(k);
}

// for me to test
void ICP::computeCorrespondencesFake() {
  // for me to test optimizer
  _correspondences.resize(_moving.size());
  for (size_t i = 0; i < _moving.size(); ++i) {
    _correspondences[i]._fixed = _fixed[i];
    _correspondences[i]._moving = _X * _fixed[i];
  }
}

void ICP::optimizeCorrespondences() {
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b;
  H.setZero();
  b.setZero();
  Eigen::Matrix<float, 2, 3> J;
  J.block<2, 2>(0, 0).setIdentity();
  _num_kernelized = 0;
  _num_inliers = 0;
  _chi2_sum = 0;
  for (const auto& c : _correspondences) {
    const auto& f = c._fixed;
    const auto& m = c._moving;
    J(0, 2) = -m.y();
    J(1, 2) = m.x();
    Vector2f e = m - f;
    float scale = 1;
    float chi = e.squaredNorm();
    _chi2_sum += chi;
    if (e.squaredNorm() > _kernel_chi2) {
      scale = sqrt(_kernel_chi2 / chi);
      ++_num_kernelized;
    } else {
      ++_num_inliers;
    }
    H.noalias() += scale * J.transpose() * J;
    b.noalias() += scale * J.transpose() * e;
  }
  _dx = H.ldlt().solve(-b);
  Eigen::Isometry2f dX;
  const Eigen::Matrix2f dR = Rtheta(_dx(2));
  dX.setIdentity();
  dX.linear() = dR;
  dX.translation() = _dx.block<2, 1>(0, 0);
  _X = dX * _X;
}

void ICP::run(int max_iterations) {
  int current_iteration = 0;
  while (current_iteration < max_iterations) {
    computeCorrespondences();
    optimizeCorrespondences();
    //draw(cout);
    ++current_iteration;
    cerr << "Iteration: " << current_iteration;
    cerr << " corr: " << numCorrespondences();
    cerr << " inl: " << numInliers();
    cerr << " ker: " << numKernelized();
    cerr << " chi: " << _chi2_sum << endl;
  }
}
void ICP::draw(std::ostream& os) {
  os << "set size ratio -1" << endl;
  os << "plot '-' w p ps 2 title \"fixed\", '-' w p ps 2 title \"moving\", '-' w l lw 1 title \"correspondences\" " << endl;
  for  (const auto& p: _fixed)
    os << p.transpose() << endl;
  os << "e" << endl;
  for  (const auto& p: _moving)
    os << (_X*p).transpose() << endl;
  os << "e" << endl;
  for (const auto& c: _correspondences) {
    os << c._fixed.transpose() << endl;
    os << c._moving.transpose() << endl;
    os << endl;
  }
  os << "e" << endl;
}