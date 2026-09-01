// Exercises every layer of tiny_geometry.hpp and prints what each design choice buys.
#include "tiny_geometry.hpp"

#include <cmath>
#include <cstdio>

using namespace tg;

int main() {
  std::puts("--- 1. storage: shape decides stack vs heap -----------------------------");
  std::printf("  sizeof(Matrix<double,3,3>) = %2zu bytes  payload 9*8=72, rest is alignas(16)\n",
              sizeof(Matrix3d));
  std::puts("                                        padding + 2 ints. The object IS the data:");
  std::puts("                                        no pointer, no heap, no indirection.");
  std::printf("  sizeof(Matrix<double,D,D>) = %2zu bytes  ptr + size + 2 ints; data on the heap\n",
              sizeof(MatrixXd));
  std::puts("  ^ that padding is exactly why Eigen needs EIGEN_MAKE_ALIGNED_OPERATOR_NEW:");
  std::puts("    an over-aligned member makes the whole enclosing class over-aligned.");

  std::puts("\n--- 2. expression templates: a + b + c allocates nothing ----------------");
  Matrix3d a = Matrix3d::Identity(3), b = Matrix3d::Identity(3);
  Matrix3d c = a + b + 2.0 * a;              // one loop, zero temporaries
  std::printf("  trace(I + I + 2I) = %.1f   (expected 12)\n", c.trace());

  std::puts("\n--- 3. views: a block is a header, not a copy ---------------------------");
  MatrixXd big(4, 4);
  for (int j = 0; j < 4; ++j)
    for (int i = 0; i < 4; ++i) big(i, j) = 10 * i + j;
  auto v = view(big).block(1, 1, 2, 2);
  std::printf("  big(1,1)=%.0f  block(0,0)=%.0f  -> same element, no allocation\n", big(1, 1), v(0, 0));
  v(0, 0) = -1;
  std::printf("  writing through the view changes the owner: big(1,1)=%.0f\n", big(1, 1));

  std::puts("\n--- 4. image: stride != width, and size_t on the row term ---------------");
  Image<std::uint8_t> im(1000, 100, 3);
  std::printf("  width*channels = %d, stride = %zu  (padded up for alignment)\n",
              1000 * 3, im.stride());
  im(50, 10, 1) = 200;
  auto r = roi(im, 10, 50, 8, 8);
  std::printf("  roi(0,0,ch=1) = %d  -> the ROI aliases the parent buffer\n", int(r(0, 0, 1)));

  std::puts("\n--- 5. volume: one more stride, plus metric spacing ---------------------");
  Volume<std::int16_t> ct(64, 64, 32, {0.33, 0.33, 0.25});
  ct(10, 20, 5) = 900;
  std::printf("  ct(10,20,5) = %d HU, spacing = %.2f x %.2f x %.2f mm\n",
              int(ct(10, 20, 5)), ct.spacing()[0], ct.spacing()[1], ct.spacing()[2]);

  std::puts("\n--- 6. SO3/SE3: manifold types, frame-tagged ----------------------------");
  const SO3 R = SO3::exp(Vec3{0, 0, 1.5707963});           // 90 deg about z
  const SE3<World, Camera> T_world_cam(R, Vec3{10, 0, 0});
  const Vec3 p_cam{1, 0, 0};
  const Vec3 p_world = T_world_cam * p_cam;
  std::printf("  T_world_cam * (1,0,0) = (%.3f, %.3f, %.3f)   expected (10, 1, 0)\n",
              p_world.x, p_world.y, p_world.z);

  const SE3<Camera, World> T_cam_world = T_world_cam.inverse();
  const Vec3 back = T_cam_world * p_world;
  std::printf("  round trip error = %.2e mm  (inverse is R^T, -R^T t)\n", (back - p_cam).norm());
  // const auto bad = T_world_cam * T_world_cam;   // <-- will not compile: frames don't chain

  std::puts("\n--- 7. Sim3: scale is part of the type, so it cannot be dropped ---------");
  const Sim3<World, Camera> S(R, Vec3{10, 0, 0}, 1.02);    // 2% scale error
  const Vec3 q = S * p_cam;
  const Vec3 q_back = S.inverse() * q;
  std::printf("  Sim3 round trip error = %.2e mm, scale = %.3f\n", (q_back - p_cam).norm(), S.scale());

  std::puts("\n--- 8. algorithms outside the container ---------------------------------");
  // Spread the points and vary the normals: a plane-fit problem with all normals
  // parallel is rank-deficient, and the solver below would "converge" instantly to
  // a zero residual while learning nothing. Observability is a property of the data.
  PointCloud src;
  const int kN = 24;
  src.resize(kN);
  src.resize_normals(kN);
  for (int i = 0; i < kN; ++i) {
    const double a = 0.37 * i;
    src.x[i] = std::cos(a) * (1 + 0.1 * i);
    src.y[i] = std::sin(a) * (1 + 0.1 * i);
    src.z[i] = 0.05 * i * i;
    Vec3 n{std::cos(a), std::sin(a), 0.6};
    const double L = n.norm();
    src.nx[i] = n.x / L; src.ny[i] = n.y / L; src.nz[i] = n.z / L;
  }
  PointCloud dst;
  transform(T_world_cam, src, dst);
  std::printf("  transformed p0 = (%.2f, %.2f, %.2f), normal = (%.2f, %.2f, %.2f)\n",
              dst.x[0], dst.y[0], dst.z[0], dst.nx[0], dst.ny[0], dst.nz[0]);
  std::printf("  point-to-plane rms(src,src) = %.3f | point-to-point = %.3f\n",
              RmsResidual<PointToPlane>()(src, src), RmsResidual<PointToPoint>()(src, src));

  std::puts("\n--- 9. three extension points --------------------------------------------");
  // (a) compile time: the metric is a template parameter, fully inlined
  std::printf("  policy    : RmsResidual<PointToPoint>(src,dst) = %.3f\n",
              RmsResidual<PointToPoint>()(src, dst));
  // (b) runtime, value semantics: type erasure -- PointToPlane inherits nothing
  std::vector<AnyMetric> metrics {PointToPlane{}, PointToPoint{}};   // heterogeneous, by value
  const Vec3 A{0, 0, 0}, B{0, 0, 1}, N{0, 0, 1};
  std::printf("  erasure   : %zu metrics in one vector -> %.1f and %.1f\n",
              metrics.size(), metrics[0].residual(A, B, N), metrics[1].residual(A, B, N));
  // (c) runtime, coarse-grained: a virtual solver that knows nothing about clouds
  PointCloud target;
  transform(SE3<World, World>(SO3::exp(Vec3{0.01, -0.02, 0.05}), Vec3{0.2, -0.1, 0.15}), src, target);

  IcpProblem problem(src, target);
  std::unique_ptr<Solver> solver = std::make_unique<GradientDescent>(0.01);
  double params[6] = {0, 0, 0, 0, 0, 0};
  const double c0 = 0.5 * [&] { std::vector<double> r(problem.num_residuals());
                                problem.evaluate(params, r.data());
                                double a2 = 0; for (double v : r) a2 += v * v; return a2; }();
  const double c1 = solver->solve(problem, params, 200);
  std::printf("  solver    : cost %.6f -> %.6f, params z=%.4f  (2 virtual calls/iter, 0 per point)\n",
              c0, c1, params[5]);
  return 0;
}
