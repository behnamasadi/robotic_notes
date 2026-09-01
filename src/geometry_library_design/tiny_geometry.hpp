// tiny_geometry.hpp -- a deliberately small library that shows the *architecture* used by
// Eigen / OpenCV / PCL. It is illustrative, not competitive: no SIMD, no vectorised kernels,
// no allocator customisation. Every design decision it does make is one those libraries make
// too, and the accompanying note (docs/geometry_library_design.md) explains why.
//
// Layers, bottom to top:
//   1. Storage      where the bytes live  -- stack for fixed size, heap for dynamic
//   2. MatrixBase   CRTP: shared operations without a vtable
//   3. Matrix/View  owner vs non-owner, the axis that must stay separate from shape
//   4. Image/Cloud  strided 2-D/3-D buffers, and structure-of-arrays for point data
//   5. SO3/SE3/Sim3 manifold types with frame tags, stored as quaternion + translation
//   6. algorithms   free functions and policy classes, outside the data types
//
// C++17.
#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

namespace tg {

// ---------------------------------------------------------------------------
// 1. Storage: the one decision that separates a 3x3 from an m x n
// ---------------------------------------------------------------------------
inline constexpr int Dynamic = -1;

// Fixed extent -> the elements ARE the object. No allocation, no indirection,
// sizeof(Matrix3d) == 72, and the whole thing fits in registers/cache lines.
template <class T, int N>
struct Storage {
  alignas(alignof(T) < 16 ? 16 : alignof(T)) T data[N] {};
  Storage() = default;
  explicit Storage(int n) { (void)n; assert(n == N && "fixed-size storage cannot be resized"); }
  static constexpr int size() { return N; }
  T* ptr() { return data; }
  const T* ptr() const { return data; }
};

// Dynamic extent -> one heap block, owned by unique_ptr so the rule of five is
// mostly = default. Copy is deep and explicit; move is free.
template <class T>
struct Storage<T, Dynamic> {
  std::unique_ptr<T[]> data;
  int n = 0;

  Storage() = default;
  explicit Storage(int n_) : data(n_ ? std::make_unique<T[]>(n_) : nullptr), n(n_) {}
  Storage(const Storage& o) : data(o.n ? std::make_unique<T[]>(o.n) : nullptr), n(o.n) {
    if (n) std::copy_n(o.data.get(), n, data.get());
  }
  Storage(Storage&&) noexcept = default;
  Storage& operator=(Storage o) noexcept { swap(o); return *this; }   // copy-and-swap
  ~Storage() = default;

  void swap(Storage& o) noexcept { data.swap(o.data); std::swap(n, o.n); }
  int size() const { return n; }
  T* ptr() { return data.get(); }
  const T* ptr() const { return data.get(); }
};

// ---------------------------------------------------------------------------
// 2. CRTP base: shared behaviour with zero runtime cost
// ---------------------------------------------------------------------------
// Every matrix-like thing (an owning Matrix, a non-owning View, an unevaluated
// Sum expression) derives from this. No virtual functions anywhere: these are
// value types on a hot path, and a vtable pointer would both cost a load per
// element access and destroy their trivial copyability.
template <class Derived>
struct MatrixBase {
  const Derived& self() const { return static_cast<const Derived&>(*this); }
  Derived& self() { return static_cast<Derived&>(*this); }

  int rows() const { return self().rows(); }
  int cols() const { return self().cols(); }
  auto operator()(int i, int j) const { return self()(i, j); }

  // Written once here, works for every derived type -- static polymorphism.
  auto trace() const {
    assert(rows() == cols());
    using S = std::decay_t<decltype(self()(0, 0))>;
    S acc {};
    for (int i = 0; i < rows(); ++i) acc += self()(i, i);
    return acc;
  }
  auto squaredNorm() const {
    using S = std::decay_t<decltype(self()(0, 0))>;
    S acc {};
    for (int i = 0; i < rows(); ++i)
      for (int j = 0; j < cols(); ++j) acc += self()(i, j) * self()(i, j);
    return acc;
  }
};

// ---------------------------------------------------------------------------
// 2b. Expression templates: a + b + c allocates nothing
// ---------------------------------------------------------------------------
// Sum holds *references* to its operands and computes elements on demand. The
// temporaries a naive operator+ would create never exist; the loop is fused at
// the point the expression is finally assigned into a Matrix.
template <class A, class B>
struct Sum : MatrixBase<Sum<A, B>> {
  const A& a;
  const B& b;
  Sum(const A& a_, const B& b_) : a(a_), b(b_) { assert(a.rows() == b.rows() && a.cols() == b.cols()); }
  int rows() const { return a.rows(); }
  int cols() const { return a.cols(); }
  auto operator()(int i, int j) const { return a(i, j) + b(i, j); }
};

template <class A, class B>
Sum<A, B> operator+(const MatrixBase<A>& a, const MatrixBase<B>& b) {
  return Sum<A, B>(a.self(), b.self());
}

// Scaling, same idea.
template <class A, class S>
struct Scaled : MatrixBase<Scaled<A, S>> {
  const A& a;
  S s;
  Scaled(const A& a_, S s_) : a(a_), s(s_) {}
  int rows() const { return a.rows(); }
  int cols() const { return a.cols(); }
  auto operator()(int i, int j) const { return s * a(i, j); }
};

template <class A, class S, class = std::enable_if_t<std::is_arithmetic_v<S>>>
Scaled<A, S> operator*(S s, const MatrixBase<A>& a) { return Scaled<A, S>(a.self(), s); }

// ---------------------------------------------------------------------------
// 3. Matrix: owns its storage. Shape is a template parameter, ownership is not.
// ---------------------------------------------------------------------------
template <class T, int R, int C>
class Matrix : public MatrixBase<Matrix<T, R, C>> {
  static constexpr bool dyn = (R == Dynamic || C == Dynamic);
  static constexpr int  cap = dyn ? Dynamic : R * C;

  Storage<T, cap> s_;
  int r_ = (R == Dynamic ? 0 : R);
  int c_ = (C == Dynamic ? 0 : C);

public:
  using Scalar = T;

  Matrix() = default;

  // Only meaningful when at least one extent is Dynamic; asserts otherwise.
  Matrix(int r, int c) : s_(r * c), r_(r), c_(c) {
    assert((R == Dynamic || R == r) && (C == Dynamic || C == c));
  }

  // Evaluate any expression into a concrete matrix. This is where the fused
  // loop finally runs and where the single allocation (if any) happens.
  template <class D>
  Matrix(const MatrixBase<D>& e) : Matrix(e.self().rows(), e.self().cols()) {
    for (int i = 0; i < r_; ++i)
      for (int j = 0; j < c_; ++j) (*this)(i, j) = e.self()(i, j);
  }

  int rows() const { return r_; }
  int cols() const { return c_; }
  T* data() { return s_.ptr(); }
  const T* data() const { return s_.ptr(); }

  // Column-major, like Eigen and Fortran/LAPACK. The choice is arbitrary; the
  // discipline of writing it down once and never assuming it again is not.
  T& operator()(int i, int j) { return s_.ptr()[static_cast<std::size_t>(j) * r_ + i]; }
  const T& operator()(int i, int j) const { return s_.ptr()[static_cast<std::size_t>(j) * r_ + i]; }

  static Matrix Identity(int n = R) {
    Matrix m(n, n);
    for (int i = 0; i < n; ++i) m(i, i) = T(1);
    return m;
  }
};

using Matrix3d = Matrix<double, 3, 3>;
using Vector3d = Matrix<double, 3, 1>;
using MatrixXd = Matrix<double, Dynamic, Dynamic>;

// ---------------------------------------------------------------------------
// 3b. View: same interface, no ownership. This is the OpenCV cv::Mat header idea
//     and Eigen::Map, and it is why a sub-image or a matrix block costs nothing.
// ---------------------------------------------------------------------------
template <class T>
class MatrixView : public MatrixBase<MatrixView<T>> {
  T* p_ = nullptr;
  int r_ = 0, c_ = 0;
  std::size_t stride_ = 0;   // elements between consecutive columns

public:
  MatrixView() = default;
  MatrixView(T* p, int r, int c, std::size_t stride) : p_(p), r_(r), c_(c), stride_(stride) {}

  int rows() const { return r_; }
  int cols() const { return c_; }
  T& operator()(int i, int j) const { return p_[static_cast<std::size_t>(j) * stride_ + i]; }

  // A block of a view is a view. No copy, no allocation, arbitrarily nestable.
  MatrixView block(int i0, int j0, int nr, int nc) const {
    return MatrixView(&(*this)(i0, j0), nr, nc, stride_);
  }
};

template <class T, int R, int C>
MatrixView<T> view(Matrix<T, R, C>& m) { return MatrixView<T>(m.data(), m.rows(), m.cols(), m.rows()); }

// ---------------------------------------------------------------------------
// 4a. Images: a strided 2-D (or 3-D) buffer. Same owner/view split.
// ---------------------------------------------------------------------------
template <class T>
class Image {
  Storage<T, Dynamic> s_;
  int w_ = 0, h_ = 0, c_ = 1;
  std::size_t stride_ = 0;   // elements per row, >= w*c when padded for alignment

public:
  Image() = default;
  Image(int w, int h, int channels = 1, std::size_t align_elems = 16)
      : w_(w), h_(h), c_(channels),
        stride_(((static_cast<std::size_t>(w) * channels + align_elems - 1) / align_elems) * align_elems) {
    s_ = Storage<T, Dynamic>(static_cast<int>(stride_ * h));
  }

  int width() const { return w_; }
  int height() const { return h_; }
  int channels() const { return c_; }
  std::size_t stride() const { return stride_; }
  T* data() { return s_.ptr(); }
  const T* data() const { return s_.ptr(); }

  // size_t on the row term: y * stride in int overflows past ~2 Gpx.
  T& operator()(int y, int x, int ch = 0) {
    return s_.ptr()[static_cast<std::size_t>(y) * stride_ + static_cast<std::size_t>(x) * c_ + ch];
  }
  const T& operator()(int y, int x, int ch = 0) const {
    return s_.ptr()[static_cast<std::size_t>(y) * stride_ + static_cast<std::size_t>(x) * c_ + ch];
  }
};

// Non-owning rectangle into an Image -- the ROI. Identical element access, no copy.
template <class T>
struct ImageView {
  T* p = nullptr;
  int w = 0, h = 0, c = 1;
  std::size_t stride = 0;
  T& operator()(int y, int x, int ch = 0) const {
    return p[static_cast<std::size_t>(y) * stride + static_cast<std::size_t>(x) * c + ch];
  }
};

template <class T>
ImageView<T> roi(Image<T>& im, int x0, int y0, int w, int h) {
  return ImageView<T>{&im(y0, x0, 0), w, h, im.channels(), im.stride()};
}

// A volume is the same object with one more stride. Nothing else changes.
template <class T>
class Volume {
  Storage<T, Dynamic> s_;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::array<double, 3> spacing_ {1, 1, 1};   // mm per voxel: CT is metric, carry it in the type

public:
  Volume() = default;
  Volume(int nx, int ny, int nz, std::array<double, 3> spacing = {1, 1, 1})
      : s_(nx * ny * nz), nx_(nx), ny_(ny), nz_(nz), spacing_(spacing) {}

  T& operator()(int x, int y, int z) {
    return s_.ptr()[(static_cast<std::size_t>(z) * ny_ + y) * nx_ + x];
  }
  std::array<double, 3> spacing() const { return spacing_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
};

// ---------------------------------------------------------------------------
// 4b. Point cloud: structure of arrays, not array of structures
// ---------------------------------------------------------------------------
// PCL stores AoS (`std::vector<PointXYZRGB>`), which is cache-friendly when every
// kernel touches every field, and wasteful when one kernel touches only xyz --
// it drags colour through cache and defeats vectorisation. SoA is the layout a
// GPU or a SIMD kernel wants. Both are defensible; the point is that it is a
// decision, and it belongs in the container, not scattered through algorithms.
struct PointCloud {
  std::vector<double> x, y, z;          // always present
  std::vector<double> nx, ny, nz;       // optional attribute
  int width = 0, height = 1;            // height > 1 => organised (came from a depth image)

  std::size_t size() const { return x.size(); }
  bool organised() const { return height > 1; }
  bool has_normals() const { return nx.size() == x.size() && !nx.empty(); }

  void resize(std::size_t n) { x.resize(n); y.resize(n); z.resize(n); width = static_cast<int>(n); }
  void resize_normals(std::size_t n) { nx.resize(n); ny.resize(n); nz.resize(n); }
};

// ---------------------------------------------------------------------------
// 5. Manifold types. NOT matrices -- a rotation is not a 3x3 array of numbers
//    that happens to be orthonormal, and typing it as one is how you end up
//    "adding" two rotations.
// ---------------------------------------------------------------------------
struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  double norm() const { return std::sqrt(dot(*this)); }
};

// Stored as a unit quaternion: 4 doubles instead of 9, composition is cheaper,
// and one normalise() per update keeps it exactly on the manifold.
class SO3 {
  double w_ = 1, x_ = 0, y_ = 0, z_ = 0;

public:
  SO3() = default;
  SO3(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) { normalize(); }

  void normalize() {
    const double n = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
    w_ /= n; x_ /= n; y_ /= n; z_ /= n;
  }

  static SO3 exp(const Vec3& w) {                    // axis-angle -> rotation
    const double th = w.norm();
    if (th < 1e-12) return SO3(1, w.x * 0.5, w.y * 0.5, w.z * 0.5);
    const double s = std::sin(th * 0.5) / th;
    return SO3(std::cos(th * 0.5), w.x * s, w.y * s, w.z * s);
  }

  SO3 inverse() const { return SO3(w_, -x_, -y_, -z_); }   // conjugate; no matrix inverse anywhere

  SO3 operator*(const SO3& o) const {
    return SO3(w_ * o.w_ - x_ * o.x_ - y_ * o.y_ - z_ * o.z_,
               w_ * o.x_ + x_ * o.w_ + y_ * o.z_ - z_ * o.y_,
               w_ * o.y_ - x_ * o.z_ + y_ * o.w_ + z_ * o.x_,
               w_ * o.z_ + x_ * o.y_ - y_ * o.x_ + z_ * o.w_);
  }

  Vec3 operator*(const Vec3& v) const {              // rotate a point
    const Vec3 u{x_, y_, z_};
    const Vec3 t = Vec3{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x} * 2.0;
    return v + t * w_ + Vec3{u.y * t.z - u.z * t.y, u.z * t.x - u.x * t.z, u.x * t.y - u.y * t.x};
  }

  Matrix3d matrix() const {                          // only when something demands 3x3
    Matrix3d R;
    R(0,0) = 1 - 2*(y_*y_ + z_*z_); R(0,1) = 2*(x_*y_ - z_*w_); R(0,2) = 2*(x_*z_ + y_*w_);
    R(1,0) = 2*(x_*y_ + z_*w_);     R(1,1) = 1 - 2*(x_*x_ + z_*z_); R(1,2) = 2*(y_*z_ - x_*w_);
    R(2,0) = 2*(x_*z_ - y_*w_);     R(2,1) = 2*(y_*z_ + x_*w_);     R(2,2) = 1 - 2*(x_*x_ + y_*y_);
    return R;
  }
};

// Frame tags: the convention lives in the type, so the compiler checks it.
// T_world_cam * T_cam_point compiles; T_world_cam * T_world_cam does not.
struct World {}; struct Camera {}; struct CT {};

template <class From, class To>
class SE3 {
  SO3 R_;
  Vec3 t_;

public:
  SE3() = default;
  SE3(const SO3& R, const Vec3& t) : R_(R), t_(t) {}

  const SO3& rotation() const { return R_; }
  const Vec3& translation() const { return t_; }

  SE3<To, From> inverse() const {                    // (R^T, -R^T t) -- never a 4x4 inverse
    const SO3 Ri = R_.inverse();
    return SE3<To, From>(Ri, Ri * t_ * -1.0);
  }

  template <class Next>
  SE3<From, Next> operator*(const SE3<To, Next>& o) const {
    return SE3<From, Next>(R_ * o.rotation(), R_ * o.translation() + t_);
  }

  Vec3 operator*(const Vec3& p) const { return R_ * p + t_; }

  // A normal is not a point: it ignores translation, and under a non-uniform or
  // scaled transform it needs the inverse transpose. Separate overload, on purpose.
  Vec3 rotate_normal(const Vec3& n) const { return R_ * n; }
};

// Monocular reconstruction produces a similarity, not a rigid transform. Giving
// it its own type is what stops the scale being silently dropped.
template <class From, class To>
class Sim3 {
  SO3 R_;
  Vec3 t_;
  double s_ = 1.0;

public:
  Sim3() = default;
  Sim3(const SO3& R, const Vec3& t, double s) : R_(R), t_(t), s_(s) {}
  double scale() const { return s_; }
  Vec3 operator*(const Vec3& p) const { return R_ * p * s_ + t_; }
  Sim3<To, From> inverse() const {                   // (1/s, R^T, -R^T t / s)
    const SO3 Ri = R_.inverse();
    return Sim3<To, From>(Ri, Ri * t_ * (-1.0 / s_), 1.0 / s_);
  }
};

// ---------------------------------------------------------------------------
// 6. Algorithms live OUTSIDE the data types
// ---------------------------------------------------------------------------
// The container's job is to own bytes and describe shape. Everything else is a
// free function or a policy class. This keeps PointCloud small enough to reason
// about, lets a new algorithm be added without touching it, and makes each
// algorithm independently testable.

template <class From, class To>
void transform(const SE3<From, To>& T, const PointCloud& in, PointCloud& out) {
  out.resize(in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    const Vec3 p = T * Vec3{in.x[i], in.y[i], in.z[i]};
    out.x[i] = p.x; out.y[i] = p.y; out.z[i] = p.z;
  }
  if (in.has_normals()) {                            // normals rotate, never translate
    out.resize_normals(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
      const Vec3 n = T.rotate_normal(Vec3{in.nx[i], in.ny[i], in.nz[i]});
      out.nx[i] = n.x; out.ny[i] = n.y; out.nz[i] = n.z;
    }
  }
  out.height = in.height;                            // organised-ness survives a rigid transform
}

// Policy-based algorithm class: the strategy is a template parameter, resolved at
// compile time, so there is no virtual call in the inner loop. Swap the policy to
// get point-to-point instead of point-to-plane without touching the driver.
struct PointToPlane {
  static double residual(const Vec3& src, const Vec3& dst, const Vec3& n) { return (src - dst).dot(n); }
};
struct PointToPoint {
  static double residual(const Vec3& src, const Vec3& dst, const Vec3&) { return (src - dst).norm(); }
};

template <class Metric = PointToPlane>
class RmsResidual {
public:
  double operator()(const PointCloud& src, const PointCloud& dst) const {
    assert(src.size() == dst.size());
    double acc = 0;
    for (std::size_t i = 0; i < src.size(); ++i) {
      const Vec3 n = dst.has_normals() ? Vec3{dst.nx[i], dst.ny[i], dst.nz[i]} : Vec3{0, 0, 1};
      const double r = Metric::residual(Vec3{src.x[i], src.y[i], src.z[i]},
                                        Vec3{dst.x[i], dst.y[i], dst.z[i]}, n);
      acc += r * r;
    }
    return std::sqrt(acc / static_cast<double>(src.size()));
  }
};

// ---------------------------------------------------------------------------
// 6b. Type erasure: runtime flexibility WITH value semantics
// ---------------------------------------------------------------------------
// PointToPlane below does not inherit from anything and does not know AnyMetric
// exists. Use this when strategies must live in a container, cross an ABI
// boundary, or be swapped by a script -- and accept the allocation + indirect
// call that buys it. This is what std::function is, internally.
class AnyMetric {
  struct Concept {
    virtual ~Concept() = default;
    virtual double residual(const Vec3&, const Vec3&, const Vec3&) const = 0;
    virtual std::unique_ptr<Concept> clone() const = 0;
  };
  template <class M>
  struct Model : Concept {
    M m;
    explicit Model(M m_) : m(std::move(m_)) {}
    double residual(const Vec3& a, const Vec3& b, const Vec3& n) const override {
      return M::residual(a, b, n);
    }
    std::unique_ptr<Concept> clone() const override { return std::make_unique<Model>(m); }
  };
  std::unique_ptr<Concept> p_;

public:
  template <class M>
  AnyMetric(M m) : p_(std::make_unique<Model<M>>(std::move(m))) {}
  AnyMetric(const AnyMetric& o) : p_(o.p_->clone()) {}
  AnyMetric(AnyMetric&&) noexcept = default;
  AnyMetric& operator=(AnyMetric o) noexcept { p_.swap(o.p_); return *this; }
  double residual(const Vec3& a, const Vec3& b, const Vec3& n) const { return p_->residual(a, b, n); }
};

// ---------------------------------------------------------------------------
// 6c. Virtual, at the coarse-grained boundary only
// ---------------------------------------------------------------------------
// The solver knows nothing about PointCloud, SE3 or images -- only how to ask for
// residuals. Two virtual calls per ITERATION; zero per point, because evaluate()
// fills a whole buffer at once. That batching is what makes virtual affordable.
class Problem {
public:
  virtual ~Problem() = default;
  virtual int num_residuals() const = 0;
  virtual int num_parameters() const = 0;
  virtual void evaluate(const double* params, double* residuals) const = 0;
};

class Solver {
public:
  virtual ~Solver() = default;
  virtual double solve(const Problem& p, double* params, int iters) = 0;
};

// A deliberately trivial solver: enough to show the seam, not to be useful.
class GradientDescent : public Solver {
  double step_;
public:
  explicit GradientDescent(double step = 1e-3) : step_(step) {}
  double solve(const Problem& p, double* params, int iters) override {
    const int m = p.num_residuals(), n = p.num_parameters();
    std::vector<double> r(m), r2(m), g(n);
    auto cost = [&](const double* x, std::vector<double>& buf) {
      p.evaluate(x, buf.data());
      double c = 0; for (double v : buf) c += v * v; return 0.5 * c;
    };
    double c = cost(params, r);
    for (int it = 0; it < iters; ++it) {
      for (int j = 0; j < n; ++j) {                    // numeric gradient: illustrative only
        const double h = 1e-6, old = params[j];
        params[j] = old + h; const double cp = cost(params, r2);
        params[j] = old;     g[j] = (cp - c) / h;
      }
      for (int j = 0; j < n; ++j) params[j] -= step_ * g[j];
      c = cost(params, r);
    }
    return c;
  }
};

// The adapter lives on the geometry side: it knows clouds, the solver does not.
// Parameters are a plain double[6] -- an se(3) increment. The manifold never
// leaks into the solver; the solver never leaks into the geometry.
class IcpProblem : public Problem {
  const PointCloud& src_;
  const PointCloud& dst_;
public:
  IcpProblem(const PointCloud& s, const PointCloud& d) : src_(s), dst_(d) {}
  int num_parameters() const override { return 6; }
  int num_residuals() const override { return static_cast<int>(src_.size()); }
  void evaluate(const double* p, double* r) const override {
    const SE3<World, World> T(SO3::exp(Vec3{p[0], p[1], p[2]}), Vec3{p[3], p[4], p[5]});
    for (std::size_t i = 0; i < src_.size(); ++i) {
      const Vec3 q = T * Vec3{src_.x[i], src_.y[i], src_.z[i]};
      const Vec3 d{dst_.x[i], dst_.y[i], dst_.z[i]};
      const Vec3 n = dst_.has_normals() ? Vec3{dst_.nx[i], dst_.ny[i], dst_.nz[i]} : Vec3{0, 0, 1};
      r[i] = PointToPlane::residual(q, d, n);
    }
  }
};

}  // namespace tg
