#include <cmath>
#include <iostream>

struct Dual {
  double val;  // the actual value
  double dval; // the derivative wrt the chosen variable

  // Constructor
  Dual(double v = 0.0, double dv = 0.0) : val(v), dval(dv) {}
};

// Overload operator+ (Dual + Dual)
Dual operator+(const Dual &x, const Dual &y) {
  // (x.val + y.val, x.dval + y.dval)
  return Dual(x.val + y.val, x.dval + y.dval);
}

// Overload operator+ (Dual + double)
Dual operator+(const Dual &x, double c) {
  // (x.val + c, x.dval + 0)
  return Dual(x.val + c, x.dval);
}

// Overload operator+ (double + Dual)
Dual operator+(double c, const Dual &x) {
  // same as above
  return x + c;
}

// Overload operator* (Dual * Dual)
Dual operator*(const Dual &x, const Dual &y) {
  // derivative of product: d/dv (x*y) = x'*y + x*y'
  return Dual(x.val * y.val, x.dval * y.val + x.val * y.dval);
}

// Overload operator* (Dual * double)
Dual operator*(const Dual &x, double c) {
  // (x.val * c, x.dval * c)
  return Dual(x.val * c, x.dval * c);
}

// Overload operator* (double * Dual)
Dual operator*(double c, const Dual &x) { return x * c; }

// Overload sin(Dual)
Dual sin(const Dual &x) {
  // val = sin(x.val)
  // derivative = cos(x.val) * x.dval
  return Dual(std::sin(x.val), std::cos(x.val) * x.dval);
}

// Overload cos(Dual) if you need it, similarly...
// Overload other functions as needed...

// Dual f(const Dual& A, const Dual& B) {
//     return B * sin(A) + B * B; // b*sin(a) + b^2
// }

template <typename T> T f(const T &A, const T &B) {
  return B * sin(A) + B * B; // b*sin(a) + b^2
}

int main() {
  std::cout << f(3.0, M_PI / 4) << std::endl;

  double a = 1.0;
  double b = 2.0;

  // (1) Partial w.r.t. a:
  Dual A_a(a, 1.0);
  Dual B_a(b, 0.0);
  Dual fa = f(A_a, B_a);
  std::cout << "f(a,b) w.r.t a => f = " << fa.val << ", df/da = " << fa.dval
            << std::endl;

  // (2) Partial w.r.t. b:
  Dual A_b(a, 0.0);
  Dual B_b(b, 1.0);
  Dual fb = f(A_b, B_b);
  std::cout << "f(a,b) w.r.t b => f = " << fb.val << ", df/db = " << fb.dval
            << std::endl;
}
