#include <iostream>
#include <type_traits>
#include <cmath>
#include <chrono>

using namespace std;

template <typename Scalar = double>
struct Dual
{
   Scalar x; // The number itself
   Scalar e; // The dual part

   Dual() = default;
   Dual(const Scalar &x, const Scalar &e = 0) : x(x), e(e){};
   template<typename S2, typename S3>
   Dual(const S2 &x, const S3 &e = 0) : x(Scalar(x)), e(Scalar(e)){};
   
   Dual operator*(const Dual &other) const
   {
      return Dual(this->x * other.x, this->x * other.e + this->e * other.x);
   }
   Dual operator+(const Dual &other) const
   {
      return Dual(this->x + other.x, this->e + other.e);
   }
   Dual operator-(const Dual &other) const
   {
      return Dual(this->x - other.x, this->e - other.e);
   }
   Dual operator-() const
   {
      return Dual(-this->x, -this->e);
   }
   Dual operator/(const Dual &other) const
   {
      return Dual(this->x / other.x, (this->e - this->x * other.e / other.x) / other.x);
   }
};

template<typename Scalar> Dual<Scalar> operator*(const Dual<Scalar>& d1, const Dual<Scalar> d2) {  
   return Dual(d1.x * d2.x, d1.x * d2.e + d1.e * d2.x);
}

template<typename Scalar> Dual<Scalar> cos(const Dual<Scalar> d) {
   return Dual(cos(d.x), -sin(d.x) * d.e);
}

template<typename Scalar> Dual<Scalar> sin(const Dual<Scalar> d) {
   return Dual(sin(d.x), cos(d.x) * d.e);
}

template<typename Scalar> Dual<Scalar> tan(const Dual<Scalar> d) {
   return Dual(tan(d.x), pow(cos(d.x), -2.0) * d.e);
}

template<typename Scalar> Dual<Scalar> acos(const Dual<Scalar> d) {
   return Dual(acos(d.x), -pow(1 - d.x*d.x, -0.5) * d.e);
}

template<typename Scalar> Dual<Scalar> asin(const Dual<Scalar> d) {
   return Dual(asin(d.x), pow(1 - d.x*d.x, -0.5) * d.e);
}

template<typename Scalar> Dual<Scalar> atan(const Dual<Scalar> d) {
   return Dual(atan(d.x), 1.0 /(1 + d.x*d.x) * d.e);
}

template<typename Scalar> Dual<Scalar> exp(const Dual<Scalar> d) {
   return Dual(exp(d.x), exp(d.x) * d.e);
}

template<typename Scalar> Dual<Scalar> log(const Dual<Scalar> d) {
   return Dual(log(d.x), 1.0/d.x * d.e);
}

template<typename Scalar> Dual<Scalar> pow(const Dual<Scalar> d, const Scalar& s) {
      return Dual(pow(d.x, s),  s * pow(d.x, s-1) * d.e);
}

template<typename Scalar> Dual<Scalar> sqrt(const Dual<Scalar> d) {
   return Dual(sqrt(d.x), 1/(2*sqrt(d.x)) * d.e);
}

template<typename Scalar> Dual<Scalar> cbrt(const Dual<Scalar> d) {
   return Dual(cbrt(d.x), 1/(3*cbrt(d.x)*cbrt(d.x)) * d.e);
}

template<typename Scalar> Dual<Scalar> abs(const Dual<Scalar> d) {
   if (d.x >= 0)
      return Dual(abs(d.x), d.e);
   else
      return Dual(abs(d.x), -d.e);
}


template<typename T> T my_functor(const T& x) {
   T y = exp(x);
   T z = y * 2 + 3;
   return sin(z);
}

double my_functor_der(const double& x) {
   double drdx = cos(2* exp(x) + 3) * exp(x) * 2;
   return drdx;
}


int main()
{
   int reps = 1e9;
   chrono::time_point<chrono::steady_clock> start;
   chrono::duration<double> dur;

   start = chrono::steady_clock::now();
   srand(0);
   for (int k=0;k<reps;++k) {
      double r = static_cast<double>(rand()) /static_cast<double>(RAND_MAX);
      Dual x(r, 1.0);
      double dfdx = my_functor(x).e;
   }
   dur = chrono::steady_clock::now() - start;
   cout << "Dual: " << dur.count() << endl;

   start = chrono::steady_clock::now();
   srand(0);
   for (int k=0;k<reps;++k) {
      double r = static_cast<double>(rand()) /static_cast<double>(RAND_MAX);
      double dfdx = (my_functor(r+1e-5) - my_functor(r-1e-5)) / (2e-5);
   }
   dur = chrono::steady_clock::now() - start;
   cout << "Numerical: " << dur.count() << endl;
   
   start = chrono::steady_clock::now();
   srand(0);
   for (int k=0;k<reps;++k) {
      double r = static_cast<double>(rand()) /static_cast<double>(RAND_MAX);
      double dfdx = my_functor_der(r);
   }
   dur = chrono::steady_clock::now() - start;
   cout << "Analytic: " << dur.count() << endl;

   return 0;
}