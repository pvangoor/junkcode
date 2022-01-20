#include <iostream>
#include <type_traits>
#include <cmath>
#include <chrono>
#include <concepts>

using namespace std;

template <size_t N, typename Scalar = double>
struct HyperDual
{
   HyperDual<N-1, Scalar> x; // The previous dual. This terminates at N=1!
   HyperDual<N-1, Scalar> e; // The additional dual part

   HyperDual() = default;
   HyperDual(const HyperDual<N-1, Scalar> &x, const HyperDual<N-1, Scalar> &e = 0) : x(x), e(e){};
   // template<typename S2>
   // HyperDual(const S2 &x) : x(HyperDual<N-1,Scalar>(x)), e(HyperDual<N-1, Scalar>(0.0)){};
   template<typename S2, typename S3 = Scalar>
   HyperDual(const S2 &x, const S3 &e = 0.0) : x(HyperDual<N-1,Scalar>(x)), e(HyperDual<N-1, Scalar>(e)){};

   template<typename T>
   static HyperDual Deriver(const T& x) {
      return HyperDual(HyperDual<N-1,Scalar>::Deriver(x), HyperDual<N-1,Scalar>(1.0));
   }
   Scalar smallestDual() const {
      if constexpr (N > 1) {
         return this->e.smallestDual();
      } else {
         return this->e;
      }
   }
   
   HyperDual operator*(const HyperDual &other) const
   {
      return HyperDual(this->x * other.x, this->x * other.e + this->e * other.x);
   }
   HyperDual operator+(const HyperDual &other) const
   {
      return HyperDual(this->x + other.x, this->e + other.e);
   }

   // template<typename T>
   // HyperDual operator*(const T &other) const
   // requires is_convertible_v<T, HyperDual>
   // {
   //    return this->operator*(HyperDual(other));
   // }
};

template <typename Scalar>
struct HyperDual<0, Scalar> {
   Scalar x;

   HyperDual() = default;
   template<typename T>
   HyperDual(const T&x)
   requires is_convertible_v<T, Scalar>
    : x(x) {};
   template<typename S2, typename S3>
   HyperDual(const S2 &x) : x(x){};

   template<typename T>
   static HyperDual Deriver(const T& x) {
      return HyperDual(x);
   }

   operator Scalar() const { return this->x; }
   
   // template<typename T>
   // HyperDual operator*(const T &other) const
   // requires is_convertible_v<T, HyperDual>
   // {
   //    return HyperDual(this->x * HyperDual(other).x);
   // }
   HyperDual operator*(const HyperDual &other) const
   {
      return HyperDual(this->x * other.x);
   }

   HyperDual operator+(const HyperDual &other) const
   {
      return HyperDual(this->x + other.x);
   }
};


struct my_functor{
   template<typename T> 
   T operator() (const T& x) const {
      // T y = exp(x);
      T z = x * x + x * 2 + 3;
      return z;
   }
};

template<size_t N, typename Scalar>
Scalar derivative(const auto& f, const Scalar& x) {
   // Returns the Nth derivative of f at x
   HyperDual<N> dnx = HyperDual<N>::Deriver(x);
   return f(dnx).smallestDual();
}


int main()
{
    double d3fdx = derivative<3>(my_functor{}, 1.0);

   cout << derivative<1>(my_functor{}, 3.0) << endl;
   cout << derivative<2>(my_functor{}, 3.0) << endl;
   cout << derivative<3>(my_functor{}, 3.0) << endl;



    return 0;
}