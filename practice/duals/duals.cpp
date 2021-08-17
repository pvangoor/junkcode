// Online C++ compiler to run C++ program online
#include <iostream>
#include <type_traits>
#include <cmath>

template <typename Scalar = double>
struct Dual
{
   Scalar x; // The number itself
   Scalar e; // The dual part

   Dual() = default;
   Dual(const Scalar &x, const Scalar &e = 0) : x(x), e(e){};
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

   Dual operator*(const Scalar &other) const
   {
      return Dual(this->x * other, this->e * other);
   }
   Dual operator+(const Scalar &other) const
   {
      return Dual(this->x + other, this->e);
   }
   Dual operator-(const Scalar &other) const
   {
      return Dual(this->x - other, this->e);
   }
   Dual operator/(const Scalar &other) const
   {
      return Dual(this->x / other, this->e / other);
   }
};

int main()
{
   Dual x(5.0, 3.0);

   double a = 4.0;

   Dual y = x * a;

   return 0;
}