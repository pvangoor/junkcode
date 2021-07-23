// Online C++ compiler to run C++ program online
#include <iostream>

using namespace std;

template<typename E>
struct NumExpression {
    double value() const { return static_cast<E const&>(*this).value(); }
};

struct Num : NumExpression<Num> {
    double val;

    Num(const double& v) : val(v) {};
    double value() const {return val;}

    // Allow construction of number from a number Numexpression
    template <typename E>
    Num(NumExpression<E> const& expr) : val(expr.value()) {}
};

template <typename E1, typename E2>
struct NumSum : public NumExpression<NumSum<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumSum(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return _u.value() + _v.value(); }
};

template <typename E1, typename E2>
NumSum<E1, E2>
operator+(NumExpression<E1> const& u, NumExpression<E2> const& v) {
   return NumSum<E1, E2>(*static_cast<const E1*>(&u), *static_cast<const E2*>(&v));
};




int main() {
    // Write C++ code here
    Num x = 3.0;
    Num y = 5.0;
    Num z = 2.5;

    cout << "Hello" << endl;
    cout << x.value() << endl;
    // Seg fault...?
    // auto sum1 = x + y;
    // auto sum2 = sum1 + z;
    auto sum3 = x + y + z;
    // Num a = sum;
    // cout << sum1.value() << endl;
    // cout << sum2.value() << endl;
    cout << sum3.value() << endl;


    
    return 0;
}