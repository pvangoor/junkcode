// Online C++ compiler to run C++ program online
#include <iostream>
#include <concepts>
#include <type_traits>
#include <cmath>

using namespace std;

template<typename E>
struct NumExpression {
    double value() const { return static_cast<E const&>(*this).value(); }
    double derivative() const {return static_cast<E const&>(*this).derivative();}
};

template<typename E>
double get_value(const E& u) {return u.value(); }
template<typename E>
double get_derivative(const E& u) {return u.derivative(); }

template<>
double get_value(const double& u) {return u; }
template<>
double get_derivative(const double& u) {return 0.0; }

struct Var : NumExpression<Var> {
    double val;

    Var(const double& v) : val(v) {};
    double value() const {return val;}
    double derivative() const {return 1.0;}

    template <typename E>
    Var(NumExpression<E> const& expr) : val(expr.value()) {}
};

// Addition
template <typename E1, typename E2>
struct NumSum : public NumExpression<NumSum<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumSum(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) + get_value(_v); }
    double derivative() const { return get_derivative(_u) + get_derivative(_v); }
};

template <typename E1, typename E2>
NumSum<E1, E2>
operator+(NumExpression<E1> const& u, NumExpression<E2> const& v) {
   return NumSum<E1, E2>(*static_cast<const E1*>(&u), *static_cast<const E2*>(&v));
};

template <typename E1>
NumSum<E1, double>
operator+(NumExpression<E1> const& u, double const& v) {
   return NumSum<E1, double>(*static_cast<const E1*>(&u), v);
};

template <typename E2>
NumSum<double, E2>
operator+(double const& u, NumExpression<E2> const& v) {
   return NumSum<double, E2>(u, *static_cast<const E2*>(&v));
};

// Multiplication
template <typename E1, typename E2>
struct NumProd : public NumExpression<NumProd<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumProd(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) * get_value(_v); }
    double derivative() const { return get_value(_v) * get_derivative(_u) + get_value(_u) * get_derivative(_v); }
};

template <typename E1, typename E2>
NumProd<E1, E2>
operator*(NumExpression<E1> const& u, NumExpression<E2> const& v) {
   return NumProd<E1, E2>(*static_cast<const E1*>(&u), *static_cast<const E2*>(&v));
};

template <typename E1>
NumProd<E1, double>
operator*(NumExpression<E1> const& u, double const& v) {
   return NumProd<E1, double>(*static_cast<const E1*>(&u), v);
};

template <typename E2>
NumProd<double, E2>
operator*(double const& u, NumExpression<E2> const& v) {
   return NumProd<double, E2>(u, *static_cast<const E2*>(&v));
};

// exponential
template <typename E>
struct NumExp : public NumExpression<NumExp<E>> {
    E const& _u;

    NumExp(E const& u) : _u(u) {}
    double value() const { return exp(get_value(_u)); }
    double derivative() const { return exp(get_value(_u)) * get_derivative(_u); }
};

template <typename E>
NumExp<E>
exp(NumExpression<E> const& u) {
   return NumExp<E>(*static_cast<const E*>(&u));
};

// Logarithm
template <typename E>
struct NumLog : public NumExpression<NumLog<E>> {
    E const& _u;

    NumLog(E const& u) : _u(u) {}
    double value() const { return log(get_value(_u)); }
    double derivative() const { return get_derivative(_u) / get_value(_u); }
};

template <typename E>
NumLog<E>
log(NumExpression<E> const& u) {
   return NumLog<E>(*static_cast<const E*>(&u));
};

int main() {
    // Write C++ code here
    Var x = 1.0;
    
    cout << "Hello" << endl;
    cout << x.value() << endl;
    auto sum3 = 10 + 1.0 * log(exp(2*x));

    cout << sum3.value() << endl;
    cout << sum3.derivative() << endl;


    
    return 0;
}