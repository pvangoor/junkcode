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
concept num_expr = disjunction<is_base_of<NumExpression<E>,E>, is_floating_point<E>>::value;

template<typename E>
struct VarExpression {
   double operator()(const double& x) const { return value(x); }
   double value(const double& x) const { return static_cast<E const&>(*this).value(x); }
   double derivative(const double& x) const {return static_cast<E const&>(*this).derivative(x);}
};
template<typename E>
concept var_expr = is_base_of_v<VarExpression<E>,E>;

template<num_expr E>
double get_value(const E& u) {return u.value(); }
template<num_expr E>
double get_derivative(const E& u) {return u.derivative(); }
double get_value(const double& u) {return u; }
double get_derivative(const double& u) {return 0.0; }

template<var_expr E>
double get_value(const E& u, const double& x) {return u.value(x); }
template<var_expr E>
double get_derivative(const E& u, const double& x) {return u.derivative(x); }
double get_value(const double& u, const double& x) {return u; }
double get_derivative(const double& u, const double& x) {return 0.0; }

struct Par : NumExpression<Par> {
    double val;

    Par(const double& v) : val(v) {};
    double value() const {return val;}
    double derivative() const {return 1.0;}

    template <typename E>
    Par(NumExpression<E> const& expr) : val(expr.value()) {}
};

struct Var : VarExpression<Var> {
   double value(const double& x) const {return x;}
   double derivative(const double& x) const {return 1.0;}
};

// Addition
template <num_expr E1, num_expr E2>
struct NumSum : public NumExpression<NumSum<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumSum(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) + get_value(_v); }
    double derivative() const { return get_derivative(_u) + get_derivative(_v); }
};

template <num_expr E1, num_expr E2>
NumSum<E1, E2>
operator+(NumExpression<E1> const& u, NumExpression<E2> const& v) {
   return NumSum<E1, E2>(*static_cast<const E1*>(&u), *static_cast<const E2*>(&v));
};

template <num_expr E1>
NumSum<E1, double>
operator+(NumExpression<E1> const& u, double const& v) {
   return NumSum<E1, double>(*static_cast<const E1*>(&u), v);
};

template <num_expr E2>
NumSum<double, E2>
operator+(double const& u, NumExpression<E2> const& v) {
   return NumSum<double, E2>(u, *static_cast<const E2*>(&v));
};

// Variable Addition
template <typename E1, typename E2>
struct VarSum : public VarExpression<VarSum<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    VarSum(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value(const double& x) const { return get_value(_u, x) + get_value(_v, x); }
    double derivative(const double& x) const { return get_derivative(_u, x) + get_derivative(_v, x); }
};

template <typename E1, typename E2>
VarSum<E1, E2>
operator+(VarExpression<E1> const& u, VarExpression<E2> const& v) {
   return VarSum<E1, E2>(*static_cast<const E1*>(&u), *static_cast<const E2*>(&v));
};

template <typename E1>
VarSum<E1, double>
operator+(VarExpression<E1> const& u, double const& v) {
   return VarSum<E1, double>(*static_cast<const E1*>(&u), v);
};

template <typename E2>
VarSum<double, E2>
operator+(double const& u, VarExpression<E2> const& v) {
   return VarSum<double, E2>(u, *static_cast<const E2*>(&v));
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
   Par x = 1.0;

   cout << "Hello" << endl;
   cout << x.value() << endl;
   auto sum3 = 10 + 1.0 * log(exp(2*x));

   cout << sum3.value() << endl;
   cout << sum3.derivative() << endl;


   Var z;
   auto f = z + 5.0;

   cout << z(3.0) << endl;
   cout << f(4.0) << endl;



   return 0;
}