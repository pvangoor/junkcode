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
// concept num_expr = is_base_of<NumExpression<E>,E>::value;
concept num_expr = disjunction<is_base_of<NumExpression<E>,E>, is_floating_point<E>>::value;

template<num_expr E>
double get_value(const E& u) {return u.value(); }
template<num_expr E>
double get_derivative(const E& u) {return u.derivative(); }
double get_value(const double& u) {return u; }
double get_derivative(const double& u) {return 0.0; }

struct Par : NumExpression<Par> {
    double val;

    Par(const double v) : val(v) {};
    double value() const {return val;}
    double derivative() const {return 1.0;}

    template <typename E>
    Par(NumExpression<E> const& expr) : val(expr.value()) {}
};

// struct Var : VarExpression<Var> {
//    double value(const double& x) const {return x;}
//    double derivative(const double& x) const {return 1.0;}
// };

// Addition
template <num_expr E1, num_expr E2>
struct NumSum : public NumExpression<NumSum<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumSum(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) + get_value(_v); }
    double derivative() const { return get_derivative(_u) + get_derivative(_v); }
};

template <num_expr E1>
struct NumSum<E1, double> : public NumExpression<NumSum<E1, double> > {
    E1 const& _u;
    double const _v;

    NumSum(E1 const& u, double const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) + get_value(_v); }
    double derivative() const { return get_derivative(_u) + get_derivative(_v); }
};

template <num_expr E2>
struct NumSum<double, E2> : public NumExpression<NumSum<double, E2> > {
    double const _u;
    E2 const& _v;

    NumSum(double const& u, E2 const& v) : _u(u), _v(v) {}
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

// Multiplication
template <typename E1, typename E2>
struct NumProd : public NumExpression<NumProd<E1, E2> > {
    E1 const& _u;
    E2 const& _v;

    NumProd(E1 const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) * get_value(_v); }
    double derivative() const { return get_value(_v) * get_derivative(_u) + get_value(_u) * get_derivative(_v); }
};

template <typename E2>
struct NumProd<double, E2> : public NumExpression<NumProd<double, E2> > {
    double const _u;
    E2 const& _v;

    NumProd(double const& u, E2 const& v) : _u(u), _v(v) {}
    double value() const { return get_value(_u) * get_value(_v); }
    double derivative() const { return get_value(_v) * get_derivative(_u) + get_value(_u) * get_derivative(_v); }
};

template <typename E1>
struct NumProd<E1, double> : public NumExpression<NumProd<E1, double> > {
    E1 const& _u;
    double const _v;

    NumProd(E1 const& u, double const& v) : _u(u), _v(v) {}
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

template<typename F>
auto Derivative(const F & f) {
   auto Df = [&f](const auto& x) { return get_derivative(f(x));};
   return Df;
}


int main() {
   // Write C++ code here
   Par x = 1.0;

   cout << "Hello" << endl;
   cout << x.value() << endl;
   auto sum3 = 10 + 1.0 * log(exp(2*x));

   cout << sum3.value() << endl;
   cout << sum3.derivative() << endl;


   // Var z;
   auto f = [](const Par& z) {
      auto y = 5.0 * z;
      return y;
      // auto y2 = 2.0 + y;
      // return exp(y2);
   };
   auto df = Derivative(f);

   // cout << z(3.0) << endl;
   Par z = 4.0;
   auto fz = f(z);
   // auto fz = 5.0 + z;
   auto fz2 = 2.0 + fz;
   double y = get_value(fz2);

   double dfdz = get_value(df(z));

   cout << y << endl;
   cout << dfdz << endl;



   return 0;
}