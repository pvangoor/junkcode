// Online C++ compiler to run C++ program online
#include <iostream>
#include <concepts>
#include <type_traits>
#include <cmath>
#include <memory>

using namespace std;

// A parameter should behave nicely with numbers.
// Everything is built around doubles
// Math expressions are differentiable - that's the point!

// Expressions are defined using CRTP: generally, an expression consists of more expressions
template<typename E>
struct MathExpression {
   double value() const { return static_cast<const E&>(*this).value(); }
   double derivative(const void* param) const { return static_cast<const E&>(*this).derivative(param); }

   operator double() {return value();}
};

// concept num_expr = is_base_of<MathExpression<E>,E>::value;

template<typename E>
concept math_expr = is_base_of<MathExpression<E>, E>::value;
template<typename E>
concept num_expr = disjunction<is_base_of<MathExpression<E>,E>, is_arithmetic<E>>::value;
template<num_expr E>
using ConstMath = std::conditional<is_base_of<E, MathExpression<E>>::value, const E&, const E>::type;

template<math_expr E>
double math_value(const E& expr) {
   return expr.value();
}
template<math_expr E>
double math_derivative(const E& expr, const void* param) {
   return expr.derivative(param);
}
template<typename ValType> requires is_arithmetic<ValType>::value 
double math_value(const ValType& num) {
   return num;
}
template<typename ValType> requires is_arithmetic<ValType>::value 
double math_derivative(const ValType& num, const void* param) {
   return 0.0;
}

// Parameters are the interesting bit.
struct Par : MathExpression<Par> {
    double val;

    Par(const double v) : val(v) {};
    double value() const {return val;}
    double derivative(const void* param) const {
       return this==param ? 1.0 : 0.0;
       }

   //  template <num_expr E>
   //  Par(MathExpression<E> const& expr) : val(expr.value()), der(expr.derivative(param)) {}

   //  template <num_expr E>
   //  Par& operator=(MathExpression<E> const& expr) {
   //     val = expr.value();
   //     der = expr.derivative(param);
   //     return *this;
   //  }
    
};

// Operations
// -------------------------
// Multiplication

template<num_expr E1, num_expr E2>
struct MathProduct : MathExpression<MathProduct<E1,E2>> {
   // E1 const operand1;
   ConstMath<E1> operand1;
   ConstMath<E2> operand2;
   double value() const {
      return math_value(operand1) * math_value(operand2);
   }
   double derivative(const void* param) const {
      return math_derivative(operand1, param) * math_value(operand2) + math_value(operand1) * math_derivative(operand2, param);
   }
   MathProduct(const E1& op1, const E2& op2) : operand1(op1), operand2(op2) {};
};

template<num_expr E1, num_expr E2>
MathProduct<E1,E2> operator*(const E1& op1, const E2& op2) {
   return MathProduct<E1,E2>(op1, op2);
}

// Addition
template<num_expr E1, num_expr E2>
struct MathSum : MathExpression<MathSum<E1,E2>> {
   ConstMath<E1> operand1;
   ConstMath<E2> operand2;
   double value() const {
      return math_value(operand1) + math_value(operand2);
   }
   double derivative(const void* param) const {
      return math_derivative(operand1, param) + math_derivative(operand2, param);
   }
   MathSum(const E1& op1, const E2& op2) : operand1(op1), operand2(op2) {};
};

template<num_expr E1, num_expr E2>
MathSum<E1,E2> operator+(const E1& op1, const E2& op2) {
   return MathSum<E1,E2>(op1, op2);
}

// Subtraction
template<num_expr E1, num_expr E2>
struct MathDiff : MathExpression<MathDiff<E1,E2>> {
   ConstMath<E1> operand1;
   ConstMath<E2> operand2;
   double value() const {
      return math_value(operand1) - math_value(operand2);
   }
   double derivative(const void* param) const {
      return math_derivative(operand1, param) - math_derivative(operand2, param);
   }
   MathDiff(const E1& op1, const E2& op2) : operand1(op1), operand2(op2) {};
};

template<num_expr E1, num_expr E2>
MathDiff<E1,E2> operator-(const E1& op1, const E2& op2) {
   return MathDiff<E1,E2>(op1, op2);
}

// Negation
template<num_expr E>
struct MathNeg : MathExpression<MathNeg<E>> {
   ConstMath<E> operand;
   double value() const {
      return - math_value(operand);
   }
   double derivative(const void* param) const {
      return - math_derivative(operand, param);
   }
   MathNeg(const E& op) : operand(op) {};
};

template<num_expr E>
MathNeg<E> operator-(const E& op) {
   return MathNeg<E>(op);
}

// Division
template<num_expr E1, num_expr E2>
struct MathDivision : MathExpression<MathDivision<E1,E2>> {
   ConstMath<E1> operand1;
   ConstMath<E2> operand2;
   double value() const {
      return math_value(operand1) / math_value(operand2);
   }
   double derivative(const void* param) const {
      const double v = math_value(operand2);
      return (math_derivative(operand1, param) * math_value(operand2) - math_value(operand1) * math_derivative(operand2, param)) / (v*v);
   }
   MathDivision(const E1& op1, const E2& op2) : operand1(op1), operand2(op2) {};
};

template<num_expr E1, num_expr E2>
MathDivision<E1,E2> operator/(const E1& op1, const E2& op2) {
   return MathDivision<E1,E2>(op1, op2);
}

// CMath

// Powers
// pow
// sqrt
// cbrt
// abs

// Forward trig

template<math_expr E>
struct MathCos : MathExpression<MathCos<E>> {
   ConstMath<E> operand;
   double value() const {
      return cos(math_value(operand));
   }
   double derivative(const void* param) const {
      return -sin(math_value(operand)) * math_derivative(operand, param);
   }
   MathCos(const E& op) : operand(op) {};
};

template<math_expr E>
MathCos<E> cos(const E& op) {
   return MathCos<E>(op);
}

template<math_expr E>
struct MathSin : MathExpression<MathSin<E>> {
   ConstMath<E> operand;
   double value() const {
      return sin(math_value(operand));
   }
   double derivative(const void* param) const {
      return cos(math_value(operand)) * math_derivative(operand, param);
   }
   MathSin(const E& op) : operand(op) {};
};

template<math_expr E>
MathSin<E> sin(const E& op) {
   return MathSin<E>(op);
}

template<math_expr E>
struct MathTan : MathExpression<MathTan<E>> {
   ConstMath<E> operand;
   double value() const {
      return tan(math_value(operand));
   }
   double derivative(const void* param) const {
      return  math_derivative(operand, param) / (cos(math_value(operand))*cos(math_value(operand)));
   }
   MathTan(const E& op) : operand(op) {};
};

template<math_expr E>
MathTan<E> tan(const E& op) {
   return MathTan<E>(op);
}







// -------------------------


int main() {
   // Write C++ code here
   Par x = 2.0;

   // auto z = - 5.0 / (4 * (x + 2) + 3.0) + 15 - 2  - 2*x + 3*x*x - 5 / x ;
   auto z = 5*x*x - x;

   cout << z.derivative(&x) << endl;

   // cout << z.value() << endl;
   // cout << z.derivative(const void* param) << endl;

   return 0;
}