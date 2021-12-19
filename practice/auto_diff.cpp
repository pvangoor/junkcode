#include <iostream>

#include <memory>
#include <vector>
#include <map>
#include <type_traits>

template<typename ftype=double>
struct Dual {
    ftype value;
    std::map<std::shared_ptr<char>, ftype> epsilon;

    Dual() = default;
    Dual(const ftype& value) : value(value) {};
    Dual(const ftype& value, const ftype& eps) : value(value) {
        epsilon[std::make_shared<char>()] = eps;
    };
};

// Addition

template<typename ftype>
Dual<ftype> operator+(const Dual<ftype>& d1, const Dual<ftype>& d2) {
    Dual<ftype> d3;
    d3.value = d1.value + d2.value;
    
    for (const auto& [key, eps] : d1.epsilon) {
        d3.epsilon[key] = eps;
    }
    for (const auto& [key, eps] : d2.epsilon) {
        if (d3.epsilon.find(key) == d3.epsilon.end()) {
            d3.epsilon[key] = eps;
        } else {
            d3.epsilon[key] += eps;
        }
    }

    return d3;
}

template<typename ftype>
Dual<ftype> operator+(const Dual<ftype>& d1, const ftype& d2) {
    return d1 + Dual<ftype>(d2);
}
template<typename ftype>
Dual<ftype> operator+( const ftype& d1, const Dual<ftype>& d2) {
    return Dual<ftype>(d2) + d1;
}


// Multiplication

template<typename ftype>
Dual<ftype> operator*(const Dual<ftype>& d1, const Dual<ftype>& d2) {
    Dual<ftype> d3;
    d3.value = d1.value * d2.value;
    
    for (const auto& [key, eps] : d1.epsilon) {
        d3.epsilon[key] = eps * d2.value;
    }
    for (const auto& [key, eps] : d2.epsilon) {
        if (d3.epsilon.find(key) == d3.epsilon.end()) {
            d3.epsilon[key] = d1.value * eps;
        } else {
            d3.epsilon[key] += d1.value * eps;
        }
    }

    return d3;
}

template<typename ftype>
Dual<ftype> operator*(const Dual<ftype>& d1, const ftype& d2) {
    return d1 * Dual<ftype>(d2);
}
template<typename ftype>
Dual<ftype> operator*( const ftype& d1, const Dual<ftype>& d2) {
    return Dual<ftype>(d2) * d1;
}



int main() {
    Dual<> d1(1, 2);
    Dual<> d2(2, 3);

    double a = 5;
    Dual<> d3 = d1 * (d1 + a) + d2;

    std::cout << d3.epsilon.begin()->second << std::endl;

    std::cout << "hi" << std::endl;
}