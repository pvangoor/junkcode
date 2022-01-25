#include <random>
#include <iostream>

int main() {
    // Use the same seed each time
    std::mt19937 mt(0);

    // Create uniform int distribution between 0 and 100.
    std::uniform_int_distribution<size_t> dist(0, 100);

    // Print random ints from the distribution.
    for (int i=0;i<10;++i) {
        std::cout << dist(mt) << ", ";
    }

    // Are these the same every time the program is run?

    std::cout << std::endl;
}