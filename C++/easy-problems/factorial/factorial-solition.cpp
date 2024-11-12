#include <iostream>

// Iterative implementation
int factorial(int n) {
    int result = 1;
    for (int i = 1; i <= n; ++i) {
        result *= i;
    }
    return result;
}

// Recursive implementation
int factorialR(int n) {
    if (n <= 1)
        return 1;
    return n * factorialR(n - 1);
}

int main() {
    int i;
    for (i = 0; i <= 12; i++) {
        std::cout << i << ": I: " << factorial(i) << " R: "<< factorialR(i) << std::endl;
    }
    return 0;
}