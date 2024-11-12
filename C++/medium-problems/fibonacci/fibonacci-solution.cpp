#include <iostream>

int fibonacci(int n) {
    if (n <= 1)
        return n;

    int a = 0, b = 1;
    int i, result;

    for (i = 2; i <= n; i++) {
        result = a + b;
        a = b;
        b = result;
    }

    return result;
}

int fibonacciR(int n) {
    if (n <= 1)
        return n;

    return fibonacciR(n - 1) + fibonacci(n - 2);
}

int main() {
    int n = 46;

    std::cout << "Iterative: " << std::endl;
    for (int i = 0; i <= n; i++) {
        std::cout << "n: " << i << "\t f(" << i << "): " << fibonacci(i) << std::endl;
    }

    std::cout << "\nRecursive: " << std::endl;
    for (int i = 0; i <= n; i++) {
        std::cout << "n: " << i << "\t f(" << i << "): " << fibonacciR(i) << std::endl;
    }
}