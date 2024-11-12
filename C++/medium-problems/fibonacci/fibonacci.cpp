#include <iostream>

int fibonacci(int n) {
    if (n <= 1)
        return n;

    int a = 0, b = 1;
    int i, result;

// Implement an iterative solution 
    for (; ; ) {
        
    }

    return result;
}

// Implement a recursive fibinacci solution
int fibonacciR(int n) {
    
    return 0; // change this
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