#include <iostream>

int isPrime(int num) {
    
    // Implement function

    return 1;
}

int main() {
    int number;
    std::cout << "Enter a number: ";
    std::cin >> number;

    if (isPrime(number))
        std::cout << number << " is a prime number." << std::endl;
    else
        std::cout << number << " is not a prime number." << std::endl;

    return 0;
}