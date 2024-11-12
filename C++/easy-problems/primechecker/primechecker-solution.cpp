#include <iostream>

int isPrime(int num) {
    if (num <= 1)
        return 0;
    
    for (int i = 2; i <= num / 2; i++)
        if (num % i == 0)
            return 0;
    
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