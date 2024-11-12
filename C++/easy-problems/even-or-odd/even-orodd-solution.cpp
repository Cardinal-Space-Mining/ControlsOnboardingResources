#include <iostream>

void checkEvenOdd(int number) {
    if (number % 2 == 0) {
        std::cout << number << " is even." << std::endl;
    } else {
        std::cout << number << " is odd." << std::endl;
    }
}

int main() {
    int i;
    for (i = 0; i <= 10; i++) {
        checkEvenOdd(i);
    }
    return 0;
}
