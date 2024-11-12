#include <iostream>
#include <string>

// Implement a recursive helper method to go through string str to see if it is a
// palindrome
int isPalindromeHelper(std::string str, int start, int end) {
    
    return 0; // change this
}

int isPalindrome(std::string str) {
    return isPalindromeHelper(str, 0, str.size() - 1); 
}

int main() {
    std::string str;

    std::cout << "Enter a string: ";
    std::cin >> str;

    if (isPalindrome(str))
        std::cout << "\"" << str << "\"" << " is a palindrome." << std::endl;
    else
        std::cout << "\"" << str << "\"" << " is not a palindrome." << std::endl;

    return 0;
}