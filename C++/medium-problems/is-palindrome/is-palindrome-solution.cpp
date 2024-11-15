#include <iostream>
#include <string>

bool isPalindromeHelper(const std::string str, int start, int end) {
    if (start >= end)  
        return true;

    if (str[start] != str[end])  
        return false;

    return isPalindromeHelper(str, start + 1, end - 1); 
}

bool isPalindrome(const std::string str) {
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