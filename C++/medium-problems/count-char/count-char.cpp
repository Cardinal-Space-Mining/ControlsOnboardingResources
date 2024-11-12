#include <iostream>
#include <cctype>

// Implement method with with a switch
void countVowelsConsonants(std::string str) {
    int vowels = 0, consonants = 0;
    
    // switch example:
    // switch (char c){
    //     case 'a':
    //         break;
    //     case 'b':
    //     case 'c': // case b and falls through case c
    //         break;
    //     default:
    //         break;
    // }

    std::cout << "Vowels: " << vowels << "\nConsonants: " << consonants << std::endl;
}

int main() {
    std::string str;

    std::cout << "Enter a string: ";
    std::getline(std::cin, str);

    countVowelsConsonants(str);

    return 0;
}