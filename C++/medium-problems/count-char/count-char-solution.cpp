#include <iostream>
#include <cctype>

void countVowelsConsonants(std::string str) {
    int vowels = 0, consonants = 0;
    
    for (int i = 0; i < str.size(); i++) {
        char c = str[i];
        
        if (isalpha(c)) {
            c = tolower(c);
            switch (c) {
                case 'a':
                case 'e':
                case 'i':
                case 'o':
                case 'u':
                    vowels++;
                    break;
                default:
                    consonants++;
                    break;
            }
        }
    }

    std::cout << "Vowels: " << vowels << "\nConsonants: " << consonants << std::endl;
}

int main() {
    std::string str;

    std::cout << "Enter a string: ";
    std::getline(std::cin, str);

    countVowelsConsonants(str);

    return 0;
}