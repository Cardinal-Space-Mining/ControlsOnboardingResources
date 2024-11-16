#include <iostream>
#include <cstdlib> 

int main() {
    int n;

    // Ask the user for the number of elements
    std::cout << "Enter the number of elements for the array: ";
    std::cin >> n;

    // Allocate memory for an array of n integers using malloc
    int* arr = (int*)malloc(n * sizeof(int));

    // Check if memory allocation was successful
    if (arr == nullptr) {
        std::cout << "Memory allocation failed!" << std::endl;
        return 1; // Exit the program with an error code
    }

    // Get input values for the array
    std::cout << "Enter " << n << " integers:" << std::endl;
    for (int i = 0; i < n; ++i) {
        std::cin >> arr[i];
    }

    // Display the values entered
    std::cout << "You entered: ";
    for (int i = 0; i < n; ++i) {
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;

    // Free the allocated memory
    free(arr);

    return 0;
}

