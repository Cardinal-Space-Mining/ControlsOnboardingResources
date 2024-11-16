# Introduction to Dynamic Memory Allocation in C++

This is a basic program designed to demonstrate how to use `malloc` and `free`
for dynamic memory allocation. While `malloc` is more commonly used in C
programming, there are some applications for its usage in C++. 

## Program Overview

You will be fully implementing this program :)

This program should do the following:

- Prompt the user to enter the number of elements they would like in an integer
  array.
- Allocates memory dynamically for the array using `malloc`.
- Make sure that the memory is properly allocated, if not exit the program with
  `return 1`.
- Require the user to input values for the array.
- Prints the contents of the integer array.
- Free the allocated memory before the programs exits.

## Required Files

- `malloc-practice.cpp`

## Prerequisites

- C++ compiler (e.g., `g++` or `clang`)

## Instructions to Run

### Compile the Program

```
g++ malloc-practice.cpp -o malloc-practice
```

### Run the Program

```
./malloc-practice
```

### Example Run

```
Enter the number of elements for the array: 5
Enter 5 integers:
1
2
3
4
5
You entered: 1 2 3 4 5 
```
