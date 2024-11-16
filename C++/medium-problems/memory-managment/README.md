# Memory Managment

Welcome to the **memory management** directory! In this directory, we will be
covering the basic and intermediate concepts of memory management in C++.

## Requrements

- C++ Compiler (e.g., `g++` or `clang`)
- Valgrind

## What is Valgrind?

Valgrind is a tool used to debug incorrect memory usage.

## How to Install Valgrind

```
sudo apt install valgrind
```

## How to Use Valgrind?

### Compile Your Program

```
g++ myprogram.cpp -o myprogram
```

### Run Valgrind on Your Compiled Code

```
valgrind myprogram
```

### Example Valgrind Output

Here is an example output of Valgrind from malloc-practice from the easy
directory.

```
valgrind ./malloc-practice 
==10964== Memcheck, a memory error detector
==10964== Copyright (C) 2002-2022, and GNU GPL'd, by Julian Seward et al.
==10964== Using Valgrind-3.22.0 and LibVEX; rerun with -h for copyright info
==10964== Command: ./malloc-practice
==10964== 
Enter the number of elements for the array: 5
Enter 5 integers:
1
2
3
4
5
You entered: 1 2 3 4 5 
==10964== 
==10964== HEAP SUMMARY:
==10964==     in use at exit: 0 bytes in 0 blocks
==10964==   total heap usage: 4 allocs, 4 frees, 75,796 bytes allocated
==10964== 
==10964== All heap blocks were freed -- no leaks are possible
==10964== 
==10964== For lists of detected and suppressed errors, rerun with: -s
==10964== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)
```

