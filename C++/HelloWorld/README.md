# Hello World Programs

Welcome to the **Hello World Programs** directory! This directory contains the
basic "Hello World" examples in various different programming languages to
demonstrate the differences between popular programming languages.

## Overview

Each of the files in this directory prints "Hello, World!" to the console. This
is typically the first program written when learning a new programming
language, and it is a great way to demonstrate how many programming languages
are very different from each other.

## Files included

- `HelloWorld.c` - C program
- `HelloWorld.cpp` - C++ program
- `HelloWorld.java` - Java program
- `HelloWorld.py` - Python program

## Prerequisites

To run each of these files, you will need to have the respective compilers
installed on your system. The C, C++, and Python compilers should have been
installed during the onboarding proccess. See EviromentSetup, in the CSMWiki
for more details on setting up previous mentioned compilers. 

If you would like to run the Java, it will require a little bit more effort and
is **NOT** required to setup, I just included it because many programmers are
familar with Java.

## How to Run Each Program

### C Program

Compile the C program with the following terminal command:

```
gcc HelloWorld.c -o HelloWorld
```

To run the now compiled program, use the following terminal command:

```
./HelloWorld
```

### C++ Program

Compile the C++ program with the following terminal command:

```
g++ HelloWorld.cpp -o HelloWorld
```

To run the now compiled program, use the following terminal command:

```
./HelloWorld
```

### Python Program

Python scripts do not need to be compiled, and are run directly from the `.py`
file with the following command:

```
python3 HelloWorld.py
```

### Java Program

**THIS IS NOT REQUIRED**  

To set up java you will have to install the Java Development Kit (JDK) with the
following terminal command: 

```
sudo apt install openjdk-17-jdk
```

After downloading the Java 17 JDK, you will be able to compile the Java file
with the following terminal command:

```
javac HelloWorld.java
```

To run the nw comiled program, use the following terminal command:

```
java HelloWorld
```

If you no do not want to keep the Java JDK installed on your machine, use the
following terminal command:

```
sudo apt remove openjdk-17-jdk
```
