#include <iostream>
#include <cstring>

/*
Memory leak: dynamically allocated memory is not freed, needs a delete statement
*/

void memoryLeak() {
	int* leakyArray = new int[10];
}

/*
Invalid memory access: writing out of bounds
*/
void invalidMemoryAccess() {
	int* arr = new int[10];
	arr[10] = 42;
	delete[] arr;
}

/*
Use of uninitialized variable
*/
void uninitializedVariable() {
	int var;
	std::cout << "Uninitialized variable output: " << var << std::endl;
}

/*
Tries to delete the same memory twice
*/
void doubleDelete() {
	int* var = new int(5);
	delete var;
	delete var;
}

/*
Poiter overwrite without freeing original allocation
*/
void pointerOverwriteWithoutFreeing() {
	char* ptr = new char[10]; // C-style string allocation
	std::strcpy(ptr, "Hello");
	std::cout << "ptr: " << ptr << std::endl;
	delete[] ptr;
	
	ptr = new char[20];
	
	std::cout << "ptr: " << ptr << std::endl;

	delete[] ptr;
}

/*

*/
void danglingPointer() {
	int* ptr;
	{
		int localVar = 99;
		ptr = &localVar;
	}
	std::cout << "Dangling pointer value: " << *ptr	<< std::endl;
}

int main() {
	memoryLeak();
	invalidMemoryAccess();
	uninitializedVariable();
	doubleDelete();
	pointerOverwriteWithoutFreeing();
	danglingPointer();
	return 0;
}
