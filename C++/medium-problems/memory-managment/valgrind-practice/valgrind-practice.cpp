#include <iostream>

struct DataHolder {
	int val;

	DataHolder(int val) : val(val) {}
	~DataHolder() {
		std::cout << "DataHolder with value " << val << " is being destroyed" << std::endl;
	}
};

void UseCMemoryAllocation() {
	std::cout << "Using C-style memory allocation (malloc and free):" << std::endl;

	int* array = (int*)malloc(5 * sizeof(int));
	
	if (array == nullptr) {
		std::cerr << "Memory allocation failed." << std::endl;
		return;
	}

	for (int i = 0; i < 6; ++i) { // i < 6 instead of 5
		array[i] = i + 10;
		std::cout << "array[" << i << "} = " << array[i] << std::endl;
	}

//	free(array);
	std::cout << "Memory allocated with malloc has been freed." <<std::endl;
}

void UseCPPMemoryAllocation() {
	std::cout << "\nUsing C++-style memory allocation (new and delete):" << std::endl;

	DataHolder* data_ptr = new DataHolder(2);

	std::cout << "DataHolder value: " << data_ptr -> val << std::endl;

//	delete data_ptr;
	std::cout << "Memory allocated with new has been deleted." << std::endl;

	DataHolder* data_array = new DataHolder[3]{DataHolder(1), DataHolder(2), DataHolder(3)};

	for (int i = 0; i < 4; ++i) { // i < 4 instead of 3
		std::cout << "DataHolder array[" << i << "] value: " << data_array[i].val << std::endl;
	}

	delete[] data_array;
	std::cout << "Array allocated with new[] has been deleted." << std::endl;
}

int main() {
	UseCMemoryAllocation();
	UseCPPMemoryAllocation();
	return 0;
}
