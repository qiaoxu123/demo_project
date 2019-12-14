#include "sort.h"

void
Sort::insertSort(vector<int>& array) {
    for (int i = 0; i < array.size(); i++) {
        int min = INT32_MAX;
        int index = 0;
        for (int j = i; j < array.size(); j++) {
            if (array[j] < min) {
                min = array[j];
                index = j;
            }
        }
        swap(array[i], array[index]);
        displayArray(array);
    }
}

void 
Sort::bubbleSort(vector<int>& array) {
    for (int i = 0; i < array.size(); i++) {
        for (int j = 0; j < array.size()-1-i; j++) {
            if (array[j] > array[j + 1]) {
                swap(array[j], array[j+1]);
            } 
        }
    }
}

void 
Sort::displayArray(const vector<int>& array) {
    std::cout << "Array = ";
    for (int i = 0;i < array.size(); ++i) {
        std::cout << array[i] << " " ; 
    }
    std::cout << std::endl;
}

int main() {
    vector<int> unorderArray = {12,3,2,5,11,6,1,10,7,2};
    
    Sort obj;
    std::cout << "Init : ";
    obj.displayArray(unorderArray);
    std::cout << "//--------------------------------------------------------" << std::endl;
    std::cout << "End  : ";
    // obj.bubbleSort(unorderArray);   // ok
    obj.insertSort(unorderArray);
    std::cout << "//--------------------------------------------------------" << std::endl;
    obj.displayArray(unorderArray);
    std::cout << std::endl;
    return 0;
}