#include <iostream>
#include <vector>

using namespace std;

class Sort {
public:
    Sort() {};
    ~Sort() {};

    void insertSort(vector<int>& array);    
    void bubbleSort(vector<int>& array);     
    void heapSort(vector<int>& array);
    void quickSort(vector<int>& array);
    void selectionSort(vector<int>& array);
    void bucketSort(vector<int>& array);
    void mergeSort(vector<int>& array);
    void countingSort(vector<int>& array);

    void displayArray(const vector<int>& array);

// private:
//     vector<int> unorderArray = {12,3,2,5,11,6,1,10,7,2};
};

