#include "MyArray.h"

template<typename T>
void MyArray<T>::print()
{
    for (int i = 0; i < m_length; ++i)
        std::cout << m_data[i] << " ";
    std::cout << std::endl;
}

int main()
{
    MyArray<char> my_array(10);

    for (int i = 0; i < my_array.getLength(); ++i)
        my_array[i] = i + 65;

    my_array.print();

    return 0;   
}