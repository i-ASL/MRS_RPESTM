#include <iostream>
#include <cassert>

template <typename T>
class MyArray
{
private:
    int m_length;
    T* m_data;
public:
    MyArray(int length)
    {
        m_data = new T[length];
        m_length = length;
    }

    ~MyArray()
    {
        reset();
    }

    void reset()
    {
        delete[] m_data;
        m_data = nullptr;
        m_length = 0;
    }

    T& operator [] (int index)
    {
        assert (index >= 0 && index < m_length);
        return m_data[index];
    }

    int getLength()
    {
        return m_length;
    }

    void print();
};
