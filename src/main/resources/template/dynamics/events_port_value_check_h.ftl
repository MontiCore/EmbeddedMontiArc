#ifndef PortValueCheck_h
#define PortValueCheck_h

#include <iostream>
using namespace std;

template <typename T>
class TestsInterface {
public:
    virtual bool check(){
        return false;
    }
};

template <typename T>
class EqualsTest: public TestsInterface<T>{
protected:
    T value;
    T* refValue;
public:
    EqualsTest(T val, T* rVal){
        value = val;
        refValue = rVal;
    }
    bool check(){
        return value == *refValue;
    }
};



template < typename T, std::size_t N >
class PortValueCheck {
    T* portReference;
    T valueHistory[N];
    TestsInterface<T>* checkers[N];

    bool isChecked = false;
    bool checkedValue = false;

public:

    void setPortReference(T* ref){
        this->portReference = ref;
    }

    void setEqualsTest(int index, T initValue, T value){
        checkers[index] = new EqualsTest<T>(value, &valueHistory[index]);
        valueHistory[index] = initValue;
    }

    void next(){
        for(int i = 0; i < (N-1); ++i){
            valueHistory[i] = valueHistory[i+1];
        }
        valueHistory[N-1] = *portReference;
        isChecked = false;
    }

    bool check(){
        if(isChecked){
            return checkedValue;
        }

        isChecked = true;
        for(int i = 0; i < N; ++i){
            if(!checkers[i]->check()){
                checkedValue = false;
                return false;
            }
        }
        checkedValue = true;
        return true;
    }
};

#endif /* PortValueCheck_h */
