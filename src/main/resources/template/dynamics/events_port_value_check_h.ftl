<#-- (c) https://github.com/MontiCore/monticore -->

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

    EqualsTest(){;}
    EqualsTest(T val, T* rVal){
        value = val;
        refValue = rVal;
    }
    bool check(){
        return value == *refValue;
    }
};

template <typename T>
class GreaterTest: public EqualsTest<T>{
public:
    GreaterTest(T val, T* rVal) : EqualsTest<T>(val, rVal) {;}
    bool check(){
        return *(this->refValue) > this->value;
    }
};

template <typename T>
class GreaterEqualsTest: public EqualsTest<T>{
public:
    GreaterEqualsTest(T val, T* rVal) : EqualsTest<T>(val, rVal) {;}
    bool check(){
        return *(this->refValue) >= this->value;
    }
};

template <typename T>
class LowerTest: public EqualsTest<T>{
public:
    LowerTest(T val, T* rVal) : EqualsTest<T>(val, rVal) {;}
    bool check(){
        return *(this->refValue) < this->value;
    }
};

template <typename T>
class LowerEqualsTest: public EqualsTest<T>{
public:
    LowerEqualsTest(T val, T* rVal) : EqualsTest<T>(val, rVal) {;}
    bool check(){
        return *(this->refValue) <= this->value;
    }
};

template <typename T>
class NotEqualsTest: public EqualsTest<T>{
public:
    NotEqualsTest(T val, T* rVal) : EqualsTest<T>(val, rVal) {;}
    bool check(){
        return *(this->refValue) != this->value;
    }
};

template <typename T>
class RangeTest: public TestsInterface<T>{
protected:
    T lowerValue;
    T upperValue;
    T* refValue;
public:
    RangeTest(T lVal, T uVal, T* rVal){
        lowerValue = lVal;
        upperValue = uVal;
        refValue = rVal;
    }
    bool check(){
        return (lowerValue <= *refValue) && (*refValue <= upperValue);
    }
};




template < typename T, std::size_t N >
class PortValueCheck {
    T* portReference;
    T valueHistory[N];
    TestsInterface<T>* checkers[N];

    bool isChecked = false;
    bool checkedValue = false;

    bool isNreached = false;
    int nCounter = 0;
public:

    void setPortReference(T* ref){
        this->portReference = ref;
    }

    void set_Test_Equals(int index, T value){
        checkers[index] = new EqualsTest<T>(value, &valueHistory[index]);
    }

    void set_Test_Greater(int index, T value){
        checkers[index] = new GreaterTest<T>(value, &valueHistory[index]);
    }

    void set_Test_GreaterEquals(int index, T value){
        checkers[index] = new GreaterEqualsTest<T>(value, &valueHistory[index]);
    }

    void set_Test_Lower(int index, T value){
        checkers[index] = new LowerTest<T>(value, &valueHistory[index]);
    }

    void set_Test_LowerEquals(int index, T value){
        checkers[index] = new LowerEqualsTest<T>(value, &valueHistory[index]);
    }

    void set_Test_NotEquals(int index, T value){
        checkers[index] = new NotEqualsTest<T>(value, &valueHistory[index]);
    }

    void set_Test_Range(int index, T lower, T upper){
        checkers[index] = new RangeTest<T>(lower, upper, &valueHistory[index]);
    }



    void next(){
        if(N == 0){
            return;
        }
        for(int i = 0; i < (N-1); ++i){
            valueHistory[i] = valueHistory[i+1];
        }
        valueHistory[N-1] = *portReference;
        isChecked = false;

        if(isNreached){
            return;
        }
        if(!isNreached && nCounter >= (N-1)){
            isNreached = true;
        }else{
            ++nCounter;
        }

    }

    bool check(){
        if(N == 0){
            return false;
        }
        if(!isNreached){
            return false;
        }

        if(isChecked && (valueHistory[N-1] == *portReference)){
            return checkedValue;
        }

        valueHistory[N-1] = *portReference;
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
