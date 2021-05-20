/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include "jni.h"



/*
    Contains the resolved references to java objects and methods, this needs to be performed only once (Per thread).
*/
struct JNIEnvironment {
    static constexpr auto CONSTRUCTOR_NAME = "<init>";
    static JNIEnvironment instance;
    void init(JNIEnv* jni);
    void drop(JNIEnv* jni);

    jclass java_lang_double_class = 0;
    jmethodID java_lang_double_constructor = 0;
    jmethodID java_lang_double_doubleValue_method = 0;

    jclass java_lang_int_class = 0;
    jmethodID java_lang_int_constructor = 0;
    jmethodID java_lang_int_intValue_method = 0;

    
    jclass cppbridge_class = 0;
    jmethodID cppbridge_log_method = 0;


    static void resolve_class(JNIEnv* jni, jclass& global_class_ref, const char* class_name);
    static void resolve_method(JNIEnv* jni, jclass class_ref, jmethodID& method, const char* method_name, const char* method_signature);
    static void resolve_static_method(JNIEnv* jni, jclass class_ref, jmethodID& method, const char* method_name, const char* method_signature);

    static jint throw_exception(JNIEnv* env, const char* message);
};

/*
    The following are used as template parameters to specify how to receive/send specific types from/to Java.
    Specifies the JNI functions for basic types and for arrays.
*/

struct JNITypeDouble {
    using CType = double;
    static inline CType call_type_method(JNIEnv* jni, jobject value) {
        return jni->CallDoubleMethod(value, JNIEnvironment::instance.java_lang_double_doubleValue_method);
    }
    static inline jobject new_object(JNIEnv* jni, CType value) {
        return jni->NewObject(JNIEnvironment::instance.java_lang_double_class, JNIEnvironment::instance.java_lang_double_constructor, (jdouble)value);
    }

    static inline CType* get_array(JNIEnv* jni, jobject value) {
        return jni->GetDoubleArrayElements((jdoubleArray)value, 0);
    }
    static inline void free_array(JNIEnv* jni, jobject value, CType* arr) {
        jni->ReleaseDoubleArrayElements((jdoubleArray)value, arr, JNI_ABORT);
    }
};

struct JNITypeInt {
    using CType = jint;
    static inline CType call_type_method(JNIEnv* jni, jobject value) {
        return jni->CallIntMethod(value, JNIEnvironment::instance.java_lang_int_intValue_method);
    }
    static inline jobject new_object(JNIEnv* jni, CType value) {
        return jni->NewObject(JNIEnvironment::instance.java_lang_int_class, JNIEnvironment::instance.java_lang_int_constructor, (jint)value);
    }
    static inline CType* get_array(JNIEnv* jni, jobject value) {
        return jni->GetIntArrayElements((jintArray)value, 0);
    }
    static inline void free_array(JNIEnv* jni, jobject value, CType* arr) {
        jni->ReleaseIntArrayElements((jintArray)value, arr, JNI_ABORT);
    }
};