/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "jni_interface.h"

#include <exception>
#include <iostream>
#include "utility/utility.h"
#include "simulator/software_simulator.h"
#include "err_out.h"

//Called by Java when the Library is loaded,
//Used to initialized the global references to standard objects
JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv* jni;
    int getEnvStat = vm->GetEnv((void**)&jni, JNI_VERSION_1_6);
    if (getEnvStat == JNI_EDETACHED) {
        std::cout << "GetEnv: not attached" << std::endl;
        /*if (vm->AttachCurrentThread((void**)&jni, NULL) != 0) {
            std::cout << "Failed to attach" << std::endl;
        }*/
    }
    else if (getEnvStat == JNI_OK) {
        //
    }
    else if (getEnvStat == JNI_EVERSION) {
        std::cout << "GetEnv: version not supported" << std::endl;
    }
    JNIEnvironment::instance.init(jni);

    ERR_OUT_set_functions(SoftwareSimulator::ERR_OUT_throw_error, SoftwareSimulator::ERR_OUT_print_cout, SoftwareSimulator::ERR_OUT_print_cerr);
    //std::cout << "JNI_OnLoad() called" << std::endl;
    //TODO DetachCurrentThread
    return JNI_VERSION_1_6;
}

//Called by Java when the Library is unloaded,
//Used to free the global references to standard objects
JNIEXPORT void JNICALL
JNI_OnUnload(JavaVM* vm, void* reserved) {
    JNIEnv* jni;
    int getEnvStat = vm->GetEnv((void**)&jni, JNI_VERSION_1_6);
    if (getEnvStat == JNI_EDETACHED) {
        std::cout << "GetEnv: not attached" << std::endl;
        /*if (vm->AttachCurrentThread((void**)&jni, NULL) != 0) {
            std::cout << "Failed to attach" << std::endl;
        }*/
    }
    else if (getEnvStat == JNI_OK) {
        //
    }
    else if (getEnvStat == JNI_EVERSION) {
        std::cout << "GetEnv: version not supported" << std::endl;
    }
    JNIEnvironment::instance.drop(jni);
    //std::cout << "JNI_OnUnload() called" << std::endl;
    //TODO DetachCurrentThread
    //vm->DetachCurrentThread();
}

JNIEnvironment JNIEnvironment::instance;

struct JNIOutput : public Log::OStreamTarget {
    JNIEnv* jni;
    jclass cppbridge_class;
    jmethodID cppbridge_log_method;

    JNIOutput(JNIEnv* jni, jclass cppbridge_class, jmethodID cppbridge_log_method) : jni(jni), cppbridge_class(cppbridge_class), cppbridge_log_method(cppbridge_log_method) {}

    void print(const char* str, ConsoleColor color, const char* name) {
        jni->CallStaticVoidMethod(cppbridge_class, cppbridge_log_method, jni->NewStringUTF(str));
    }
};

void JNIEnvironment::init(JNIEnv* jni)
{
    resolve_class(jni, java_lang_double_class, "java/lang/Double");
    resolve_method(jni, java_lang_double_class, java_lang_double_constructor, CONSTRUCTOR_NAME, "(D)V");
    resolve_method(jni, java_lang_double_class, java_lang_double_doubleValue_method, "doubleValue", "()D");

    resolve_class(jni, java_lang_int_class, "java/lang/Integer");
    resolve_method(jni, java_lang_int_class, java_lang_int_constructor, CONSTRUCTOR_NAME, "(I)V");
    resolve_method(jni, java_lang_int_class, java_lang_int_intValue_method, "intValue", "()I");

    resolve_class(jni, cppbridge_class, "de/rwth/montisim/hardware_emulator/CppBridge");
    resolve_static_method(jni, cppbridge_class, cppbridge_log_method, "log", "(Ljava/lang/String;)V");

    Log::output_stream = std::make_unique<JNIOutput>(jni, cppbridge_class, cppbridge_log_method);
}

void JNIEnvironment::drop(JNIEnv* jni)
{
    jni->DeleteGlobalRef(java_lang_double_class);
    jni->DeleteGlobalRef(java_lang_int_class);
}


void JNIEnvironment::resolve_class(JNIEnv* jni, jclass& global_class_ref, const char* class_name)
{
    jclass local_class = jni->FindClass(class_name);
    if (local_class == nullptr) 
        throw_error(Error::jni_error(std::string("Could not get Java Class: ") + class_name));
    global_class_ref = (jclass)jni->NewGlobalRef(local_class);
}

void JNIEnvironment::resolve_method(JNIEnv* jni, jclass class_ref, jmethodID& method, const char* method_name, const char* method_signature)
{
    method = jni->GetMethodID(class_ref, method_name, method_signature);
    if (method == nullptr)
        throw_error(Error::jni_error(std::string("Could not get Java Method: ") + method_name));
}

void JNIEnvironment::resolve_static_method(JNIEnv* jni, jclass class_ref, jmethodID& method, const char* method_name, const char* method_signature)
{
    method = jni->GetStaticMethodID(class_ref, method_name, method_signature);
    if (method == nullptr)
        throw_error(Error::jni_error(std::string("Could not get Static Java Method: ") + method_name));
}


jint JNIEnvironment::throw_exception(JNIEnv* env, const char* message)
{
    //std::cout << "Native exception: " << message << std::endl;
    jclass exClass = env->FindClass("de/rwth/montisim/hardware_emulator/HardwareEmulatorException");
    if (exClass != NULL) {
        return env->ThrowNew(exClass, message);
    }
    exClass = env->FindClass("java/lang/Exception");
    if (exClass == NULL) {
        throw SystemException("Could not resolve java.lang.Exception class to throw exception...");
    }
    return env->ThrowNew(exClass, message);
}