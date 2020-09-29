#include "de_rwth_montisim_hardware_emulator_CppBridge.h"
#include "jni_interface.h"
#include "software_simulator_manager.h"






/*
    Implementations of the JNI functions that are called from within the RMIModelServer
*/





/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getVersion
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getVersion
(JNIEnv* jni, jclass) {
    return jni->NewStringUTF(VERSION);
}

/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    enableJavaLogging
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_enableJavaLogging
(JNIEnv*, jclass) {
    // TODO
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    initManager
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_initManager
(JNIEnv* jni, jclass, jstring string1) {
    try {
        auto config = jni->GetStringUTFChars(string1, 0);
        SoftwareSimulatorManager::instance.init(json::parse(config));
        jni->ReleaseStringUTFChars(string1, config);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    queryManager
 * Signature: (Ljava/lang/String;)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_queryManager
(JNIEnv* jni, jclass, jstring string) {
    try {
        auto message = jni->GetStringUTFChars(string, 0);
        auto res = SoftwareSimulatorManager::instance.query(json::parse(message));
        jni->ReleaseStringUTFChars(string, message);
        return jni->NewStringUTF(res.dump().c_str());
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    allocSimulator
 * Signature: (Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_allocSimulator
(JNIEnv* jni, jclass, jstring string) {
    try {
        auto config = jni->GetStringUTFChars(string, 0);
        auto res = SoftwareSimulatorManager::instance.alloc_simulator(json::parse(config));
        jni->ReleaseStringUTFChars(string, config);
        return res;
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return -1;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    freeSimulator
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_freeSimulator
(JNIEnv* jni, jclass, jint id) {
    try {
        SoftwareSimulatorManager::instance.free_simulator(id);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getInterface
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getInterface
(JNIEnv* jni, jclass, jint id) {
    try {
        auto res = SoftwareSimulatorManager::instance.simulators[id]->program_interface->get_interface();
        return jni->NewStringUTF(res);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    setPort
 * Signature: (IILjava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_setPort
(JNIEnv* jni, jclass, jint id, jint i, jstring data) {
    try {
        auto data_str = jni->GetStringUTFChars(data, 0);
        SoftwareSimulatorManager::instance.simulators[id]->program_interface->set_port(i, data_str);
        jni->ReleaseStringUTFChars(data, data_str);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getPort
 * Signature: (II)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getPort
(JNIEnv* jni, jclass, jint id, jint i) {
    try {
        auto res = SoftwareSimulatorManager::instance.simulators[id]->program_interface->get_port(i);
        return jni->NewStringUTF(res);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    startTimer
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_startTimer
(JNIEnv* jni, jclass, jint id) {
    try {
        SoftwareSimulatorManager::instance.simulators[id]->start_timer();
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getTimer
 * Signature: (I)Ljava/time/Duration;
 */
JNIEXPORT jlong JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getTimerMicrosec
(JNIEnv* jni, jclass, jint id) {
    try {
        return SoftwareSimulatorManager::instance.simulators[id]->get_timer_micro();
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return 0;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    execute
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_execute
(JNIEnv* jni, jclass, jint id, jdouble delta_sec) {
    try {
        SoftwareSimulatorManager::instance.simulators[id]->program_interface->execute(delta_sec);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

