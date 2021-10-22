/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "de_rwth_montisim_hardware_emulator_CppBridge.h"
#include "jni_interface.h"
#include "simulator/software_simulator_manager.h"
#include "buffer.h"


DynamicBuffer buff; // This buffer is shared between all "setPortBinary" calls => native calls are not thread safe anymore


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
        auto res = SoftwareSimulatorManager::instance.simulators[id]->program_functions->get_interface();
        return jni->NewStringUTF(res);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}


/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    setPortJson
 * Signature: (IILjava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_setPortJson
(JNIEnv* jni, jclass, jint id, jint i, jstring data) {
    try {
        auto data_str = jni->GetStringUTFChars(data, 0);
        SoftwareSimulatorManager::instance.simulators[id]->program_functions->set_port(i, data_str, 1);
        jni->ReleaseStringUTFChars(data, data_str);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getPortJson
 * Signature: (II)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getPortJson
(JNIEnv* jni, jclass, jint id, jint i) {
    try {
        auto res = SoftwareSimulatorManager::instance.simulators[id]->program_functions->get_port(i, 1);
        return jni->NewStringUTF(res);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}

/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    setPortBinary
 * Signature: (II[B)V
 */
JNIEXPORT void JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_setPortBinary
(JNIEnv* jni, jclass, jint id, jint i, jbyteArray arr) {
    try {
        buff.reset();
        BinaryWriter bw = { buff };
        auto length = jni->GetArrayLength(arr);
        bw.write_u32(length);
        buff.reserve(length + 4);
        jni->GetByteArrayRegion(arr, 0, length, (jbyte*)(buff.get_buffer() + 4));
        SoftwareSimulatorManager::instance.simulators[id]->program_functions->set_port(i, buff.get_buffer(), 0);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_montisim_hardware_emulator_CppBridge
 * Method:    getPortBinary
 * Signature: (II)[B
 */
JNIEXPORT jbyteArray JNICALL Java_de_rwth_montisim_hardware_1emulator_CppBridge_getPortBinary
(JNIEnv* jni, jclass, jint id, jint i) {
    try {
        auto res = SoftwareSimulatorManager::instance.simulators[id]->program_functions->get_port(i, 0);
        BinaryReader br = {res, 4};
        auto len = br.read_u32();
        auto byte_arr = jni->NewByteArray(len);
        jni->SetByteArrayRegion(byte_arr, 0, len, (const jbyte*)(res+4));
        return byte_arr;
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
        SoftwareSimulatorManager::instance.simulators[id]->program_functions->execute(delta_sec);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

