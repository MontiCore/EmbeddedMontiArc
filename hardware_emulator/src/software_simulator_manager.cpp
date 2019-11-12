/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "software_simulator_manager.h"
#include "de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge.h"

#include "hardware_emulator.h"
#include "direct_software_simulator.h"
#include "utility/config.h"
#include <thread>

SoftwareSimulatorManager SoftwareSimulatorManager::instance;


void SoftwareSimulatorManager::init( const char *config) {
    available_threads = std::thread::hardware_concurrency();
    available_softwares_string = "";
    
    //path = fs::current_path();
    softwares_folder = FS::current_directory();
    MessageParser parser( config );
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "softwares_folder" ) )
            //path = fs::canonical( parser.get_string() );
            softwares_folder = FS::Directory(parser.get_string()).canonical();
        else
            parser.unknown();
    }
    
    Log::info << Log::tag << "softwares_folder: " << softwares_folder.to_string() << "\n";
    
    for ( const auto &file : softwares_folder.get_files()) {
        //std::cout << p << "\t" << p.filename() << "\t" << ext/*p.extension()*/ << std::endl;
        auto extension = file.get_extension();
        if (extension.compare( ".so" ) == 0 || extension.compare( ".dll" ) == 0 ) {
            available_softwares.emplace_back( file );
            if ( available_softwares_string.size() > 0 )
                available_softwares_string += ';';
            available_softwares_string += file.get_name();
        }
    }
    simulators.clear();
    simulator_count = 0;
}

int SoftwareSimulatorManager::alloc_simulator( const char *config ) {
    std::unique_ptr<SoftwareSimulator> simulator;
    std::string simulator_mode;
    
    //Check simulator mode
    MessageParser parser(config);
    while (parser.has_next()) {
        if (parser.is_cmd("mode"))
            simulator_mode = parser.get_string();
    }

    if (simulator_mode.compare("direct") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new DirectSoftwareSimulator());
    }
    else if (simulator_mode.compare("emu") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new HardwareEmulator());
    }
    else
        throw_error("SoftwareSimulatorManager: Config exception: unknown simulator mode: " + simulator_mode);

    //Initialize simulator
    simulator->init(*this, config);

    //Get id and store simulator
    if ( simulator_count >= simulators.size() )
        simulators.resize( simulators.size() + 5 );
    for ( auto i : irange( (int) simulators.size() ) ) {
        if ( !simulators[i] ) {
            simulators[i] = std::move(simulator);
            simulator_count++;
            Log::info << Log::tag << "Emulator allocated with id " << i << "\n"; //TODO log
            return i;
        }
    }

    throw_error("Unknown Error");
}

void SoftwareSimulatorManager::free_simulator( int id ) {
    Log::info << Log::tag << "Emulator " << id << " freed\n";
    simulator_count--;
    simulators[id].reset();
}


std::string SoftwareSimulatorManager::query( const char *msg ) {
    MessageParser parser( msg );
    MessageBuilder builder;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "get_error_msg" ) )
            builder.add( "error_msg", error_msg );
        else if ( parser.is_cmd( "get_available_autopilots" ) )
            builder.add( "available_autopilots", available_softwares_string );
        else if ( parser.is_cmd( "get_available_threads" ) )
            builder.add( "available_threads", std::to_string( available_threads ) );
        else if ( parser.is_cmd( "get_autopilots_folder" ) )
            builder.add( "softwares_folder", softwares_folder.to_string() );
        else
            parser.unknown();
    }
    return builder.res;
}


/*
    Implementations of the JNI functions that are called from within the RMIModelServer

    add_one_input (Java_simulator_integration_HardwareEmulatorInterface_add_1one_1input)
    must be updated when new data types need to be converted from Java types to C types.
    Same for query_outputs (Java_simulator_integration_HardwareEmulatorInterface_query_1outputs)
    if new data types must be converted from C types to Java types.
*/


/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    initManager
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_initManager
(JNIEnv* jni, jclass, jstring string1) {
    try {
        auto config = jni->GetStringUTFChars(string1, 0);
        SoftwareSimulatorManager::instance.init(config);
        jni->ReleaseStringUTFChars(string1, config);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    allocSimulator
 * Signature: (Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_allocSimulator
(JNIEnv* jni, jclass, jstring string) {
    try {
        auto config = jni->GetStringUTFChars(string, 0);
        auto res = SoftwareSimulatorManager::instance.alloc_simulator(config);
        jni->ReleaseStringUTFChars(string, config);
        return res;
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return -1;
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    freeSimulator
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_freeSimulator
(JNIEnv* jni, jclass, jint id) {
    try {
        SoftwareSimulatorManager::instance.free_simulator(id);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    queryManager
 * Signature: (Ljava/lang/String;)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_queryManager
(JNIEnv* jni, jclass, jstring string) {
    try {
        auto message = jni->GetStringUTFChars(string, 0);
        auto res = SoftwareSimulatorManager::instance.query(message);
        jni->ReleaseStringUTFChars(string, message);
        return jni->NewStringUTF(res.c_str());
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    runCycle
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_runCycle
(JNIEnv* jni, jclass, jint id) {
    try {
        return SoftwareSimulatorManager::instance.simulators[id]->run_cycle_get_time();
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return 0;
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    querySimulator
 * Signature: (ILjava/lang/String;)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_querySimulator
(JNIEnv* jni, jclass, jint id, jstring string) {
    try {
        auto message = jni->GetStringUTFChars(string, 0);
        auto res = SoftwareSimulatorManager::instance.simulators[id]->query(message);
        jni->ReleaseStringUTFChars(string, message);
        return jni->NewStringUTF(res.c_str());
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
    return nullptr;
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    addOneInput
 * Signature: (ILjava/lang/String;Ljava/io/Serializable;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_addOneInput
(JNIEnv* jni, jclass, jint id, jstring key, jobject value) {
    try {
        (*SoftwareSimulatorManager::instance.simulators[id]).add_one_input(jni, key, value);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}

/*
 * Class:     de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_emulator_CppBridge
 * Method:    queryOutputs
 * Signature: (ILjava/util/HashMap;)V
 */
JNIEXPORT void JNICALL Java_de_rwth_monticore_EmbeddedMontiArc_simulators_hardware_1emulator_CppBridge_queryOutputs
(JNIEnv* jni, jclass cls, jint id, jobject opaque_hashmap) {
    try {
        (*SoftwareSimulatorManager::instance.simulators[id]).query_outputs(jni, cls, opaque_hashmap);
    }
    catch (std::exception & e) {
        JNIEnvironment::throw_exception(jni, e.what());
    }
}







