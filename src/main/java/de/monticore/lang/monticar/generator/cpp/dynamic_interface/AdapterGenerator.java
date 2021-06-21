package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cpp.FileUtil;
import de.monticore.lang.monticar.generator.cpp.dynamic_interface.ProgramInterfaceResolver.SocketInfo;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

/*
    Generates the CPP files necessary to build the 'library_interface' or the 'server_adapter' for the ema component.
*/
public class AdapterGenerator {
    
    HashSet<String> fileDependenciesOwnCpp = new HashSet<>();
    HashSet<String> fileDependenciesSharedCpp = new HashSet<>();
    List<FileContent> files = new ArrayList<>();

    ProgramInterfaceResolver interfaceResolver;


    public AdapterGenerator(
        EMAComponentInstanceSymbol componentSymbol, 
        CMakeConfig cmake,
        String outputName,
        boolean genLibInterface, 
        boolean genServerAdapter
    ) throws SerializationException, IOException {
        if (outputName.length() == 0) outputName = componentSymbol.getName();

        // Read the ProgramInterface from the model
        interfaceResolver = new ProgramInterfaceResolver();
        interfaceResolver.resolve(componentSymbol);
        

        String targetNameWithSerialization = outputName+ "_with_serialization";
        String targetNameLib = outputName+ "_lib";
        String targetNameServer = outputName+ "_server";

        // Generate the serialization code

        HashMap<String, Object> serializationTemplateData = new HashMap<>();

        serializationTemplateData.put("mainModelName", interfaceResolver.componentName);
        serializationTemplateData.put("interfaceDescription", interfaceResolver.progInterfaceString);
        serializationTemplateData.put("portCount", interfaceResolver.programInterface.ports.size());
        serializationTemplateData.put("isSocket", getIsSocketList());
        serializationTemplateData.put("isOutput", getIsOutputList());
        

        new JsonSerializationGenerator(this).generate(serializationTemplateData);
        new BinarySerializationGenerator(this).generate(serializationTemplateData);
        new PortInitGenerator(this).generate(serializationTemplateData);;

        files.add(new FileContent(AllTemplates.generate(AllTemplates.ADAPTER_PROGRAM_H, serializationTemplateData), "program.h"));
        files.add(new FileContent(AllTemplates.generate(AllTemplates.ADAPTER_PROGRAM_CPP, serializationTemplateData), "program.cpp"));

        addFileDependencySharedCpp("buffer.h");
        addFileDependencySharedCpp("buffer.cpp");
        addFileDependencySharedCpp("json.h");
        addFileDependencySharedCpp("json.cpp");
        addFileDependencySharedCpp("printf.h");
        addFileDependencySharedCpp("printf.cpp");
        addFileDependencySharedCpp("err_out.h");
        addFileDependencySharedCpp("err_out.cpp");

        // TODO "intermediate target" with serialization code ?
        cmake.addCMakeCommandEnd("# A static library around the root EMA component with JSON/Binary serialization functions.");
        cmake.addCMakeCommandEnd("add_library("+targetNameWithSerialization+" STATIC program.cpp buffer.cpp json.cpp printf.cpp err_out.cpp)");
        cmake.addCMakeCommandEnd("target_include_directories("+targetNameWithSerialization+" PUBLIC ${INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})");
        //cmake.addCMakeCommandEnd("target_link_libraries("+targetNameWithSerialization+" PUBLIC ${LIBS} -static-libgcc -static-libstdc++)");
        cmake.addCMakeCommandEnd("target_link_libraries("+targetNameWithSerialization+" PRIVATE ${LIBS})");
        cmake.addCMakeCommandEnd("target_compile_features("+targetNameWithSerialization+" PUBLIC cxx_std_11)");
        cmake.addCMakeCommandEnd("set_target_properties("+targetNameWithSerialization+" PROPERTIES LINKER_LANGUAGE CXX POSITION_INDEPENDENT_CODE ON)");
        cmake.addCMakeCommandEnd("");
        cmake.addCMakeCommandEnd("");


        // Generate the files
        if (genLibInterface) {
            addFileDependencySharedCpp("library_interface.h");
            addFileDependencyOwnCpp("library_interface.cpp");

            // CMake
            // create shared lib for "library_interface"
            cmake.addCMakeCommandEnd("# The library-interface based communication adapter (for the hardware_emulator)");
            cmake.addCMakeCommandEnd("add_library("+targetNameLib+" SHARED library_interface.cpp)");
            cmake.addCMakeCommandEnd("target_link_libraries("+targetNameLib+" PUBLIC -static-libgcc -static-libstdc++ "+targetNameWithSerialization+")");
            cmake.addCMakeCommandEnd("set_target_properties("+targetNameLib+" PROPERTIES LINKER_LANGUAGE CXX POSITION_INDEPENDENT_CODE ON)");
            cmake.addCMakeCommandEnd("set_target_properties("+targetNameLib+" PROPERTIES PREFIX \"\")");
            // install shared lib
            cmake.addCMakeCommandEnd("install(TARGETS "+targetNameLib+" DESTINATION $ENV{DLL_DIR})");
            //cmake.addCMakeCommandEnd("export(TARGETS "+outputName+" FILE "+outputName+".cmake)");
            cmake.addCMakeCommandEnd("");
        }
        if (genServerAdapter) {
            addFileDependencySharedCpp("network.h");
            addFileDependencySharedCpp("network.cpp");
            addFileDependencySharedCpp("tcp_protocol.h");
            addFileDependencyOwnCpp("server_adapter.h");
            addFileDependencySharedCpp("standard_err_out.h");
            addFileDependencySharedCpp("standard_err_out.cpp");
            addFileDependencyOwnCpp("server_adapter.cpp");

            // create adapter executable
            cmake.addCMakeCommandEnd("# ServerAdapter for the EMA component");
            cmake.addCMakeCommandEnd("add_executable("+targetNameServer+" server_adapter.cpp network.cpp standard_err_out.cpp)");
            cmake.addCMakeCommandEnd("target_link_libraries("+targetNameServer+" PUBLIC "+targetNameWithSerialization+")");
            cmake.addCMakeCommandEnd("if (CMAKE_HOST_WIN32)");
            cmake.addCMakeCommandEnd("    target_link_libraries("+targetNameServer+" PUBLIC ws2_32)");
            cmake.addCMakeCommandEnd("endif()");
            cmake.addCMakeCommandEnd("");
        }

        // Add the common file dependencies
        for (String f : fileDependenciesOwnCpp) {
            files.add(FileUtil.getResourceAsFile("/template/adapters/"+f, f));
        }
        for (String f : fileDependenciesSharedCpp) {
            files.add(FileUtil.getResourceAsFile("/shared_cpp/"+f, f));
        }

    }

    String getIsSocketList() {
        String isSocketList = "";
        boolean first = true;
        for (PortInformation portInfo : interfaceResolver.programInterface.ports) {
            if (first) first = false; 
            else isSocketList += ", ";

            isSocketList += portInfo.port_type == PortType.SOCKET ? "true" : "false";
        }
        return isSocketList;
    }
    String getIsOutputList() {
        String isOutputList = "";
        boolean first = true;
        for (PortInformation portInfo : interfaceResolver.programInterface.ports) {
            if (first) first = false; 
            else isOutputList += ", ";

            isOutputList += portInfo.isOutput() ? "true" : "false";
        }
        return isOutputList;
    }

    public List<FileContent> getFiles() {
        return files;
    }

    public void addFileDependencyOwnCpp(String name) {
        fileDependenciesOwnCpp.add(name);
    }

    public void addFileDependencySharedCpp(String name) {
        fileDependenciesSharedCpp.add(name);
    }
    
    

}
