/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;

import de.monticore.lang.monticar.cnnarch.generator.DataPathConfigParser;
import de.monticore.lang.monticar.cnnarch.generator.Target;

import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CNNArch2Caffe2 extends CNNArchGenerator {
    CMakeConfig cMakeConfig = new CMakeConfig("");

    public CNNArch2Caffe2() {
        architectureSupportChecker = new CNNArch2Caffe2ArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2Caffe2LayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ArchitectureSymbol architecture){
        if(architecture != null && architecture.getFullName() != null) {
            cMakeConfig.getCMakeListsViewModel().setCompName(architecture.getFullName().replace('.', '_').replace('[', '_').replace(']', '_'));
        }
        // Add cmake dependencies when they are needed
        addCMakeDependencies();

        List<FileContent> fileContents = new ArrayList<>();
        CNNArchTemplateController archTc = new CNNArchTemplateController(architecture);
        FileContent temp;

        temp = archTc.process("CNNPredictor", Target.CPP);
        fileContents.add(temp);

        temp = archTc.process("CNNCreator", Target.PYTHON);
        fileContents.add(temp);

        temp = archTc.process("CNNDataCleaner", Target.PYTHON);
        fileContents.add(temp);

        temp = archTc.process("execute", Target.CPP);
        temp.setFileName(temp.getFileName().replace(".h", ""));
        fileContents.add(temp);

        return fileContents;
    }

    private void addCMakeDependencies() {
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addModuleDependency(new CMakeFindModule("Caffe2", true));
        cMakeConfig.addCMakeCommand("set(LIBS ${LIBS} -lprotobuf -lglog -lgflags)");
        cMakeConfig.addCMakeCommand("find_package(CUDA)" + "\n");
        //Needed since CUDA cannot be found correctly (including CUDA_curand_LIBRARY)

        cMakeConfig.addCMakeCommand("if(CUDA_FOUND)" + "\n"
                + "  set(LIBS ${LIBS} caffe2 caffe2_gpu)" + "\n"
                + "  set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})" + "\n"
                + "  set(LIBS ${LIBS} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})" + "\n"
                + "else()" + "\n"
                + "  set(LIBS ${LIBS} caffe2)" + "\n"
                + "endif()");
    }

    public List<FileContent> generateCMakeContent(String rootModelName) {
        List<FileContent> fileContents = new ArrayList<>();
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        cMakeConfig = new CMakeConfig(rootModelName);
        addCMakeDependencies();

        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContents.add(fileContent);
        }
        return fileContents;
    }

    public CMakeConfig getCmakeConfig() {
        return this.cMakeConfig;
    }
}
