/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.ArchitectureSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchSymbolCompiler;
import de.monticore.lang.monticar.cnnarch.generator.DataPathConfigParser;
import de.monticore.lang.monticar.cnnarch.generator.LayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CNNArch2MxNet extends CNNArchGenerator {

    CMakeConfig cMakeConfig = new CMakeConfig("");

    public CNNArch2MxNet() {
        architectureSupportChecker = new CNNArch2MxNetArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2MxNetLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public List<FileContent> generateStrings(TaggingResolver var1, ArchitectureSymbol architecture){
        if(architecture != null && architecture.getFullName() != null) {
            cMakeConfig.getCMakeListsViewModel().setCompName(architecture.getFullName().replace('.', '_').replace('[', '_').replace(']', '_'));
        }
        // Add cmake dependencies when they are needed
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCmakeLibraryLinkage("mxnet");

        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;
        CNNArch2MxNetTemplateController archTc = new CNNArch2MxNetTemplateController(architecture);

        temp = archTc.process("CNNPredictor", Target.CPP);
        fileContents.add(temp);

        temp = archTc.process("CNNDataCleaner", Target.PYTHON);
        fileContents.add(temp);

        temp = archTc.process("CNNCreator", Target.PYTHON);
        fileContents.add(temp);

        temp = archTc.process("execute", Target.CPP);
        temp.setFileName(temp.getFileName().replace(".h", ""));
        fileContents.add(temp);

        temp = archTc.process("CNNBufferFile", Target.CPP);
        temp.setFileName("CNNBufferFile.h");
        fileContents.add(temp);

        return fileContents;
    }

    public List<FileContent>  generateCMakeContent(String rootModelName) {
        List<FileContent> fileContents = new ArrayList<>();
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName = rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        cMakeConfig = new CMakeConfig(rootModelName);
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCmakeLibraryLinkage("mxnet");

        fileContents.addAll(cMakeConfig.generateCMakeFiles());

        return fileContents;
    }

    public CMakeConfig getCmakeConfig() {
        return this.cMakeConfig;
    }
}
