package de.monticore.lang.monticar.cnnarch.pytorchgenerator;


import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.ArrayList;
import java.util.List;

public class CNNArch2PyTorch extends CNNArchGenerator {
    private CMakeConfig cMakeConfig = new CMakeConfig("");

    public CNNArch2PyTorch() {
        architectureSupportChecker = new CNNArch2PyTorchArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2PyTorchLayerSupportChecker();
    }
    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ArchitectureSymbol architecture){
        if(architecture != null && architecture.getFullName() != null) {
            cMakeConfig.getCMakeListsViewModel().setCompName(architecture.getFullName().replace('.', '_').replace('[', '_').replace(']', '_'));
        }
        List<FileContent> fileContents = compileFileContentMap(architecture);

        return fileContents;
    }
    private List<FileContent> compilePythonFiles(CNNArch2PyTorchTemplateController controller, ArchitectureSymbol architecture) {
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;

        //temp = controller.process("CNNCreator", Target.PYTHON);
        //fileContents.add(temp);

        temp = controller.process("CNNNet", Target.PYTHON);
        fileContents.add(temp);

        //if (architecture.getDataPath() != null) {
        //  temp = controller.process("CNNDataLoader", Target.PYTHON);
        //fileContents.add(temp);
        //}

        return fileContents;
    }

    private List<FileContent> compileCppFiles(CNNArch2PyTorchTemplateController controller) {
        // Add cmake dependencies when they are needed
        addCMakeDependencies();
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContents.add(temp);

        temp = controller.process("execute", Target.CPP);
        fileContents.add(new FileContent(temp.getFileContent(), temp.getFileName().replace(".h", "")));

        return fileContents;
    }

    private List<FileContent> compileFileContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new PyTorchTemplateConfiguration();

        List<FileContent> fileContents = new ArrayList<>();
        CNNArch2PyTorchTemplateController archTc = new CNNArch2PyTorchTemplateController(
                architecture, templateConfiguration);

        fileContents.addAll(compilePythonFiles(archTc, architecture));
        fileContents.addAll(compileCppFiles(archTc));

        return fileContents;
    }

    public List<FileContent> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        cMakeConfig.getCMakeListsViewModel().setCompName(rootModelName);

        addCMakeDependencies();

        List<FileContent> fileContents = new ArrayList<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContents.add(fileContent);
        }
        return fileContents;
    }

    private void addCMakeDependencies() {
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCMakeCommand("find_package (OpenCV 4.0.0 REQUIRED)");
        cMakeConfig.addCMakeCommand("include_directories (${OpenCV_INCLUDE_DIRS})");
        cMakeConfig.addCMakeCommand("find_package(Torch REQUIRED)");
        cMakeConfig.addCMakeCommand("set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}\")");
        cMakeConfig.addCMakeCommand("set(TORCH_INCLUDE_DIRS ${TORCH_INSTALL_PREFIX}/include ${TORCH_INSTALL_PREFIX}/include/torch/csrc/api/include)");
    }

    @Override
    public CMakeConfig getCmakeConfig() {
        return cMakeConfig;
    }
}
