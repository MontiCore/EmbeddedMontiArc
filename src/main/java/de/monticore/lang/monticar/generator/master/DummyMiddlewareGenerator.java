package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class DummyMiddlewareGenerator implements GeneratorImpl {

    private String generationTargetPath;

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {

        List<File> res = new ArrayList<>();
        res.add(FileHelper.generateFile(generationTargetPath, generateCMake(componentInstanceSymbol)));
        res.add(FileHelper.generateFile(generationTargetPath, generateWrapper(componentInstanceSymbol)));
        return res;
    }


    private String dummyWrapperTemplate = "#pragma once\n" +
            "#include \"${compName}.h\"\n" +
            "#include <thread>\n" +
            "#include <chrono>\n" +
            "#include \"IAdapter.h\"\n" +
            "\n" +
            "class ${compName}_DummyWrapper: public IAdapter{\n" +
            "\t${compName}* component;\n" +
            "\n" +
            "public:\n" +
            "\t${compName}_DummyWrapper(){\n" +
            "\n" +
            "\t}\n" +
            "\n" +
            "\tvoid tick(){\n" +
            "\t\tcout << \"Dummy publish data: component.out1 = \"<< component->out1 << endl;\n" +
            "\t}\n" +
            "\t\n" +
            "\tvoid init(${compName}* comp){\n" +
            "\t\tthis->component = comp;\n" +
            "\t\twhile(1){\n" +
            "       \t\t    std::this_thread::sleep_for(std::chrono::seconds(1));\n" +
            "\t\t    component->in2 += 1000;\n" +
            "\t\t}\n" +
            "\t}\n" +
            "\n" +
            "\t\n" +
            "};";

    private FileContent generateWrapper(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        String content = dummyWrapperTemplate
                .replace("${compName}", name);

        FileContent res = new FileContent();
        res.setFileName(name + "_DummyWrapper.h");
        res.setFileContent(content);
        return res;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }


    private String cmakeTemplate =
            "cmake_minimum_required(VERSION 3.5)\n" +
                    "project (${compName}_DummyWrapper)\n" +
                    "\n" +
                    "add_library(${compName}_DummyWrapper ${compName}_DummyWrapper.h)\n" +
                    "set_target_properties(${compName}_DummyWrapper PROPERTIES LINKER_LANGUAGE CXX)\n" +
                    "target_link_libraries(${compName}_DummyWrapper ${compName})\n" +
                    "target_include_directories(${compName}_DummyWrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})\n" +
                    "export(TARGETS ${compName}_DummyWrapper FILE ${compName}_DummyWrapper.cmake)";

    private FileContent generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        FileContent res = new FileContent();
        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        String content = cmakeTemplate
                .replace("${compName}", name);

        res.setFileName("CMakeLists.txt");
        res.setFileContent(content);
        return res;
    }
}
