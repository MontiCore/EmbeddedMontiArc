package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LanguageUnitRosCMake {
    private String cmakeTemplate =
            "cmake_minimum_required(VERSION 3.5)\n" +
                    "project (<name>)\n" +
                    "\n" +
                    "<packages>\n" +
                    "\n" +
                    "add_library(<name> <name>.h)\n" +
                    "set_target_properties(<name> PROPERTIES LINKER_LANGUAGE CXX)\n" +
                    "target_link_libraries(<name> <compName> <libraries>)\n" +
                    "target_include_directories(<name> PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} <include_dirs>)\n" +
                    "<dependency>\n" +
                    "\n" +
                    "export(TARGETS <name> FILE <name>.cmake)";

    FileContent generate(EMAComponentInstanceSymbol componentInstanceSymbol, List<String> additionalPackages, boolean isRos2Mode) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");

        List<String> allPackages = new ArrayList<>();
        allPackages.addAll(additionalPackages);
        if(!isRos2Mode) {
            allPackages.add("roscpp");
        }else{
            allPackages.add("rclcpp");
        }

        List<String> distinctSortedPackages = allPackages.stream()
                .distinct()
                .sorted()
                .collect(Collectors.toList());

        String compName = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        String name = NameHelper.getAdapterName(componentInstanceSymbol);
        String packages = distinctSortedPackages.stream()
                .filter(p -> !p.equals("struct_msgs"))
                .map(p -> "find_package(" + p + " REQUIRED)")
                .collect(Collectors.joining("\n"));

        String libraries = distinctSortedPackages.stream()
                .map(p -> "${" + p + "_LIBRARIES}")
                .collect(Collectors.joining(" "));

        String include_dirs = distinctSortedPackages.stream()
                .map(p -> "${" + p + "_INCLUDE_DIRS}")
                .collect(Collectors.joining(" "));

        String dependency = "";
        if (distinctSortedPackages.stream().filter(pack -> pack.startsWith("struct_msgs")).count() > 0) {
            dependency = "add_dependencies(<name> struct_msgs_generate_messages)".replace("<name>", name);
        }


        String content = cmakeTemplate
                .replace("<name>", name)
                .replace("<compName>", compName)
                .replace("<packages>", packages)
                .replace("<libraries>", libraries)
                .replace("<include_dirs>", include_dirs)
                .replace("<dependency>", dependency);

        fileContent.setFileContent(content);
        return fileContent;
    }

}
