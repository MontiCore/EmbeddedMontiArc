package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.template.RosCppCMakeListsModel;
import de.monticore.lang.monticar.generator.roscpp.template.RosCppTemplates;
import de.monticore.lang.monticar.generator.roscpp.util.RosInterface;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class LanguageUnitRosCMake {

    public static List<FileContent> generateFiles(EMAComponentInstanceSymbol component, List<RosInterface> rosInterfaces, boolean ros2Mode) {
        String compName = NameHelper.getComponentNameTargetLanguage(component.getFullName());
        String name = NameHelper.getAdapterName(component);
        RosCppCMakeListsModel model = new RosCppCMakeListsModel(name, compName);


        List<String> allPackages = new ArrayList<>();
        if (!ros2Mode) {
            allPackages.add("roscpp");
        } else {
            allPackages.add("rclcpp");
        }

        rosInterfaces.stream()
                .map(RosInterface::getRosConnectionSymbol)
                .map(RosConnectionSymbol::getTopicType)
                .map(Optional::get)
                .map(n -> n.split("/")[0])
                .forEach(allPackages::add);

        List<String> distinctSortedPackages = allPackages.stream()
                .distinct()
                .sorted()
                .collect(Collectors.toList());

        distinctSortedPackages.stream()
                .filter(p -> !p.equals("struct_msgs"))
                .forEach(model::addPackage);

        distinctSortedPackages
                .forEach(model::addInclude);

        distinctSortedPackages
                .forEach(model::addLibrary);

        List<FileContent> result = new ArrayList<>();

        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        fileContent.setFileContent(RosCppTemplates.generateRosCMakeLists(model));
        result.add(fileContent);

        FileContent cppFile = new FileContent();
        cppFile.setFileName(name + ".cpp");
        cppFile.setFileContent("#include \"" + name + ".h\"");
        result.add(cppFile);

        return result;
    }
}
