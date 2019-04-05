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

public class LanguageUnitRosCMake {

    public static List<FileContent> generateFiles(EMAComponentInstanceSymbol component, List<RosInterface> rosInterfaces, boolean ros2Mode) {
        String compName = NameHelper.getComponentNameTargetLanguage(component.getFullName());
        String name = NameHelper.getAdapterName(component);
        RosCppCMakeListsModel model = new RosCppCMakeListsModel(name, compName);

        List<String> allPackages = new ArrayList<>();
        allPackages.add(ros2Mode ? "rclcpp" : "roscpp");

        rosInterfaces.stream()
                .map(RosInterface::getRosConnectionSymbol)
                .map(RosConnectionSymbol::getTopicType)
                .map(Optional::get)
                .map(n -> n.split("/")[0])
                .forEach(allPackages::add);

        allPackages.forEach(model::addPackage);

        List<FileContent> result = new ArrayList<>();
        result.add(new FileContent("CMakeLists.txt", RosCppTemplates.generateRosCMakeLists(model)));
        result.add(new FileContent(name + ".cpp","#include \"" + name + ".h\""));
        return result;
    }
}
