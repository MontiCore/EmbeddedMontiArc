/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.*;
import de.monticore.lang.monticar.generator.roscpp.template.RosAdapterModel;
import de.monticore.lang.monticar.generator.roscpp.template.RosCppCMakeListsModel;
import de.monticore.lang.monticar.generator.roscpp.template.RosCppTemplates;
import de.monticore.lang.monticar.generator.roscpp.util.RosInterface;
import de.monticore.lang.monticar.generator.roscpp.util.RosPublisher;
import de.monticore.lang.monticar.generator.roscpp.util.RosSubscriber;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class GeneratorRosCpp {

    private String generationTargetPath;
    private boolean generateCMake = false;
    private boolean ros2Mode = false;

    public boolean isRos2Mode() {
        return ros2Mode;
    }

    public void setRos2Mode(boolean ros2Mode) {
        this.ros2Mode = ros2Mode;
    }

    public void setGenerateCMake(boolean generateCMake) {
        this.generateCMake = generateCMake;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath.endsWith("/") ? generationTargetPath : generationTargetPath + "/";
    }

    public List<File> generateFiles(EMAComponentInstanceSymbol component, TaggingResolver symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(component);

        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {
            files.add(PrinterHelper.generateFile(fileContent, getGenerationTargetPath()));
        }

        return files;
    }

    public List<FileContent> generateStrings(EMAComponentInstanceSymbol component) {
        List<FileContent> fileContents = new ArrayList<>();
        fileContents.addAll(generateRosAdapter(component));
        fileContents.stream()
                .filter(fc -> fc.getFileName().endsWith(".h"))
                .forEach(fc -> fc.setFileContent(FormatHelper.fixIndentation(fc.getFileContent())));
        return fileContents;
    }

    private List<FileContent> generateRosAdapter(EMAComponentInstanceSymbol component) {
        List<FileContent> res = new ArrayList<>();

        if (TagHelper.rosConnectionsValid(component, ros2Mode)) {
            List<EMAPortSymbol> rosPorts = component.getPortInstanceList().stream()
                    .filter(EMAPortSymbol::isRosPort)
                    .collect(Collectors.toList());

            List<RosSubscriber> rosSubscribers = rosPorts.stream()
                    .filter(EMAPortSymbol::isIncoming)
                    .map(RosSubscriber::new)
                    .collect(Collectors.toList());

            List<RosPublisher> rosPublishers = rosPorts.stream()
                    .filter(EMAPortSymbol::isOutgoing)
                    .map(RosPublisher::new)
                    .collect(Collectors.toList());

            List<RosInterface> rosInterfaces = new ArrayList<>();
            rosInterfaces.addAll(rosSubscribers);
            rosInterfaces.addAll(rosPublishers);

            if (generateCMake) {
                res.addAll(generateCMakeFiles(component, rosInterfaces, isRos2Mode()));
            }

            RosAdapterModel model = new RosAdapterModel(ros2Mode, NameHelper.getComponentNameTargetLanguage(component.getFullName()));
            model.addPublishers(rosPublishers);
            model.addSubscribers(rosSubscribers);
            model.addGenerics(GenericsHelper.getGenericsDefinition(component));
            if(ros2Mode) {
                rosSubscribers.forEach(s -> model.addInclude(s.getRos2Include()));
                rosPublishers.forEach(s -> model.addInclude(s.getRos2Include()));
            }else{
                rosSubscribers.forEach(s -> model.addInclude(s.getRosInclude()));
                rosPublishers.forEach(s -> model.addInclude(s.getRosInclude()));
            }
            String s = RosCppTemplates.generateRosAdapter(model);
            res.add(new FileContent("RosAdapter_" + NameHelper.getComponentNameTargetLanguage(component.getFullName()) + ".h",s));
        }
        return res;
    }

    public List<FileContent> generateCMakeFiles(EMAComponentInstanceSymbol component, List<RosInterface> rosInterfaces, boolean ros2Mode) {
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
        if(!ros2Mode){
            model.addExcludeFindPackage("struct_msgs");
        }

        List<FileContent> result = new ArrayList<>();
        result.add(new FileContent("CMakeLists.txt", RosCppTemplates.generateRosCMakeLists(model)));
        result.add(new FileContent(name + ".cpp","/* (c) https://github.com/MontiCore/monticore */\n#include \"" + name + ".h\""));
        return result;
    }

}
