package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.ClusterHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.FileHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;
import de.monticore.lang.monticar.generator.middleware.impls.GeneratorImpl;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class DistributedTargetGenerator extends CMakeGenerator {
    private Set<String> subDirs = new HashSet<>();


    public DistributedTargetGenerator() {
    }

    @Override
    public void setGenerationTargetPath(String path) {
        super.setGenerationTargetPath(path);
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        Map<PortSymbol, RosConnectionSymbol> resolvedTags = TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        Map<ExpandedComponentInstanceSymbol, GeneratorImpl> generatorMap = new HashMap<>();

        fixComponentInstance(componentInstanceSymbol);

        List<ExpandedComponentInstanceSymbol> clusterSubcomponents = ClusterHelper.getClusterSubcomponents(componentInstanceSymbol);
        if (clusterSubcomponents.size() > 0) {
            clusterSubcomponents.forEach(clusterECIS -> {
                generatorMap.put(clusterECIS, createFullGenerator(clusterECIS.getFullName().replace(".", "_")));
            });
        } else {
            generatorMap.put(componentInstanceSymbol, createFullGenerator(componentInstanceSymbol.getFullName().replace(".", "_")));
        }

        List<File> files = new ArrayList<>();

        subDirs.add("rosMsg");
        for (ExpandedComponentInstanceSymbol comp : generatorMap.keySet()) {
            files.addAll(generatorMap.get(comp).generate(comp, taggingResolver));
            //add empty generator to subDirs so that CMakeLists.txt will be generated correctly
            subDirs.add(comp.getFullName().replace(".", "_"));
        }

        files.add(generateCMake());
        files.add(generateRosMsgGen());

        return files;
    }

    //TODO:refactor, dont always generate
    private File generateRosMsgGen() throws IOException {
        File file = new File(generationTargetPath + "rosMsg/CMakeLists.txt");
        FileUtils.write(file, TemplateHelper.struct_msgsCmakeTemplate);
        return file;
    }

    private GeneratorImpl createFullGenerator(String subdir) {
        MiddlewareGenerator res = new MiddlewareGenerator();
        res.setGenerationTargetPath(generationTargetPath + (subdir.endsWith("/") ? subdir : subdir + "/"));

        this.getGeneratorImpls().forEach(gen -> res.add(gen, this.getImplSubdir(gen)));

        return res;
    }

    private void fixComponentInstance(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        fixRosTopics(componentInstanceSymbol);
    }

    private void fixRosTopics(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        componentInstanceSymbol.getConnectors().stream()
                .filter(connectorSymbol -> connectorSymbol.getSourcePort().isRosPort() && connectorSymbol.getTargetPort().isRosPort())
                .forEach(connectorSymbol -> {
                    if (Objects.equals(connectorSymbol.getSourcePort().getComponentInstance().orElse(null), componentInstanceSymbol)) {
                        //In port of supercomp
                        inferRosConnectionIfPossible(connectorSymbol.getSourcePort(), connectorSymbol.getTargetPort());
                        generateRosConnectionIfPossible(connectorSymbol);
                    } else if (Objects.equals(connectorSymbol.getTargetPort().getComponentInstance().orElse(null), componentInstanceSymbol)) {
                        //out port of supercomp
                        inferRosConnectionIfPossible(connectorSymbol.getTargetPort(), connectorSymbol.getSourcePort());
                        generateRosConnectionIfPossible(connectorSymbol);
                    } else {
                        //In between subcomps
                        generateRosConnectionIfPossible(connectorSymbol);
                    }

                });
    }

    private void generateRosConnectionIfPossible(ConnectorSymbol connectorSymbol) {
        MiddlewareSymbol sourceTag = connectorSymbol.getSourcePort().getMiddlewareSymbol().orElse(null);
        MiddlewareSymbol targetTag = connectorSymbol.getTargetPort().getMiddlewareSymbol().orElse(null);
        if (sourceTag == null || targetTag == null || !sourceTag.isKindOf(RosConnectionSymbol.KIND) || !targetTag.isKindOf(RosConnectionSymbol.KIND)) {
            Log.debug("Both sourcePort and targetPort need to have a RosConnectionSymbol", "RosConnectionSymbol");
            return;
        }
        RosConnectionSymbol rosConnectionA = (RosConnectionSymbol) sourceTag;
        RosConnectionSymbol rosConnectionB = (RosConnectionSymbol) targetTag;
        RosConnectionSymbol emptyRosConnection = new RosConnectionSymbol();
        if (!rosConnectionA.equals(emptyRosConnection) || !rosConnectionB.equals(emptyRosConnection)) {
            Log.debug("Will not override rosConnections that are not empty!", "RosConnectionSymbol");
            return;
        }

        //target port name is unique: each in port can only have one connection!
        String topicName = connectorSymbol.getTargetPort().getFullName().replace(".", "_");
        RosMsg rosTypeA = GeneratorRosMsg.getRosType("struct_msgs", connectorSymbol.getTargetPort().getTypeReference());
        RosMsg rosTypeB = GeneratorRosMsg.getRosType("struct_msgs", connectorSymbol.getSourcePort().getTypeReference());
        if (!rosTypeA.equals(rosTypeB)) {
            Log.error("topicType mismatch! "
                    + connectorSymbol.getSourcePort().getFullName() + " has " + rosTypeB + " and "
                    + connectorSymbol.getTargetPort().getFullName() + " has " + rosTypeA);
            return;
        }

        if (rosTypeA.getFields().size() == 1) {
            connectorSymbol.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol(topicName, rosTypeB.getName(), rosTypeB.getFields().get(0).getName()));
            connectorSymbol.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol(topicName, rosTypeA.getName(), rosTypeA.getFields().get(0).getName()));
        } else {
            connectorSymbol.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol(topicName, rosTypeB.getName()));
            connectorSymbol.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol(topicName, rosTypeA.getName()));
        }
    }

    private void inferRosConnectionIfPossible(PortSymbol sourcePort, PortSymbol targetPort) {
        MiddlewareSymbol sourceTag = sourcePort.getMiddlewareSymbol().orElse(null);
        MiddlewareSymbol targetTag = targetPort.getMiddlewareSymbol().orElse(null);
        if (sourceTag == null || targetTag == null || !sourceTag.isKindOf(RosConnectionSymbol.KIND) || !targetTag.isKindOf(RosConnectionSymbol.KIND)) {
            Log.debug("Both sourcePort and targetPort need to have a RosConnectionSymbol", "RosConnectionSymbol");
            return;
        }

        RosConnectionSymbol sourceRosConnection = (RosConnectionSymbol) sourceTag;
        RosConnectionSymbol targetRosConnection = (RosConnectionSymbol) targetTag;

        if (sourceRosConnection.getTopicName().isPresent() && !targetRosConnection.getTopicName().isPresent())
            targetRosConnection.setTopicName(sourceRosConnection.getTopicName().get());

        if (sourceRosConnection.getTopicType().isPresent() && !targetRosConnection.getTopicType().isPresent())
            targetRosConnection.setTopicType(sourceRosConnection.getTopicType().get());

        if (sourceRosConnection.getMsgField().isPresent() && !targetRosConnection.getMsgField().isPresent())
            targetRosConnection.setMsgField(sourceRosConnection.getMsgField().get());
    }

    //TODO: refactor
    private File generateCMake() throws IOException {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");
        StringBuilder content = new StringBuilder();
        content.append("cmake_minimum_required(VERSION 3.5)\n");
        //TODO setProjectName?
        content.append("project (default)\n");
        content.append("set (CMAKE_CXX_STANDARD 11)\n");

        subDirs.stream().filter(dir -> dir.equals("rosMsg")).forEach(
                dir -> content.append("add_subdirectory(" + dir + ")\n")
        );

        subDirs.stream().filter(dir -> !dir.equals("rosMsg")).forEach(
                dir -> content.append("add_subdirectory(" + dir + ")\n")
        );

        fileContent.setFileContent(content.toString());

        return FileHelper.generateFile(generationTargetPath, fileContent);
    }

}
