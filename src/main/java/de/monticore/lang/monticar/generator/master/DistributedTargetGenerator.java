package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.*;

public class DistributedTargetGenerator extends CMakeMasterGenerator {
    private RosMsgImpl rosMsgImpl;

    public DistributedTargetGenerator(String generationTargetPath) {
        setGenerationTargetPath(generationTargetPath);
        rosMsgImpl = new RosMsgImpl("rosmsg");
        //this.add(rosMsgImpl,"rosmsg/");
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        Map<PortSymbol, RosConnectionSymbol> resolvedTags = TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        Map<ExpandedComponentInstanceSymbol, GeneratorImpl> generatorMap = new HashMap<>();

        fixComponentInstance(componentInstanceSymbol);
        Collection<ExpandedComponentInstanceSymbol> subComps = getSubComponentInstances(componentInstanceSymbol);

        //TODO: cluster non mw subcomps together with mw subcomps

        boolean allSubsMwOnly = subComps.stream()
                .flatMap(comp -> comp.getPorts().stream())
                .allMatch(p -> p.getMiddlewareSymbol().isPresent());

        if (allSubsMwOnly) {
            subComps.forEach(comp ->
                    generatorMap.put(comp, createFullGenerator(comp.getFullName().replace(".", "_")))
            );
        } else {
            generatorMap.put(componentInstanceSymbol, createFullGenerator(componentInstanceSymbol.getFullName().replace(".", "_")));
        }

        List<File> files = new ArrayList<>();

        CMakeMasterGenerator cmakeListsGenerator = new CMakeMasterGenerator();
        cmakeListsGenerator.setGenerationTargetPath(generationTargetPath);
        for (ExpandedComponentInstanceSymbol comp : generatorMap.keySet()) {
            files.addAll(generatorMap.get(comp).generate(comp, taggingResolver));
            //add empty generator to cmakeListsGenerator so that CMakeLists.txt will be generated
            cmakeListsGenerator.add(new GeneratorImpl() {
            }, comp.getFullName().replace(".", "_"));
        }

        files.addAll(cmakeListsGenerator.generate(componentInstanceSymbol, taggingResolver));
        return files;
    }

    private GeneratorImpl createFullGenerator(String subdir) {
        MiddlewareMasterGenerator res = new MiddlewareMasterGenerator();
        res.setGenerationTargetPath(generationTargetPath + (subdir.endsWith("/") ? subdir : subdir + "/"));

        this.getGeneratorImpls().forEach(gen -> res.add(gen, this.getImplSubfolder(gen)));

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
                    } else if (Objects.equals(connectorSymbol.getTargetPort().getComponentInstance().orElse(null), componentInstanceSymbol)) {
                        //out port of supercomp
                        inferRosConnectionIfPossible(connectorSymbol.getTargetPort(), connectorSymbol.getSourcePort());
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
        RosMsg rosTypeA = rosMsgImpl.getRosType(connectorSymbol.getTargetPort().getTypeReference());
        RosMsg rosTypeB = rosMsgImpl.getRosType(connectorSymbol.getSourcePort().getTypeReference());
        if (!rosTypeA.equals(rosTypeB)) {
            Log.error("topicType mismatch! "
                    + connectorSymbol.getSourcePort().getFullName() + " has " + rosTypeB + " and "
                    + connectorSymbol.getTargetPort().getFullName() + " has " + rosTypeA);
            return;
        }

        rosMsgImpl.addRosTypeToGenerate(connectorSymbol.getTargetPort().getTypeReference());
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

    private Collection<ExpandedComponentInstanceSymbol> getSubComponentInstances(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        //TODO: check which subcomponents are mw only
        //TODO: (build clusters?)
        return componentInstanceSymbol.getSubComponents();
    }


}
