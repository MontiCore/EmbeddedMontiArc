/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.helpers;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.FileContent;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TagHelper {
    private TagHelper(){

    }


    public static String getFileContent(EMAComponentInstanceSymbol componentInstanceSymbol, List<Set<EMAComponentInstanceSymbol>> clustering){
        List<EMAConnectorInstanceSymbol> affectedConnectors = AutomaticClusteringHelper.getInterClusterConnectors(componentInstanceSymbol, clustering);
        List<EMAPortInstanceSymbol> affectedPorts = affectedConnectors.stream().flatMap(c -> Stream.of(c.getSourcePort(), c.getTargetPort())).collect(Collectors.toList());
        return generateRosTags(componentInstanceSymbol.getPackageName(), affectedPorts, true).getFileContent();
    }

    private static FileContent generateRosTags(String packageName , List<EMAPortInstanceSymbol> affectedPorts, boolean ignoreOldValues){
        FileContent result = new FileContent();

        List<EMAPortInstanceSymbol> rosPorts;
        if(!ignoreOldValues) {
            rosPorts = affectedPorts.stream()
                    .filter(EMAPortInstanceSymbol::isRosPort)
                    .collect(Collectors.toList());
        }else{
            rosPorts = affectedPorts;
        }

        result.setFileName("RosConnections.tag");

        StringBuilder content = new StringBuilder();
        content.append("package " + packageName +";\n");
        content.append("conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;\n");

        content.append("\n");
        content.append("tags RosConnections{\n");

        //Pro port: tag ${port.fullName} with RosConnection (; | topic=(${topicName},${topicType}) , (msgField=${msgField})?);
        rosPorts.forEach(p -> {
            content.append("tag " + p.getFullName() + " with RosConnection");
            if(!ignoreOldValues) {
                RosConnectionSymbol rosSymbol = (RosConnectionSymbol) p.getMiddlewareSymbol().get();
                if (rosSymbol.getTopicName().isPresent() && rosSymbol.getTopicType().isPresent()) {
                    content.append(" = {topic = (" + rosSymbol.getTopicName().get() + ", " + rosSymbol.getTopicType().get() + ")");

                    if (rosSymbol.getMsgField().isPresent()) {
                        content.append(", msgField = " + rosSymbol.getMsgField().get());
                    }

                    content.append("}");
                }
            }
            content.append(";\n");

        });
        content.append("}");

        result.setFileContent(content.toString());
        return result;
    }
}
