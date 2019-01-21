package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.FileHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.AbstractCollection;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class MiddlewareTagGenImpl implements GeneratorImpl {

    private String generationTargetPath;

    @Override
    public List<File> generate(EMAComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> res = new ArrayList<>();

        //Collect Ports with Middleware Symbols from Super Component and first level subcomponents
        List<EMAPortInstanceSymbol> middlewarePortsSuper = componentInstanceSymbol.getPortInstanceList().stream()
                .filter(portSymbol -> portSymbol.getMiddlewareSymbol().isPresent())
                .collect(Collectors.toList());

        List<EMAPortInstanceSymbol> middlewarePortsSub = componentInstanceSymbol.getSubComponents()
                .stream()
                .flatMap(ecis -> ecis.getPortInstanceList().stream())
                .filter(portSymbol -> portSymbol.getMiddlewareSymbol().isPresent())
                .collect(Collectors.toList());

        List<EMAPortInstanceSymbol> middlewarePorts = new ArrayList<>();
        middlewarePorts.addAll(middlewarePortsSub);
        middlewarePorts.addAll(middlewarePortsSuper);

        res.add(generateRosTags(componentInstanceSymbol.getPackageName(),middlewarePorts));

        return res;
    }

    private File generateRosTags(String packageName ,List<EMAPortInstanceSymbol> middlewarePorts) throws IOException {
        List<EMAPortInstanceSymbol> rosPorts = middlewarePorts.stream()
                .filter(EMAPortInstanceSymbol::isRosPort)
                .collect(Collectors.toList());

        FileContent result = new FileContent();

        result.setFileName("RosConnections.tag");

        StringBuilder content = new StringBuilder();
        content.append("package " + packageName +";\n");
        content.append("conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;\n");

        content.append("\n");
        content.append("tags RosConnections{\n");

        //Pro port: tag ${port.fullName} with RosConnection (; | topic=(${topicName},${topicType}) , (msgField=${msgField})?);
        rosPorts.forEach(p -> {
            content.append("tag " + p.getFullName() + " with RosConnection");
            RosConnectionSymbol rosSymbol = (RosConnectionSymbol) p.getMiddlewareSymbol().get();
            if(rosSymbol.getTopicName().isPresent() && rosSymbol.getTopicType().isPresent()){
                content.append(" = {topic = ("+ rosSymbol.getTopicName().get() +", " +rosSymbol.getTopicType().get() + ")");

                if(rosSymbol.getMsgField().isPresent()){
                    content.append(", msgField = " + rosSymbol.getMsgField().get());
                }

                content.append("}");
            }
            content.append(";\n");

        });
        content.append("}");

        result.setFileContent(content.toString());
        return FileHelper.generateFile(generationTargetPath ,result);
    }

    @Override
    public void setGenerationTargetPath(String path) {
        generationTargetPath = path;
    }

    @Override
    public boolean willAccept(EMAComponentInstanceSymbol componentInstanceSymbol) {
        //TODO: fill?
        return true;
    }
}
