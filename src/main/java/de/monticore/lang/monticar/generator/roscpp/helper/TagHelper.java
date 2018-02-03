package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticar.lang.monticar.generator.python.RosInterface;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.monticar.generator.roscpp.ResolvedRosTag;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class TagHelper {


    public static List<File> generate(GeneratorRosCpp generatorRosCpp, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol) throws IOException {
        List<PortSymbol> rosPorts = componentInstanceSymbol.getPorts().stream()
                .filter(p -> {
                            List<RosConnectionSymbol> tags = taggingResolver.getTags(p, RosConnectionSymbol.KIND).stream()
                                    .map(rcs -> (RosConnectionSymbol) rcs)
                                    .peek(tagSymbol -> {
                                        if (!tagSymbol.getMsgField().isPresent()) {
                                            Log.warn("RosConnectionSymbol without msgField found. Will be filtered out for generation!");
                                        }
                                    })
                                    .filter(tagSymol -> tagSymol.getMsgField().isPresent()).collect(Collectors.toList());
                            return !tags.isEmpty();
                        }
                )
                .collect(Collectors.toList());

        if (rosPorts.size() == 0) {
            throw new RuntimeException("No RosConnectionSymbols found for component " + componentInstanceSymbol.getName());
        }

        List<RosConnectionSymbol> rosConnections = rosPorts.stream()
                .flatMap(p -> taggingResolver.getTags(p, RosConnectionSymbol.KIND).stream())
                .map(rcs -> (RosConnectionSymbol) rcs)
                .collect(Collectors.toList());
        checkConsistency(rosConnections);

        RosTag rosTag = convertToRosTag(taggingResolver, componentInstanceSymbol, rosPorts);
        ResolvedRosTag resolvedRosTag = ResolveHelper.resolveRosTag(rosTag, taggingResolver);

        return generatorRosCpp.generateFiles(resolvedRosTag, taggingResolver);
    }

    private static RosTag convertToRosTag(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, List<PortSymbol> rosPorts) {
        RosTag res;

        res = new RosTag();
        res.component = componentInstanceSymbol.getFullName();

        rosPorts.stream()
                .filter(PortSymbol::isIncoming)
                .forEach(p -> {
                    List<RosConnectionSymbol> tags = taggingResolver.getTags(p, RosConnectionSymbol.KIND).stream()
                            .map(tagSymbol -> (RosConnectionSymbol) tagSymbol)
                            .collect(Collectors.toList());

                    assert (tags.size() == 1);

                    //Add subscriber
                    RosInterface rosInterface = new RosInterface();
                    rosInterface.type = tags.get(0).getTopicType();
                    rosInterface.topic = tags.get(0).getTopicName();
                    rosInterface.ports.put(p.getName(), tags.get(0).getMsgField().get());

                    res.subscriber.add(rosInterface);
                });

        rosPorts.stream()
                .filter(PortSymbol::isOutgoing)
                .forEach(p -> {
                    List<RosConnectionSymbol> tags = taggingResolver.getTags(p, RosConnectionSymbol.KIND).stream()
                            .map(tagSymbol -> (RosConnectionSymbol) tagSymbol)
                            .collect(Collectors.toList());

                    assert (tags.size() == 1);

                    //Add publisher
                    RosInterface rosInterface = new RosInterface();
                    rosInterface.type = tags.get(0).getTopicType();
                    rosInterface.topic = tags.get(0).getTopicName();
                    rosInterface.ports.put(p.getName(), tags.get(0).getMsgField().get());

                    res.publisher.add(rosInterface);
                });

        return res;
    }

    //TODO: as CoCo?
    private static void checkConsistency(List<RosConnectionSymbol> rosConnections) {
        //Topic types
        Map<String, String> nameToType = new HashMap<>();
        rosConnections.forEach(rcs -> {
            String curName = rcs.getTopicName();
            String curType = rcs.getTopicType();
            if (nameToType.containsKey(curName)) {
                if (!nameToType.get(curName).equals(curType))
                    throw new RuntimeException("Ros topics can have only one type but " + curName + " has types " + curType + " and " + nameToType.get(curName));
            } else {
                nameToType.put(curName, curType);
            }
        });
    }


}
