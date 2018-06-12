package de.monticore.lang.monticar.generator.roscpp.helper;

import com.esotericsoftware.yamlbeans.YamlReader;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.monticar.generator.roscpp.ResolvedRosTag;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.antlr.v4.runtime.misc.Pair;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

//TODO: better exceptions
public class YamlHelper {

    private YamlHelper() {
    }

    public static List<File> generateFromFile(String configFilePath, TaggingResolver symtab, GeneratorRosCpp generatorRosCpp) throws IOException {
        YamlReader reader = new YamlReader(new FileReader(configFilePath));
        List<RosTag> rosTags = (List<RosTag>) reader.read();
        List<File> result = new ArrayList<>();

        for (RosTag rosTag : rosTags) {
            fixPortSyntax(rosTag, symtab);
            fixMsgFieldIndices(rosTag);
            ResolvedRosTag resolvedRosTag = ResolveHelper.resolveRosTag(rosTag, symtab);
            Stream.concat(resolvedRosTag.getSubscriberInterfaces().stream(), resolvedRosTag.getPublisherInterfaces().stream()).
                    forEach(inter -> {
                        inter.getPorts().forEach(p -> {
                            String topicName = inter.getTopic();
                            String topicType = inter.getInclude();
                            String msgField = inter.getMsgFieldForPort(p);
                            p.setMiddlewareSymbol(new RosConnectionSymbol(topicName, topicType, msgField));
                        });
                    });
            result.addAll(generatorRosCpp.generateFiles(resolvedRosTag.getComponent(), symtab));
        }

        return result;
    }

    private static void fixPortSyntax(RosTag rosTag, TaggingResolver symtab) {
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve(rosTag.component, ExpandedComponentInstanceSymbol.KIND)
                .orElseThrow(() -> new RuntimeException("Component " + rosTag.component + " could not be found!"));

        //[:] syntax
        Stream.concat(rosTag.subscriber.stream(), rosTag.publisher.stream())
                .forEach(inter -> {
                    Set<Pair<String, String>> newPortsAndMsgFields = new HashSet<>();

                    inter.ports.forEach((portName, msgField) -> {
                        if (portName.contains("[:]")) {
                            newPortsAndMsgFields.addAll(fixPortWithColonSyntax(inter.ports, portName, component));
                        }
                    });

                    inter.ports.entrySet().removeIf(entry -> entry.getKey().contains("[:]"));

                    newPortsAndMsgFields.forEach(pair -> inter.ports.put(pair.a, pair.b));

                });
    }

    private static Set<Pair<String, String>> fixPortWithColonSyntax(Map<String, String> portNameToMsgField, String portName, ExpandedComponentInstanceSymbol component) {
        assert (portName.contains("[:]"));
        assert (portNameToMsgField.containsKey(portName));

        String msgField = portNameToMsgField.get(portName);

        Set<Pair<String, String>> res = new HashSet<>();

//        portNameToMsgField.remove(portName);

        int i = 1;
        PortSymbol curPort = component.getPort(portName.replace("[:]", "[" + i + "]"))
                .orElseThrow(() -> new RuntimeException("No Ports matching " + portName + " found in component " + component.getName()));
        while (curPort != null) {
            String curMsgField = msgField.replace("[:]", "[" + i + "]");
            res.add(new Pair<>(curPort.getName(), curMsgField));
            i++;
            curPort = component.getPort(portName.replace("[:]", "[" + i + "]")).orElse(null);
        }

        return res;
    }

    //TODO: hard to read
    //EMAM and the yaml config files are 1 index based, cpp is 0 index based
    private static void fixMsgFieldIndices(RosTag rosTag) {
        Stream.concat(rosTag.publisher.stream(), rosTag.subscriber.stream())
                .forEach(inter -> {
                    inter.ports = new HashMap<>(
                            inter.ports.entrySet().stream()
                                    .collect(Collectors.toMap(Map.Entry::getKey,
                                            e -> e.getValue().contains("[") && e.getValue().contains("]") ? NameHelper.getFixedMsgFieldName(e.getValue()) : e.getValue())));
                });
    }
}
