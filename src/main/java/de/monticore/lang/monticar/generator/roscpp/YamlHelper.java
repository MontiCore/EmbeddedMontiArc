package de.monticore.lang.monticar.generator.roscpp;

import de.monticar.lang.monticar.generator.python.RosInterface;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

//TODO: better exceptions
public class YamlHelper {

    private static HashMap<String, RosTopic> uniqueTopics = new HashMap<>();

    public static void rosTagToDataHelper(Scope symtab, RosTag rosTag) {
        DataHelper.reset();
        uniqueTopics.clear();
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve(rosTag.component, ExpandedComponentInstanceSymbol.KIND)
                .orElseThrow(() -> new RuntimeException("Component " + rosTag.component + " could not be found!"));

        for (RosInterface sub : rosTag.subscriber) {

            //Add each port specified in the subscriber to its topic
            for (PortSymbol currentPort : resolvePorts(sub.ports.keySet(), componentInstanceSymbol)) {
                String msgField = sub.ports.get(currentPort.getName());
                if (currentPort.isOutgoing())
                    throw new RuntimeException("Only incoming ports can be used for subscribers but " + currentPort.getName() + " is outgoing!");
                DataHelper.addPortToTopic(currentPort, getRosTopicFromRosInterface(sub), msgField);
            }

        }

        for (RosInterface pub : rosTag.publisher) {

            //Add each port specified in the publisher to its topic
            for (PortSymbol currentPort : resolvePorts(pub.ports.keySet(), componentInstanceSymbol)) {
                String msgField = pub.ports.get(currentPort.getName());
                if (currentPort.isIncoming())
                    throw new RuntimeException("Only outgoing ports can be used for publishers but " + currentPort.getName() + " is incoming!");
                DataHelper.addPortToTopic(currentPort, getRosTopicFromRosInterface(pub), msgField);
            }

        }

    }

    private static List<PortSymbol> resolvePorts(Set<String> names, ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        List<PortSymbol> result = new ArrayList<>();
        names.forEach(name -> {
            result.add(componentInstanceSymbol.getPort(name)
                    .orElseThrow(() -> new RuntimeException("Port " + componentInstanceSymbol.getName() + "." + name + " not found")));
        });
        return result;
    }

    private static RosTopic getRosTopicFromRosInterface(RosInterface rosInterface) {
        if (uniqueTopics.containsKey(rosInterface.topic)) {
            //TODO: refactor, add more checks for valid config file
            if (!uniqueTopics.get(rosInterface.topic).getImportString().equals(rosInterface.type))
                throw new IllegalArgumentException("ROS topic can only have one type.");

            return uniqueTopics.get(rosInterface.topic);
        } else {

            String includeString = rosInterface.type;
            if (!includeString.contains("/"))
                throw new IllegalArgumentException("The ROS msg type has to be given in the form package/msgName!");
            String topicType = includeString.substring(includeString.lastIndexOf("/") + 1);
            RosTopic tmpTopic = new RosTopic(rosInterface.topic, topicType, includeString);
            uniqueTopics.put(rosInterface.topic, tmpTopic);
            return tmpTopic;
        }


    }

    public static List<File> generateFromFile(String configFilePath, TaggingResolver symtab, GeneratorRosCpp generatorRosCpp) throws IOException {
        TagReader<RosTag> reader = new TagReader<>();
        //TODO: fails silently, rewrite?
        List<RosTag> rosTags = reader.readYAML(configFilePath);
        List<File> result = new ArrayList<>();

        for (RosTag rosTag : rosTags) {
            rosTagToDataHelper(symtab, rosTag);
            ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve(rosTag.component, ExpandedComponentInstanceSymbol.KIND)
                    .orElseThrow(() -> new RuntimeException("ComponentInstance " + rosTag.component + " not found!"));

            result.addAll(generatorRosCpp.generateFiles(componentInstanceSymbol, symtab));
        }

        return result;
    }
}
