package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

import java.util.*;


public class DataHelper {
    private static final Set<RosTopic> topics = new HashSet<>();
    private static final Map<PortSymbol, RosTopic> portToTopic = new HashMap<>();
    private static final Map<PortSymbol, String> portToMsgField = new HashMap<>();

    public static Optional<RosTopic> getTopicFromPort(PortSymbol portSymbol) {
        return Optional.ofNullable(portToTopic.get(portSymbol));
    }

    public static void addPortToTopic(PortSymbol portSymbol, RosTopic rosTopic, String msgField) {
        if (!topics.contains(rosTopic))
            topics.add(rosTopic);
        //only one topic per port
        if (!getTopicFromPort(portSymbol).isPresent()) {
            portToTopic.put(portSymbol, rosTopic);
        } else {
            throw new IllegalArgumentException("More then one topic is defined for port " + portSymbol.getName());
        }
        //only one msgField per port
        if (!getMsgFieldFromPort(portSymbol).isPresent()) {
            portToMsgField.put(portSymbol, msgField);
        } else {
            throw new IllegalArgumentException("More then one msgField is defined for port " + portSymbol.getName());
        }

        //only add new ports to topics
        if (!rosTopic.getPorts().contains(portSymbol)) {
            rosTopic.addPort(portSymbol);
        }


    }

    public static Set<RosTopic> getTopics() {
        return topics;
    }

    public static Set<PortSymbol> getPorts() {
        return portToTopic.keySet();
    }

    public static Optional<String> getMsgFieldFromPort(PortSymbol portSymbol) {
        return Optional.ofNullable(portToMsgField.get(portSymbol));
    }

    public static void reset() {
        portToTopic.clear();
        portToMsgField.clear();
        topics.clear();
    }
}
