package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

import java.util.*;


public class DataHelper {
    private static final Map<PortSymbol, RosTopic> portToTopic = new HashMap<>();
    //TODO: replace with RosTopic.ports
    private static final Map<RosTopic, Set<PortSymbol>> topicToPorts = new HashMap<>();
    private static final Map<PortSymbol, String> portToMsgField = new HashMap<>();

    public static Set<PortSymbol> getPortsFromTopic(RosTopic rosTopic) {
        if (topicToPorts.containsKey(rosTopic)) {
            return topicToPorts.get(rosTopic);
        } else {
            return new HashSet<>();
        }
    }

    public static Optional<RosTopic> getTopicFromPort(PortSymbol portSymbol) {
        return Optional.ofNullable(portToTopic.get(portSymbol));
    }

    public static void addPortToTopic(PortSymbol portSymbol, RosTopic rosTopic, String msgField) {
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
        if (!getPortsFromTopic(rosTopic).contains(portSymbol)) {
            //create new set if needed
            if (topicToPorts.get(rosTopic) == null) {
                HashSet<PortSymbol> tmpSet = new HashSet<>();
                tmpSet.add(portSymbol);
                topicToPorts.put(rosTopic, tmpSet);
            } else {
                topicToPorts.get(rosTopic).add(portSymbol);
            }
            rosTopic.addPort(portSymbol);
        }


    }

    public static Set<RosTopic> getTopics() {
        return topicToPorts.keySet();
    }

    public static Set<PortSymbol> getPorts() {
        return portToTopic.keySet();
    }

    public static Optional<String> getMsgFieldFromPort(PortSymbol portSymbol) {
        return Optional.ofNullable(portToMsgField.get(portSymbol));
    }
}
