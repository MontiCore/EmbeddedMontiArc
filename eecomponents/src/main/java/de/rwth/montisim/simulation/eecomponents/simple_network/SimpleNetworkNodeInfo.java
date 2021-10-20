/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import de.rwth.montisim.simulation.commons.StaticObject;

public class SimpleNetworkNodeInfo {
    public String nodeName;
    public IPV6Address address;
    public StaticObject physicalObject;
    public SimpleCommunicationGateway component;

    public SimpleNetworkNodeInfo(String nodeName, IPV6Address address, StaticObject physicalObject, SimpleCommunicationGateway component) {
        this.nodeName = nodeName;
        this.address = address;
        this.physicalObject = physicalObject;
        this.component = component;
    }
}
