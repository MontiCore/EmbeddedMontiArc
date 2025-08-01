/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;

/**
 * A Test component that can be fitted with ports and registers all incoming
 * events.
 */
public class TestEEComponent extends EEComponent {
    public transient List<MessageReceiveEvent> events = new ArrayList<>();
    transient TestCompProperties properties;

    public TestEEComponent(TestCompProperties properties, EESystem eesystem) {
        super(properties, eesystem);
        this.properties = properties;
        for (PortInformation inf : properties.ports) {
            addPort(inf);
        }
    }

    public TestEEComponent(String name, EESystem eesystem) {
        super(new TestCompProperties(name), eesystem);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        this.events.add(msgRecvEvent);
    }

}