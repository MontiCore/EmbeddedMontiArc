/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;

/**
 * A Test component that can be fitted with ports and registers all incoming
 * events.
 */
public class TestEEComponent extends EEComponent {
    public List<MessageReceiveEvent> events = new ArrayList<>();

    public TestEEComponent(EESimulator simulator, String name) {
        super(simulator, name);
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.TEST_COMPONENT;
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        this.events.add(msgRecvEvent);
    }
    
}