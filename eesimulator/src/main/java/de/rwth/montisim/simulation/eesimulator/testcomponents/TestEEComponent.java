/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.testcomponents;

import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;

/**
 * A Test component that can be fitted with ports and registers all incoming
 * events.
 */
public class TestEEComponent extends EEComponent {
    public List<MessageReceiveEvent> events = new ArrayList<>();

    public TestEEComponent(String name) {
        super(new TestCompProperties(name));
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.TEST_COMPONENT;
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        this.events.add(msgRecvEvent);
    }

    @Override
    protected void init() {
        // TODO Auto-generated method stub

    }
    
}