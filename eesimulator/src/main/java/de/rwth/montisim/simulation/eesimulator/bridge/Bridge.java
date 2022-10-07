/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bridge;

import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;

public class Bridge extends EEComponent {

    public final transient BridgeProperties properties;

    public Bridge(BridgeProperties properties, EESystem eesystem) {
        super(properties, eesystem);
        this.properties = properties;
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        this.eesystem.simulator.addEvent(
                new MessageSendEvent(
                        this,
                        msgRecvEvent.getEventTime().plus(properties.delay),
                        msgRecvEvent.getMessage()
                )
        );
    }

}