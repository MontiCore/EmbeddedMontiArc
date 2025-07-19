/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network.events;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;

public class SimpleNetworkComCloseEvent extends DiscreteEvent {
    public static final int type = registerType(SimpleNetworkComCloseEvent.class);

    @Override
    public int getType() {
        return type;
    }

}
