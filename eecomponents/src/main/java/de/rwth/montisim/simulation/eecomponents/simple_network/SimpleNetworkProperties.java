/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.time.Duration;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;

@Typed(SimpleNetworkProperties.TYPE)
public class SimpleNetworkProperties implements ModuleProperties {
    public static final String TYPE = "simple_network";
    public Duration transmission_time = Duration.ofMillis(5);
    public double car_transmission_range = 50;

    @Override
    public String getName() {
        return TYPE;
    }

    @Override
    public SimulatorModule build(BuildContext context) {
        DiscreteEventSimulator simulator = context.getObject(DiscreteEventSimulator.CONTEXT_KEY);
        return new SimpleNetwork(this, simulator);
    }
}
