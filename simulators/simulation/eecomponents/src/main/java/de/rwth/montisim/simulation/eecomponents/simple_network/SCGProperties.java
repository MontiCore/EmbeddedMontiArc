/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.time.Duration;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(SCGProperties.TYPE)
public class SCGProperties extends EEComponentProperties {
    public static final String TYPE = "simple_communication_gateway";
    public static final String COMPONENT_NAME = "SimpleCommunicationGateway";

    public Duration processing_time = Duration.ofMillis(1);

    public SCGProperties() {
        this.name = COMPONENT_NAME;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.COMPUTER;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException {
        SimpleNetwork network = context.getObject(SimpleNetwork.CONTEXT_KEY);
        Vehicle vehicle = context.getObject(Vehicle.CONTEXT_KEY);
        return new SimpleCommunicationGateway(this, eesystem, network, vehicle);
    }

}
