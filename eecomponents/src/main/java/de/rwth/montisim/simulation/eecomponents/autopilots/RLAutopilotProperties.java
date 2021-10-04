/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

@Typed(RLAutopilotProperties.TYPE)
public class RLAutopilotProperties extends EEComponentProperties {
    public static final String TYPE = "rl_autopilot";

    public Duration compute_time = Duration.ZERO;
    public double maxVehicleAccel; // TODO get from properties system

    public RLAutopilotProperties(double maxVehicleAccel) {
        this.maxVehicleAccel = maxVehicleAccel;
        this.name = "RLAutopilot";
    }

    protected RLAutopilotProperties() {
    }

    public RLAutopilotProperties setName(String name) {
        this.name = name;
        return this;
    }

    public RLAutopilotProperties setComputeTime(Duration computeTime) {
        this.compute_time = computeTime;
        return this;
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
        return new RLAutopilot(this, eesystem);
    }

}