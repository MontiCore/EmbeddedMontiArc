/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

@Typed(JavaAutopilotProperties.TYPE)
public class JavaAutopilotProperties extends EEComponentProperties {
    public static final String TYPE = "java_autopilot";

    public Duration compute_time = Duration.ZERO;
    public double maxVehicleAccel; // TODO get from properties system

    public JavaAutopilotProperties(double maxVehicleAccel) {
        this.maxVehicleAccel = maxVehicleAccel;
        this.name = "JavaAutopilot";
    }

    protected JavaAutopilotProperties() {
    }

    public JavaAutopilotProperties setName(String name) {
        this.name = name;
        return this;
    }

    public JavaAutopilotProperties setComputeTime(Duration computeTime) {
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
        return new JavaAutopilot(this, eesystem);
    }

}