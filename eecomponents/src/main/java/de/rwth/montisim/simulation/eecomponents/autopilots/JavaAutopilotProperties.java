/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(JavaAutopilotProperties.TYPE)
public class JavaAutopilotProperties extends BusUserProperties {
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
    public EEEventProcessor build(ComponentBuildContext context) {
        return new JavaAutopilot(this);
    }

}