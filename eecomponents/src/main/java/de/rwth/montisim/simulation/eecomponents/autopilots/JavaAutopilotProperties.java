/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;

@Typed(JavaAutopilotProperties.TYPE)
public class JavaAutopilotProperties extends EEComponentProperties {
    public static final String TYPE = "java_autopilot";
    static {
        VehicleBuilder.registerComponentBuilder(TYPE,
                (properties, context) -> new JavaAutopilot((JavaAutopilotProperties) properties));
    }
    public Duration compute_time = Duration.ZERO;
    public transient double maxVehicleAccel;

    public JavaAutopilotProperties(double maxVehicleAccel) {
        this.maxVehicleAccel = maxVehicleAccel;
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

}