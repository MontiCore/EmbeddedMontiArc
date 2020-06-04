/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.autopilots;

import java.time.Duration;

import de.rwth.montisim.simulation.vehicle.componentbuilders.ComputerComponentBuilder;
import de.rwth.montisim.simulation.vehicle.componentbuilders.ComputerComponentProperties;

public class JavaAutopilotProperties extends ComputerComponentProperties {
    public static final String COMPUTER_TYPE = "JavaAutopilot";
    static {
        ComputerComponentBuilder.registerComputerBuilder(
            COMPUTER_TYPE, 
            (ComputerComponentProperties properties) -> new JavaAutopilot((JavaAutopilotProperties) properties)
        );
    }
    public Duration computeTime = Duration.ZERO;
    public double maxVehicleAccel;

    public JavaAutopilotProperties(double maxVehicleAccel) {
        super(COMPUTER_TYPE);
        this.maxVehicleAccel = maxVehicleAccel;
    }

    public JavaAutopilotProperties setName(String name){
        this.name = name;
        return this;
    }

    public JavaAutopilotProperties setComputeTime(Duration computeTime) {
        this.computeTime = computeTime;
        return this;
    }
    
}