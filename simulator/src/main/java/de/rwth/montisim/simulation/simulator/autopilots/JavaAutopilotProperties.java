/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
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

    public JavaAutopilotProperties() {
        super(COMPUTER_TYPE);
        // TODO Auto-generated constructor stub
    }
    
}