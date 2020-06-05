/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.simulation.vehicle.componentbuilders.*;

public class TestAutopilotProperties extends ComputerComponentProperties {
    public static final String COMPUTER_TYPE = "TestAutopilot";
    static {
        ComputerComponentBuilder.registerComputerBuilder(
            COMPUTER_TYPE, 
            (ComputerComponentProperties properties) -> new TestAutopilot((TestAutopilotProperties) properties)
        );
    }
    public static enum Mode {
        CIRCLE, START_STOP
    }
    public Mode mode = Mode.CIRCLE;
    public Duration computeTime = Duration.ZERO;
    public double turnAngle = 10;
    public double targetVelocity = 50; // km/h
    public double maxVehicleAccel;

    public TestAutopilotProperties() {
        super(COMPUTER_TYPE);
    }

    public TestAutopilotProperties setName(String name){
        this.name = name;
        return this;
    }

    public TestAutopilotProperties(Mode mode, Duration computeTime, double maxVehicleAccel,
    double targetVelocity, double turnAngle) {
        super(COMPUTER_TYPE);
        this.mode = mode;
        this.computeTime = computeTime;
        this.maxVehicleAccel = maxVehicleAccel;
        this.targetVelocity = targetVelocity;
        this.turnAngle = turnAngle;
    }

    
    public static TestAutopilotProperties circleAutopilot(Duration computeTime, double maxVehicleAccel,
            double targetVelocity, double turnAngle) {
        return new TestAutopilotProperties(Mode.CIRCLE, computeTime, maxVehicleAccel, targetVelocity, turnAngle);
    }

    public static TestAutopilotProperties startStopAutopilot(Duration computeTime,
            double maxVehicleAccel, double targetVelocity) {
        return new TestAutopilotProperties(Mode.START_STOP, computeTime, maxVehicleAccel, targetVelocity, 0);
    }
    
}