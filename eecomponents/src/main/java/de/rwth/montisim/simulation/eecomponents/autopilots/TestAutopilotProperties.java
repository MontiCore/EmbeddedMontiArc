/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;

@Typed(TestAutopilotProperties.TYPE)
public class TestAutopilotProperties extends BusUserProperties {
    public static final String TYPE = "test_autopilot";
    public static final String COMPUTER_TYPE = "test_autopilot";
    static {
        VehicleBuilder.registerComponentBuilder(TYPE,
                (properties, context) -> new TestAutopilot((TestAutopilotProperties) properties));
    }

    public static enum Mode {
        @JsonEntry("circle")
        CIRCLE, @JsonEntry("start_stop")
        START_STOP
    }

    public Mode mode = Mode.CIRCLE;
    public Duration compute_time = Duration.ZERO;
    public double turn_angle = 10;
    public double target_velocity = 50; // km/h
    public transient double maxVehicleAccel;

    public TestAutopilotProperties() {
    }

    public TestAutopilotProperties setName(String name) {
        this.name = name;
        return this;
    }

    public TestAutopilotProperties(Mode mode, Duration computeTime, double maxVehicleAccel, double targetVelocity,
            double turnAngle) {
        this.mode = mode;
        this.compute_time = computeTime;
        this.maxVehicleAccel = maxVehicleAccel;
        this.target_velocity = targetVelocity;
        this.turn_angle = turnAngle;
    }

    public static TestAutopilotProperties circleAutopilot(Duration computeTime, double maxVehicleAccel,
            double targetVelocity, double turnAngle) {
        return new TestAutopilotProperties(Mode.CIRCLE, computeTime, maxVehicleAccel, targetVelocity, turnAngle);
    }

    public static TestAutopilotProperties startStopAutopilot(Duration computeTime, double maxVehicleAccel,
            double targetVelocity) {
        return new TestAutopilotProperties(Mode.START_STOP, computeTime, maxVehicleAccel, targetVelocity, 0);
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