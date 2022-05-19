/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import static org.junit.Assert.assertEquals;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.Test;

import com.vividsolutions.jts.util.Assert;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulatorManager;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.*;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.DirectSoftwareSimulatorManager;
import simulation.EESimulator.ControllerAsEEComponent;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EEDiscreteEventTypeEnum;
import simulation.EESimulator.EESimulator;
import simulation.bus.BusMessageEvent;
import simulation.bus.InstantBus;

public class ControllerTest {

    //hardcoded config
    private final ControllerConfig AUTOPILOT_CONFIG = new ControllerConfig(ControllerConfig.EmulatorType.HARDWARE_EMULATOR, "AutopilotAdapter")
            .set_os(ControllerConfig.OS.WINDOWS).set_timemodel_constant(Duration.ofMillis(60));

    @Test
    public void testController() throws Exception {
        SoftwareSimulatorManager softwareSimulatorManager = new DirectSoftwareSimulatorManager(new SoftwareSimulatorConfig().set_softwares_folder("autopilots"));
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createControllerSensors(bus);
        eeVehicleBuilder.createMassPointActuators(bus);
        ControllerAsEEComponent ecu = eeVehicleBuilder.createController(softwareSimulatorManager, AUTOPILOT_CONFIG, bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();

        //get actuators
        VehicleActuator steering = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).get();
        VehicleActuator brakeBL = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get();
        VehicleActuator brakeBR = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).get();
        VehicleActuator brakeFL = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).get();
        VehicleActuator brakeFR = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).get();
        VehicleActuator motor = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).get();

        HashMap<VehicleActuator, Double> initialValuesByActuator = new HashMap<VehicleActuator, Double>();
        initialValuesByActuator.put(steering, steering.getActuatorValueTarget());
        initialValuesByActuator.put(brakeBL, brakeBL.getActuatorValueTarget());
        initialValuesByActuator.put(brakeBR, brakeBR.getActuatorValueTarget());
        initialValuesByActuator.put(brakeFL, brakeFL.getActuatorValueTarget());
        initialValuesByActuator.put(brakeFR, brakeFR.getActuatorValueTarget());
        initialValuesByActuator.put(motor, motor.getActuatorValueTarget());

        vehicle.executeLoopIteration(Duration.ofMillis(30));

        //ecu emits 3 messages
        int controllerMessagesCount = 0;
        for (EEDiscreteEvent event : eeVehicle.getEESimulator().getEventList()) {
            if (event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
                BusMessageEvent msg = (BusMessageEvent) event;
                if (msg.getControllerID() == ecu.getId()) {
                    controllerMessagesCount++;
                }
            }
        }

        assertEquals(3, controllerMessagesCount);

        vehicle.executeLoopIteration(Duration.ofMillis(30));

        boolean initialValuesChanged = false;
        for (Map.Entry<VehicleActuator, Double> initialValueByActuator : initialValuesByActuator.entrySet()) {
            if (Math.abs(initialValueByActuator.getKey().getActuatorValueTarget() - initialValueByActuator.getValue()) > 0.001) {
                initialValuesChanged = true;
            }
        }

        Assert.isTrue(initialValuesChanged);
    }

}
