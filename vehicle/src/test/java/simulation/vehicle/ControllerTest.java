/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import static org.junit.Assert.assertEquals;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.Test;

import com.vividsolutions.jts.util.Assert;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;
import simulation.EESimulator.DirectModelAsEEComponent;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EEDiscreteEventTypeEnum;
import simulation.EESimulator.EESimulator;
import simulation.bus.BusMessage;

public class ControllerTest {

	//hardcoded config
	private final String AUTOPILOT_CONFIG = "autopilot=AutopilotAdapter\n"+
												"os=windows\n" + 
												"no_time=true";
	
	@Test
	public void testController() throws Exception {
		HardwareEmulatorInterface modelServer = new HardwareEmulatorInterface("autopilots_folder=autopilots", "");
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getVehicle();
        EEVehicle eeVehicle = vehicle.getEEVehicle();
        EESimulator eeSimulator = eeVehicle.getEESimulator();
        
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


        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        for(BusEntry busEntry : DirectModelAsEEComponent.MASSPOINT_OUTPUT_MESSAGES) {
        	targetsByMessageId.put(busEntry, Collections.singletonList(eeVehicle.getBusList().get(0)));
        }
		DirectModelAsEEComponent controller = new DirectModelAsEEComponent(modelServer, AUTOPILOT_CONFIG, eeVehicle.getEESimulator(), targetsByMessageId);
		eeVehicle.getBusList().get(0).registerComponent(controller);
		vehicle.executeLoopIteration(Instant.EPOCH.plusMillis(30));
		
		vehicle.executeLoopIteration(Instant.EPOCH.plusMillis(30));
		boolean initialValuesChanged = false;
		for(Map.Entry<VehicleActuator, Double> initialValueByActuator : initialValuesByActuator.entrySet()) {
			if(Math.abs(initialValueByActuator.getKey().getActuatorValueTarget()-initialValueByActuator.getValue()) > 0.001) {
				initialValuesChanged = true;
			}
		}
		
		Assert.isTrue(initialValuesChanged);
	}

}
