/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import commons.controller.interfaces.Bus;
import commons.controller.interfaces.FunctionBlockInterface;
import commons.simulation.PhysicalObject;
import org.apache.commons.math3.analysis.function.Power;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.BeforeClass;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.powermock.api.mockito.PowerMockito;
import simulation.environment.object.House;
import simulation.environment.util.VehicleType;
import simulation.util.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import simulation.vehicle.Battery;
import simulation.vehicle.BatteryInterface;

public class BatteryTest {
	Vehicle v;
	Battery battery;

	@Before
	public void setAll() {
		v = new MassPointPhysicalVehicle().getSimulationVehicle();
		battery = new Battery(v, 1000);
		v.setBattery(battery);

        VehicleActuator steering = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        VehicleActuator throttle = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
        throttle.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MAX);

        VehicleActuator gear = v.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR);
        gear.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_GEAR_MAX);
	}
	
	// @Test
	// public void fullBattery() {
	// 	Assert.assertEquals(v.getBattery().getBatteryPercentage(), 100.0, 0.1);
	// }

	// @Test
	// public void testBatteryConsumption() {
	// 	v.setBattery(new Battery(v,1000));
	// 	v.getBattery().discharge();
	// 	Assert.assertEquals(900, v.getBattery().getCurrentCapacity(), 1.0);

	// 	v.getBattery().discharge();
	// 	Assert.assertEquals(800, v.getBattery().getCurrentCapacity(), 1.0);
	// }
	
	@Test
	public void testBatteryCharging_on_Battery() {
		battery.setPercentage(90);
		battery.charge(100.0, 1.0);
		assertEquals(1000, battery.getCurrentCapacity(), 1.0);
	}
	
	@Test
	public void testBatteryCharging_on_Vehicle() {
		battery.setPercentage(90);
		battery.charge(100.0, 1.0);
		assertEquals(1000, v.getBattery().get().getCurrentCapacity(), 1.0);
	}
	
	@Test
	public void testBatteryTimeToCharge(){
		battery.setPercentage(0.0);
		
		double t1 = battery.timeToCharge(10.0, 100.0, 1.0);
		assertEquals(t1, 1.0, 1.0);
		
		double t2 = battery.timeToCharge(20.0, 100.0, 1.0);
		assertEquals(t2, 2.0, 1.0);
		
		double t3 = battery.timeToCharge(30.0, 100.0, 1.0);
		assertEquals(t3, 3.0, 1.0);
		
//		battery.charge(100.0, 1.0);		
//		assertEquals(30, v.getBattery().get().getCurrentCapacity(), 1.0);
//		battery.timeToCharge(100.0, 100.0, 1.0);
	}
	
	@Test
	public void testDischarge_success() {
		battery.setPercentage(100.0);
		double c1 = battery.getCurrentCapacity();
		
		double consumption = battery.getBatteryConsumption_test_only();
		double remainCapacity = c1 - consumption;
		
		battery.discharge();
		double c2 = battery.getCurrentCapacity();
		
		assertEquals(remainCapacity, c2 , 0.1);
	}
	
	@Test(expected = IllegalArgumentException.class)
	public void testDischarge_fail() {
		battery.setPercentage(battery.getCriticalPercentage() - 1);
		battery.discharge();
	}
	
	
	
}
