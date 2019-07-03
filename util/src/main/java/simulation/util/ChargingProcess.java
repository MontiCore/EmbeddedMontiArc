package simulation.util;

import rwth.server.simulation.Car;
import simulation.environment.object.ChargingStation;

/**
 * Charging Station Class
 *
 * Questions: carObjectArray AND inUse?
 * 			  PhysicalObject has its own idGenerator
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingProcess {
	private Car car;
	private ChargingStation cs;
	private long chargingTimeMillis;

	public boolean initCharging(Car car, ChargingStation cs){
		this.car = car;
		this.cs = cs;
		//start car's Battery's chargingMethod and get the chargingTimeMillis and
		//consumption from it
		cs.startCharging(car, chargingTimeMillis);
	}

	public boolean endCharging(Car car, long consumption){
		cs.stopCharging(car, consumption);
	}


	public Car getCar() {
		return car;
	}

	public void setCar(Car car) {
		this.car = car;
	}

	public ChargingStation getCs() {
		return cs;
	}

	public void setCs(ChargingStation cs) {
		this.cs = cs;
	}
}
